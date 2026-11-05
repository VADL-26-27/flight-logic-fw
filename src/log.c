#include "log.h"
#include <string.h>

/* Sector 0 is reserved for the existing flash bench test. */
#define LOG_START FLASH_SECTOR_SIZE
#define LOG_MAGIC 0x31474F4CU /* "LOG1" in little-endian byte order. */
#define RECORD_BYTES 48U
#define BLOCK_RECORDS 80U

typedef enum {
  BLOCK_HEADER = 1,
  BLOCK_GROUND,
  BLOCK_FLIGHT,
  BLOCK_FOOTER
} BlockKind;

typedef enum {
  LOGGER_PROBING,
  LOGGER_CHECKING_SECTOR,
  LOGGER_READY,
  LOGGER_WRITING,
  LOGGER_CLEAR_SCANNING,
  LOGGER_CLEAR_ERASING,
  LOGGER_OCCUPIED,
  LOGGER_DONE,
  LOGGER_FULL,
  LOGGER_ERROR
} LoggerStatus;

_Static_assert(sizeof(float) == 4U, "Log format requires 32-bit floats");
_Static_assert(sizeof(IMURecord) == RECORD_BYTES, "Check record buffer sizing");
_Static_assert(24U + BLOCK_RECORDS * RECORD_BYTES + 4U <= FLASH_SECTOR_SIZE,
               "A log block must fit in one sector");

static PreLaunchBuf ground;
static FlightBuf flight[2];
static uint8_t fill_index;
static uint8_t write_index;
static uint32_t flight_offset;
static uint32_t ground_index;
static uint32_t ground_remaining;
static bool started;
static bool finishing;

static flash_t *storage;
static volatile LoggerStatus logger_status; /* Inspect in debugger. */
static volatile uint32_t dropped_records;
static uint32_t sector = LOG_START;
static bool clear_cancelled;
static uint32_t block_sequence;
static uint32_t records_written;
static uint8_t block[FLASH_SECTOR_SIZE];
static BlockKind pending_kind;
static uint32_t pending_records;

typedef enum {
  DUMP_BEGIN,
  DUMP_READ,
  DUMP_WAIT,
  DUMP_TAG,
  DUMP_DATA,
  DUMP_END,
  DUMP_ERROR
} DumpStage;

static DumpStage dump_stage;
static uint32_t dump_address;
static uint32_t dump_offset;

static void put_word(uint32_t *length, uint32_t value) {
  for (uint32_t shift = 0; shift < 32U; shift += 8U)
    block[(*length)++] = (uint8_t)(value >> shift);
}

static void put_record(uint32_t *length, const IMURecord *record) {
  const float values[] = {
      record->rawPacket.yaw,      record->rawPacket.pitch,
      record->rawPacket.roll,     record->rawPacket.ax,
      record->rawPacket.ay,       record->rawPacket.az,
      record->rawPacket.pressure, record->postKf.altitude,
      record->postKf.velocity,    record->postKf.acceleration,
  };
  put_word(length, record->timestamp_ms);
  for (uint32_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
    uint32_t bits;
    memcpy(&bits, &values[i], sizeof(bits));
    put_word(length, bits);
  }
  put_word(length, (uint32_t)record->flightState);
}

static uint32_t block_crc(uint32_t length) {
  uint32_t crc = UINT32_MAX;
  for (uint32_t i = 0; i < length; ++i) {
    crc ^= block[i];
    for (uint32_t bit = 0; bit < 8U; ++bit)
      crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
  }
  return ~crc;
}

static void write_block(BlockKind kind, uint32_t count, uint32_t info) {
  uint32_t length = 0;
  /* Six little-endian words, records for data blocks, then CRC-32.
   * info: standby record count in the header, dropped records in the footer.
   * count: payload record count for data, total stored records for the footer.
   */
  put_word(&length, LOG_MAGIC);
  put_word(&length, kind);
  put_word(&length, LOG_START);
  put_word(&length, block_sequence);
  put_word(&length, count);
  put_word(&length, info);
  if (kind == BLOCK_GROUND) {
    for (uint32_t i = 0; i < count; ++i)
      put_record(&length,
                 &ground.records[(ground_index + i) % PRELAUNCH_RECORDS]);
  } else if (kind == BLOCK_FLIGHT) {
    for (uint32_t i = 0; i < count; ++i)
      put_record(&length, &flight[write_index].records[flight_offset + i]);
  }
  put_word(&length, block_crc(length));

  if (!flash_program(storage, sector, block, length)) {
    logger_status = LOGGER_ERROR;
    return;
  }
  pending_kind = kind;
  pending_records = count;
  if (kind == BLOCK_FLIGHT)
    flight[write_index].state = LOG_BUF_WRITING;
  logger_status = LOGGER_WRITING;
}

static void write_complete(void) {
  if (pending_kind == BLOCK_FOOTER) {
    logger_status = LOGGER_DONE;
    return;
  }
  if (pending_kind == BLOCK_GROUND) {
    ground_index = (ground_index + pending_records) % PRELAUNCH_RECORDS;
    ground_remaining -= pending_records;
    records_written += pending_records;
  } else if (pending_kind == BLOCK_FLIGHT) {
    FlightBuf *buffer = &flight[write_index];
    flight_offset += pending_records;
    records_written += pending_records;
    if (flight_offset == buffer->count) {
      buffer->count = 0;
      buffer->state = LOG_BUF_FREE;
      flight_offset = 0;
      write_index ^= 1U;
    }
  }
  ++block_sequence;
  sector += FLASH_SECTOR_SIZE;
  logger_status = LOGGER_CHECKING_SECTOR;
}

void logger_init(flash_t *device) {
  storage = device;
  logger_status = flash_probe(storage) ? LOGGER_PROBING : LOGGER_ERROR;
}

bool logger_request_clear(void) {
  if (!storage->initialized || storage->busy ||
      storage->result != FLASH_RESULT_NONE)
    return false;

  switch (logger_status) {
  case LOGGER_READY:
    if (started)
      return false;
    break;
  case LOGGER_OCCUPIED:
  case LOGGER_DONE:
  case LOGGER_FULL:
  case LOGGER_ERROR:
    break;
  default:
    return false;
  }

  sector = LOG_START;
  clear_cancelled = false;
  logger_status = LOGGER_CLEAR_SCANNING;
  return true;
}

const char *logger_status_reply(void) {
  switch (logger_status) {
  case LOGGER_READY:
    return started ? "LOG RECORDING\r\n" : "LOG READY\r\n";
  case LOGGER_WRITING:
    return "LOG RECORDING\r\n";
  case LOGGER_PROBING:
  case LOGGER_CHECKING_SECTOR:
    return "LOG CHECKING\r\n";
  case LOGGER_CLEAR_SCANNING:
  case LOGGER_CLEAR_ERASING:
    return "LOG CLEARING\r\n";
  case LOGGER_OCCUPIED:
    return "LOG OCCUPIED\r\n";
  case LOGGER_DONE:
    return "LOG COMPLETE\r\n";
  case LOGGER_FULL:
    return "LOG FULL\r\n";
  default:
    return "LOG ERROR\r\n";
  }
}

static void clear_complete(void) {
  flight[0].count = flight[1].count = 0;
  flight[0].state = flight[1].state = LOG_BUF_FREE;
  fill_index = write_index = 0;
  flight_offset = block_sequence = records_written = dropped_records = 0;
  started = finishing = false;
  logger_status = LOGGER_READY;
}

static bool sector_is_blank(void) {
  for (uint32_t i = 0; i < sizeof(block); ++i) {
    if (block[i] != 0xFFU)
      return false;
  }
  return true;
}

bool logger_request_dump(void) {
  if (!storage->initialized || storage->busy ||
      storage->result != FLASH_RESULT_NONE || logger_status < LOGGER_OCCUPIED)
    return false;

  dump_address = LOG_START;
  dump_stage = DUMP_BEGIN;
  return true;
}

/* Wire format: banner, 'D' + 4096 bytes per sector, then 'E' or 'X'. */
bool logger_dump_poll(bool (*send)(const uint8_t *, uint32_t)) {
  static const uint8_t banner[] = "\r\nLOG DUMP 1\r\n";
  switch (dump_stage) {
  case DUMP_BEGIN:
    if (send(banner, sizeof(banner) - 1U))
      dump_stage = DUMP_READ;
    break;
  case DUMP_READ:
    if (dump_address == FLASH_CAPACITY_BYTES)
      dump_stage = DUMP_END;
    else if (flash_read(storage, dump_address, block, sizeof(block)))
      dump_stage = DUMP_WAIT;
    else
      dump_stage = DUMP_ERROR;
    break;
  case DUMP_WAIT:
    if (!storage->busy) {
      bool ok = storage->result == FLASH_RESULT_OK;
      if (!flash_ack_result(storage) || !ok)
        dump_stage = DUMP_ERROR;
      else
        dump_stage = sector_is_blank() ? DUMP_END : DUMP_TAG;
    }
    break;
  case DUMP_TAG:
    if (send((const uint8_t *)"D", 1U)) {
      dump_offset = 0;
      dump_stage = DUMP_DATA;
    }
    break;
  case DUMP_DATA: {
    uint32_t length = sizeof(block) - dump_offset;
    if (length > 64U)
      length = 64U;
    if (send(block + dump_offset, length)) {
      dump_offset += length;
      if (dump_offset == sizeof(block)) {
        dump_address += FLASH_SECTOR_SIZE;
        dump_stage = DUMP_READ;
      }
    }
    break;
  }
  case DUMP_END:
    return send((const uint8_t *)"E", 1U);
  case DUMP_ERROR:
    return send((const uint8_t *)"X", 1U);
  }
  return false;
}

void logger_push_ground(const IMURecord *record) {
  ground.records[ground.next_write] = *record;
  ground.next_write = (ground.next_write + 1U) % PRELAUNCH_RECORDS;
  if (ground.count < PRELAUNCH_RECORDS)
    ++ground.count;
}

void logger_start(void) {
  if (logger_status == LOGGER_CLEAR_SCANNING ||
      logger_status == LOGGER_CLEAR_ERASING) {
    clear_cancelled = true;
    return;
  }
  if (logger_status >= LOGGER_OCCUPIED)
    return;

  ground_index = (ground.next_write + PRELAUNCH_RECORDS - ground.count) %
                 PRELAUNCH_RECORDS;
  ground_remaining = ground.count;
  started = true;
}

void logger_push_flight(const IMURecord *record) {
  if (!started || logger_status >= LOGGER_OCCUPIED)
    return;

  FlightBuf *buffer = &flight[fill_index];
  if (buffer->state != LOG_BUF_FREE && buffer->state != LOG_BUF_FILLING) {
    ++dropped_records;
    return;
  }
  buffer->state = LOG_BUF_FILLING;
  buffer->records[buffer->count++] = *record;
  if (buffer->count == FLIGHT_BUFFER_RECORDS) {
    buffer->state = LOG_BUF_READY;
    fill_index ^= 1U;
  }
}

void logger_finish(void) {
  if (!started || logger_status >= LOGGER_OCCUPIED)
    return;

  if (flight[fill_index].state == LOG_BUF_FILLING)
    flight[fill_index].state = LOG_BUF_READY;
  finishing = true;
}

void logger_poll(void) {
  if (logger_status >= LOGGER_OCCUPIED || storage->busy)
    return;

  if (storage->result != FLASH_RESULT_NONE) {
    bool ok = storage->result == FLASH_RESULT_OK;
    if (!flash_ack_result(storage) || !ok) {
      logger_status = LOGGER_ERROR;
      return;
    }
    if (clear_cancelled) {
      logger_status = LOGGER_ERROR;
      return;
    }
    switch (logger_status) {
    case LOGGER_PROBING:
      logger_status = LOGGER_CHECKING_SECTOR;
      break;
    case LOGGER_CHECKING_SECTOR:
      logger_status = sector_is_blank() ? LOGGER_READY : LOGGER_OCCUPIED;
      break;
    case LOGGER_CLEAR_SCANNING: {
      bool blank = sector_is_blank();
      if (!blank)
        sector += FLASH_SECTOR_SIZE;
      if (blank || sector == FLASH_CAPACITY_BYTES) {
        if (sector == LOG_START) {
          clear_complete();
        } else {
          /* Erase backwards so a reset leaves a discoverable prefix. */
          sector -= FLASH_SECTOR_SIZE;
          logger_status = LOGGER_CLEAR_ERASING;
        }
      }
      break;
    }
    case LOGGER_CLEAR_ERASING:
      if (sector == LOG_START)
        clear_complete();
      else
        sector -= FLASH_SECTOR_SIZE;
      break;
    case LOGGER_WRITING:
      write_complete();
      break;
    default:
      logger_status = LOGGER_ERROR;
      return;
    }
  }

  if (clear_cancelled) {
    logger_status = LOGGER_ERROR;
    return;
  }

  if (logger_status == LOGGER_CLEAR_ERASING) {
    if (!flash_erase_sector(storage, sector))
      logger_status = LOGGER_ERROR;
    return;
  }

  if (logger_status == LOGGER_CHECKING_SECTOR ||
      logger_status == LOGGER_CLEAR_SCANNING) {
    if (sector >= FLASH_CAPACITY_BYTES)
      logger_status = LOGGER_FULL;
    else if (!flash_read(storage, sector, block, sizeof(block)))
      logger_status = LOGGER_ERROR;
    return;
  }

  if (logger_status != LOGGER_READY || !started)
    return;

  FlightBuf *buffer = &flight[write_index];
  if (block_sequence == 0U) {
    write_block(BLOCK_HEADER, 0U, ground.count);
  } else if (ground_remaining != 0U) {
    uint32_t count =
        ground_remaining < BLOCK_RECORDS ? ground_remaining : BLOCK_RECORDS;
    write_block(BLOCK_GROUND, count, 0U);
  } else if (buffer->state == LOG_BUF_READY ||
             buffer->state == LOG_BUF_WRITING) {
    uint32_t remaining = buffer->count - flight_offset;
    uint32_t count = remaining < BLOCK_RECORDS ? remaining : BLOCK_RECORDS;
    write_block(BLOCK_FLIGHT, count, 0U);
  } else if (finishing) {
    write_block(BLOCK_FOOTER, records_written, dropped_records);
  }
}
