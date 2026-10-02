#include "ring_buf.h"

void init_buf(rx_buf* buffer) {
    atomic_init(&buffer->read, 0);
    atomic_init(&buffer->write, 0);
    atomic_init(&buffer->count, 0);
}

void enqueue(rx_buf* buffer, const uint8_t byte) {
    if (buffer == NULL) {
        return;
    }

    if (atomic_load_explicit(&buffer->count, memory_order_acquire) >= BUFFER_SIZE) {
        return;
    }

    uint8_t write = atomic_load_explicit(&buffer->write, memory_order_acquire);

    buffer->buf[write] = byte;

    // update only takes one clock cycle
    write = (write + 1U) & (BUFFER_SIZE - 1);
    atomic_store_explicit(&buffer->write, write, memory_order_release);
    atomic_fetch_add_explicit(&buffer->count, 1, memory_order_release);

}

// This function is only called when the count is >= 36
void read_from_buf(rx_buf* buffer, uint8_t* payload, uint8_t len) {
    if (buffer == NULL || payload == NULL) {
        return;
    }
    if (len > atomic_load_explicit(&buffer->count, memory_order_acquire)) {
        return;
    }

    uint8_t read = atomic_load_explicit(&buffer->read, memory_order_acquire);

    for (uint8_t i = 0; i < len; ++i) {
        *payload = buffer->buf[read];
        ++payload;
        read = (read + 1U) & (BUFFER_SIZE - 1);
        atomic_store_explicit(&buffer->read, read, memory_order_release);
        atomic_fetch_sub_explicit(&buffer->count, 1, memory_order_release);
    }
}