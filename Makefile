.DEFAULT_GOAL := all

CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

OUT      := build/firmware
LDSCRIPT := STM32F413xx_FLASH.ld

CPUFLAGS := -mcpu=cortex-m4 -mthumb \
            -mfpu=fpv4-sp-d16 -mfloat-abi=hard

CPPFLAGS := -DSTM32F413xx -Iinc -Icmsis -Isrc
CFLAGS   := $(CPUFLAGS) -std=c11 -O0 -g \
            -ffunction-sections -fdata-sections \
            -Wall -Wextra -MMD -MP

LDFLAGS := $(CPUFLAGS) -T$(LDSCRIPT) \
           -nostartfiles -nostdlib \
           -Wl,--gc-sections \
           -Wl,-Map=$(OUT).map

LDLIBS := -Wl,--start-group -lm -lc -lgcc -Wl,--end-group

# Add any separate file containing your USART IRQ handlers here.
C_SOURCES := \
    src/main.c \
    src/system_stm32f4xx.c \
    src/syscalls.c \
    src/hardware.c \
    src/imu.c \
    src/ring_buf.c \
    src/usart.c \
    src/usart_board.c \
    src/vn_kalman.c \
    src/flight_state_logic.c \
    src/timer.c \
	src/usart_irq.c

C_OBJECTS := $(patsubst src/%.c,build/%.o,$(C_SOURCES))
OBJECTS   := $(C_OBJECTS) build/startup.o
DEPS      := $(C_OBJECTS:.o=.d)

.PHONY: all clean size

all: $(OUT).elf $(OUT).bin $(OUT).hex
	$(SIZE) $(OUT).elf

build:
	mkdir -p build

build/%.o: src/%.c Makefile | build
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

build/startup.o: src/startup_stm32f413xx.s Makefile | build
	$(CC) $(CPUFLAGS) -g -c $< -o $@

$(OUT).elf: $(OBJECTS) $(LDSCRIPT) Makefile
	$(CC) $(LDFLAGS) $(OBJECTS) $(LDLIBS) -o $@

$(OUT).bin: $(OUT).elf
	$(OBJCOPY) -O binary $< $@

$(OUT).hex: $(OUT).elf
	$(OBJCOPY) -O ihex $< $@

size: $(OUT).elf
	$(SIZE) $<

clean:
	rm -rf build

-include $(DEPS)