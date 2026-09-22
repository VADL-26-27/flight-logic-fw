#!/bin/bash
set -e

echo "============================== Building STM32F411 firmware ================================"

# Toolchain

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
SIZE=arm-none-eabi-size

# Flags

CFLAGS="
-mcpu=cortex-m4
-mthumb
-O0 
-g 
-ffunction-sections 
-fdata-sections 
-Wall
-Wextra
-mfpu=fpv4-sp-d16
-mfloat-abi=hard
"

DEFINES="
-DSTM32F411xE
"

INCLUDES="
-Iinc 
-Icmsis 
-Isrc
"

# Linker

LDSCRIPT=STM32F411xx_FLASH.ld

# Output

OUT=build/firmware

mkdir -p build

echo "======================================= Compiling ========================================="

$CC $CFLAGS $DEFINES $INCLUDES -c src/main.c -o build/main.o
$CC $CFLAGS $DEFINES $INCLUDES -c src/system_stm32f4xx.c -o build/system.o

$CC $CFLAGS \
-c src/startup_stm32f411xe.s \
-o build/startup.o

echo "======================================== Linking =========================================="

$CC $CFLAGS \
build/main.o \
build/system.o \
build/startup.o \
-T $LDSCRIPT \
-nostartfiles \
-nostdlib \
-Wl,--gc-sections \
-Wl,-Map=build/firmware.map \
-o $OUT.elf

echo "========================================= Size ============================================"

$SIZE $OUT.elf

echo "================================= Generating binary/hex ==================================="

$OBJCOPY -O binary $OUT.elf $OUT.bin
$OBJCOPY -O ihex   $OUT.elf $OUT.hex

echo "======================================= Flashing =========================================="

# dfu-util -a 0 -s 0x08000000:force:leave -D $OUT.bin
st-flash --reset write "$OUT.bin" 0x08000000

echo "========================================= Done ============================================"