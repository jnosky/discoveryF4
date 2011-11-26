EXECUTABLE=HypertermIrq.elf
BIN_IMAGE=HypertermIrq.bin

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS=-g -O2 -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m4	
CFLAGS+=-ffreestanding 

#Don't use newlib
CFLAGS+=-nostdlib
#Link to newlib
#CFLAGS+=-L/cygdrive/c/sat/arm-none-eabi/lib/thumb2


# to run from FLASH
CFLAGS+=-Wl,-T,stm32_flash.ld

#Search current directory too
CFLAGS+=-I./

# stm32f4_discovery lib
CFLAGS+=-I../libstm/STM32F4xx_StdPeriph_Driver/inc
CFLAGS+=-I../libstm/STM32F4xx_StdPeriph_Driver/inc/device_support
CFLAGS+=-I../libstm/STM32F4xx_StdPeriph_Driver/inc/core_support

#Utilities
CFLAGS+=-I../libstm/Utilities/STM32F4-Discovery

all: $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

$(EXECUTABLE): main.c system_stm32f4xx.c startup_stm32f4xx.s stm32f4xx_it.c \
	../libstm/Utilities/STM32F4-Discovery/stm32f4_discovery.c \



	$(CC) $(CFLAGS) $^ -o $@  -L../libstm/STM32F4xx_StdPeriph_Driver/build -lSTM32F4xx_StdPeriph_Driver \

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)

.PHONY: all clean
