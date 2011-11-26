EXECUTABLE=virtualCOM.elf
BIN_IMAGE=virtualCOM.bin

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS=-g -O2 -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m4
CFLAGS+=-ffreestanding -nostdlib

#usb_conf.h
CFLAGS+=-DUSE_USB_OTG_FS=1

# to run from FLASH
CFLAGS+=-Wl,-T,stm32_flash.ld

CFLAGS+=-I./

# stm32f4_discovery lib
CFLAGS+=-I../libstm/STM32F4xx_StdPeriph_Driver/inc
CFLAGS+=-I../libstm/STM32F4xx_StdPeriph_Driver/inc/device_support
CFLAGS+=-I../libstm/STM32F4xx_StdPeriph_Driver/inc/core_support

#STM32_USB_Device_Library
CFLAGS+=-I../libstm/STM32_USB_Device_Library/Class/cdc/inc
CFLAGS+=-I../libstm/STM32_USB_Device_Library/Core/inc

#STM32_USB_OTG_Driver
CFLAGS+=-I../libstm/STM32_USB_OTG_Driver/inc

#STM32F4xx_StdPeriph_Driver\inc
CFLAGS+=-I../libstm/STM32F4xx_StdPeriph_Driver/inc

#Utilities
CFLAGS+=-I../libstm/Utilities/STM32F4-Discovery

all: $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

$(EXECUTABLE): main.c system_stm32f4xx.c startup_stm32f4xx.s stm32f4xx_it.c \
	usb_bsp.c usbd_desc.c usbd_usr.c usb_core.c  usbd_cdc_if_template.c \
	../libstm/Utilities/STM32F4-Discovery/stm32f4_discovery.c \
	../libstm/STM32_USB_OTG_Driver/src/usb_dcd_int.c \
	../libstm/STM32_USB_OTG_Driver/src/usb_dcd.c \
	../libstm/STM32_USB_Device_Library/Core/src/usbd_core.c \
	../libstm/STM32_USB_Device_Library/Core/src/usbd_req.c \
	../libstm/STM32_USB_Device_Library/Core/src/usbd_ioreq.c \
	../libstm/STM32_USB_Device_Library/Class/cdc/src/usbd_cdc_core.c \

		
	$(CC) $(CFLAGS) $^ -o $@  -L../libstm/STM32F4xx_StdPeriph_Driver/build -lSTM32F4xx_StdPeriph_Driver -L../libstm/STM32F_USB_OTG_Driver/build

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)

.PHONY: all clean
