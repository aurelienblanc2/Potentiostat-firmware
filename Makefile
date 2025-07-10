#Define the Version of the build
VERSION := v0.0.1

# Define the C compiler
CC := gcc-arm-none-eabi

# Define the assembler
AS := gcc-arm-none-eabi

# Directory for object files
OBJ_DIR := build

# Executable names
RELEASE := potentiostat_rev_C_$(VERSION).elf
DEBUG := potentiostat_rev_C_$(VERSION)_debug.elf


# Define the C compiler flags
CFLAGS_RELEASE  := \
-mcpu=cortex-m4 \
-std=gnu11 \
-DSTM32G473xx \
-DUSE_HAL_DRIVER \
-DSTM32_THREAD_SAFE_STRATEGY=2 \
-O2 \
-ffunction-sections \
-fdata-sections \
-Wall \
-fstack-usage \
--specs=nano.specs \
-mfpu=fpv4-sp-d16 \
-mfloat-abi=hard \
-mthumb

CFLAGS_DEBUG := \
-mcpu=cortex-m4 \
-std=gnu11 \
-DSTM32G473xx \
-DUSE_HAL_DRIVER \
-DSTM32_THREAD_SAFE_STRATEGY=2 \
-Og \
-ffunction-sections \
-fdata-sections \
-Wall \
-fstack-usage \
--specs=nano.specs \
-mfpu=fpv4-sp-d16 \
-mfloat-abi=hard \
-mthumb \
-DDEBUG \
-g

LDFLAGS := -Wl,--gc-sections -static

AFLAGS := -mcpu=cortex-m4 --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb



# Define the source files
SRC_C := \
src/USB_Device/App/usb_device.c \
src/USB_Device/App/usbd_cdc_if.c \
src/USB_Device/App/usbd_desc.c \
src/USB_Device/Target/usbd_conf.c \
src/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
src/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
src/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
src/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
src/Middlewares/Third_Party/FatFs/src/diskio.c \
src/Middlewares/Third_Party/FatFs/src/ff.c \
src/Middlewares/Third_Party/FatFs/src/ff_gen_drv.c \
src/Middlewares/Third_Party/FatFs/src/option/ccsbcs.c \
src/Middlewares/Third_Party/FatFs/src/option/syscall.c \
src/Middlewares/Third_Party/Flash_Driver/helper_functions.c \
src/Middlewares/Third_Party/Flash_Driver/spi_driver.c \
src/Middlewares/Third_Party/Flash_Driver/standardflash.c \
src/FATFS/Target/user_diskio.c \
src/FATFS/App/app_fatfs.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cordic.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_crc.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_crc_ex.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dac.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dac_ex.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fmac.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_iwdg.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_lptim.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_opamp.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_opamp_ex.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd_ex.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rng.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rtc.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rtc_ex.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c \
src/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usb.c \
src/Core/Src/FileManager.c \
src/Core/Src/LedFlash.c \
src/Core/Src/ModbusRTU.c \
src/Core/Src/NVM.c \
src/Core/Src/QueryList.c \
src/Core/Src/analog.c \
src/Core/Src/fifo.c \
src/Core/Src/main.c \
src/Core/Src/shtc3.c \
src/Core/Src/stm32g4xx_hal_msp.c \
src/Core/Src/stm32g4xx_hal_timebase_tim.c \
src/Core/Src/stm32g4xx_it.c \
src/Core/Src/syscalls.c \
src/Core/Src/sysmem.c \
src/Core/Src/system_stm32g4xx.c \
src/Core/Src/task_timer.c \
src/Core/Src/user.c \
src/Core/ThreadSafe/newlib_lock_glue.c 

SRC_S := \
src/Core/Startup/startup_stm32g473cctx.s 

INCL := \
-I src/USB_Device/App \
-I src/USB_Device/Target \
-I src/Middlewares/Third_Party/FatFs/src \
-I src/Middlewares/Third_Party/Flash_Driver \
-I src/Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
-I src/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
-I src/FATFS/Target \
-I src/FATFS/App \
-I src/Drivers/STM32G4xx_HAL_Driver/Inc \
-I src/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy \
-I src/Drivers/CMSIS/Device/ST/STM32G4xx/Include \
-I src/Drivers/CMSIS/Include \
-I src/Core/Inc \
-I src/Core/ThreadSafe

LIB := \
-lm \
-lc

LINKER := \
-T src/STM32G473CCTX_FLASH.ld


# Define the object files from sources
OBJS_C := $(patsubst %.c, $(OBJ_DIR)/%.o, $(SRC_C))
OBJS_S := $(patsubst %.s, $(OBJ_DIR)/%.o, $(SRC_S))
OBJS := $(OBJS_C) $(OBJS_S)



# Default to release build, can be overridden by "make BUILD=debug"
BUILD ?= release

ifeq ($(BUILD),debug)
    CFLAGS := $(CFLAGS_DEBUG)
    TARGET := $(DEBUG)
else
    CFLAGS := $(CFLAGS_RELEASE)
    TARGET := $(RELEASE)
endif


# Default target
all: clean release

# Release build target
release: BUILD := release
release: $(RELEASE)

# Debug build target
debug: BUILD := debug
debug: $(DEBUG)


# Link the final executable
$(RELEASE): $(OBJS)
	@echo "Linking Release build: $@"
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) $(LINKER) -o $@ $(LIB)

$(DEBUG): $(OBJS)
	@echo "Linking Debug build: $@"
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) $(LINKER) -o $@ $(LIB)



# Create object directories as needed, compile C
$(OBJ_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCL) -c $< -o $@

# Create object directories as needed, compile assembly
$(OBJ_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) $(AFLAGS) $(INCL) -c -x assembler-with-cpp $< -o $@



# Clean rule
.PHONY: clean all release debug

clean:
	@rm -rf $(OBJ_DIR)
	@rm -f $(RELEASE) $(DEBUG)
	@rm -f ./*.elf
	@echo "Clean done."