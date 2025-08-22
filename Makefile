# Toolchain
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

# MCU specifics and flags
MCU_FLAGS = -mcpu=cortex-m3 -mthumb

CFLAGS = -Os -Wall -DSTM32F103xB $(MCU_FLAGS) \
 -I./Core/Inc \
 -I./Drivers/STM32F1xx_HAL_Driver/Inc \
 -I./Drivers/CMSIS/Device/ST/STM32F1xx/Include \
 -I./Drivers/CMSIS/Include \
 -ffunction-sections -fdata-sections

LDFLAGS = -TSTM32F103C8TX_FLASH.ld -Wl,--gc-sections -nostartfiles $(MCU_FLAGS)

# Source folders
CORE_SRC = Core/Src
HAL_SRC = Drivers/STM32F1xx_HAL_Driver/Src

# Source files list
#   $(CORE_SRC)/uart.c \
#   $(CORE_SRC)/can.c \
#   $(CORE_SRC)/commands.c \
#   $(CORE_SRC)/error_handler.c \
#   $(CORE_SRC)/gpio.c \

C_SRC = \
  $(CORE_SRC)/main.c \
  $(CORE_SRC)/stm32f1xx_it.c \
  $(CORE_SRC)/system_stm32f1xx.c \
  $(CORE_SRC)/syscalls.c \
  $(CORE_SRC)/stm32f1xx_hal_msp.c \
  $(HAL_SRC)/stm32f1xx_hal.c \
  $(HAL_SRC)/stm32f1xx_hal_rcc.c \
  $(HAL_SRC)/stm32f1xx_hal_gpio.c \
  $(HAL_SRC)/stm32f1xx_hal_uart.c \
  $(HAL_SRC)/stm32f1xx_hal_can.c \
  $(HAL_SRC)/stm32f1xx_hal_cortex.c \
  $(HAL_SRC)/stm32f1xx_hal_dma.c

ASM_SRCS = Core/Startup/startup_stm32f103c8tx.s

# Object files
OBJ = $(C_SRC:.c=.o) $(ASM_SRCS:.s=.o)

# Output binaries
TARGET = bluepillBlink.elf
BIN = bluepillBlink.bin
HEX = bluepillBlink.hex

.PHONY: all clean

all: $(TARGET) $(BIN) $(HEX)

# Link step: note the proper space between $@ and $(LDFLAGS)!
$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $@ $(LDFLAGS)
	$(SIZE) $@

# Compile .c to .o (CFLAGS only)
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	$(CC) $(CFLAGS) -c $< -o $@

# Binary and hex generation
$(BIN): $(TARGET)
	$(OBJCOPY) -O binary $< $@

$(HEX): $(TARGET)
	$(OBJCOPY) -O ihex $< $@

clean:
	rm -f $(OBJ) $(TARGET) $(BIN) $(HEX)
