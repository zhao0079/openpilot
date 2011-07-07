BOARD_TYPE          := 0x05
BOARD_REVISION      := 0x01
BOOTLOADER_VERSION  := 0x00
HW_TYPE             := 0x00

MCU                 := cortex-m3
CHIP                := STM32F205RGT
BOARD               := STM32205_PX2FMU_Rev1
MODEL               := HD
MODEL_SUFFIX        := _OP

# Note, osc freq will probably change with a board revision
OSCILLATOR_FREQ     := 8000000
SYSCLK_FREQ         := 120000000

# Note: These must match the values in link_$(BOARD)_memory.ld
# Note: currently building PX2FMU with no bootloader support
#
BL_BANK_BASE        := 0x08000000  # Start of bootloader flash
BL_BANK_SIZE        := 0x00000000  # Should include BD_INFO region
FW_BANK_BASE        := 0x08000000  # Start of firmware flash
FW_BANK_SIZE        := (1024 * 1024)  # Should include FW_DESC_SIZE

FW_DESC_SIZE        := 0x00000064
