BOARD_TYPE          := 0x06
BOARD_REVISION      := 0x01
BOOTLOADER_VERSION  := 0x00
HW_TYPE             := 0x01

MCU                 := cortex-m3
CHIP                := STM32F100C6T6B
BOARD               := STM32F100C6T6B_PX2IO_Rev1

# should be MD_VL here but makefile's not ready for it
MODEL               := MD
MODEL_SUFFIX        :=

# Note, osc freq will probably change with a board revision
OSCILLATOR_FREQ     := 8000000
SYSCLK_FREQ         := 24000000

# Note: These must match the values in link_$(BOARD)_memory.ld
# XXX currently not supporting a bootloader
FW_BANK_BASE        := 0x08000000   # Start of firmware flash
FW_BANK_SIZE        := (32 * 1024)  # Should include FW_DESC_SIZE

FW_DESC_SIZE        := 0x00000064
