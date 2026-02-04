#!/bin/bash
# Kalico Firmware Build Script for Voron Trident
# Builds firmware for BTT Octopus V1.1 and Fly SHT36 V3 Max using Docker

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KALICO_DIR="/home/sandra/source/kalico"
OUTPUT_DIR="$SCRIPT_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Kalico Firmware Builder ===${NC}"
echo "Output directory: $OUTPUT_DIR"
echo ""

# Check Docker is available
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Error: Docker is not installed or not in PATH${NC}"
    exit 1
fi

build_octopus() {
    echo -e "${YELLOW}Building BTT Octopus V1.1 (USB-to-CAN Bridge)...${NC}"

    docker run --rm -v "$KALICO_DIR":/klipper -w /klipper ubuntu:24.04 bash -c "
apt-get update > /dev/null 2>&1 &&
apt-get install -y gcc-arm-none-eabi libnewlib-arm-none-eabi make python3 > /dev/null 2>&1 &&
make clean > /dev/null 2>&1 &&
rm -f .config &&

cat > .config << 'EOFCONFIG'
CONFIG_LOW_LEVEL_OPTIONS=y
CONFIG_MACH_STM32=y
CONFIG_MACH_STM32F446=y
CONFIG_MACH_STM32F4=y
CONFIG_HAVE_STM32_USBOTG=y
CONFIG_HAVE_STM32_CANBUS=y
CONFIG_HAVE_STM32_USBCANBUS=y
CONFIG_STM32_DFU_ROM_ADDRESS=0x1fff0000
CONFIG_STM32_FLASH_START_2000=y
CONFIG_FLASH_SIZE=0x80000
CONFIG_FLASH_BOOT_ADDRESS=0x8000000
CONFIG_RAM_START=0x20000000
CONFIG_RAM_SIZE=0x20000
CONFIG_STACK_SIZE=512
CONFIG_STM32F0_TRIM=16
CONFIG_CLOCK_FREQ=180000000
CONFIG_FLASH_APPLICATION_ADDRESS=0x8008000
CONFIG_STM32_CLOCK_REF_12M=y
CONFIG_USBCANBUS=y
CONFIG_USB_VENDOR_ID=0x1d50
CONFIG_USB_DEVICE_ID=0x614e
CONFIG_USB_SERIAL_NUMBER_CHIPID=y
CONFIG_STM32F4_USB_PA11_PA12=y
CONFIG_CAN_PINS_PD0_PD1=y
CONFIG_CANBUS_FREQUENCY=1000000
CONFIG_INITIAL_PINS=\"\"
CONFIG_HAVE_GPIO=y
CONFIG_HAVE_GPIO_ADC=y
CONFIG_HAVE_GPIO_SPI=y
CONFIG_HAVE_GPIO_SDIO=y
CONFIG_HAVE_GPIO_I2C=y
CONFIG_HAVE_GPIO_HARD_PWM=y
CONFIG_HAVE_STRICT_TIMING=y
CONFIG_HAVE_CHIPID=y
CONFIG_HAVE_STEPPER_BOTH_EDGE=y
CONFIG_HAVE_BOOTLOADER_REQUEST=y
CONFIG_INLINE_STEPPER_HACK=y
EOFCONFIG

make olddefconfig &&
make -j\$(nproc)
"

    cp "$KALICO_DIR/out/klipper.bin" "$OUTPUT_DIR/octopus-v1.1-can-bridge.bin"
    echo -e "${GREEN}Built: octopus-v1.1-can-bridge.bin${NC}"
}

build_sht36() {
    echo -e "${YELLOW}Building Fly SHT36 V3 Max (CAN)...${NC}"

    docker run --rm -v "$KALICO_DIR":/klipper -w /klipper ubuntu:24.04 bash -c "
apt-get update > /dev/null 2>&1 &&
apt-get install -y gcc-arm-none-eabi libnewlib-arm-none-eabi make python3 > /dev/null 2>&1 &&
make clean > /dev/null 2>&1 &&
rm -f .config &&

cat > .config << 'EOFCONFIG'
CONFIG_LOW_LEVEL_OPTIONS=y
CONFIG_MACH_RPXXXX=y
CONFIG_MACH_RP2040=y
CONFIG_BOARD_DIRECTORY=\"rp2040\"
CONFIG_MCU=\"rp2040\"
CONFIG_CLOCK_FREQ=12000000
CONFIG_FLASH_SIZE=0x200000
CONFIG_FLASH_BOOT_ADDRESS=0x10000100
CONFIG_RAM_START=0x20000000
CONFIG_RAM_SIZE=0x42000
CONFIG_STACK_SIZE=512
CONFIG_RPXXXX_FLASH_START_4000=y
CONFIG_FLASH_APPLICATION_ADDRESS=0x10004000
CONFIG_RP2040_HAVE_STAGE2=y
CONFIG_RPXXXX_HAVE_BOOTLOADER=y
CONFIG_RP2040_FLASH_W25Q080=y
CONFIG_RP2040_STAGE2_FILE=\"boot2_w25q080.S\"
CONFIG_RP2040_STAGE2_CLKDIV=2
CONFIG_RPXXXX_CANBUS=y
CONFIG_CANSERIAL=y
CONFIG_CANBUS=y
CONFIG_CANBUS_FREQUENCY=1000000
CONFIG_RPXXXX_CANBUS_GPIO_RX=1
CONFIG_RPXXXX_CANBUS_GPIO_TX=0
CONFIG_INITIAL_PINS=\"!gpio5\"
CONFIG_HAVE_GPIO=y
CONFIG_HAVE_GPIO_ADC=y
CONFIG_HAVE_GPIO_SPI=y
CONFIG_HAVE_GPIO_I2C=y
CONFIG_HAVE_GPIO_HARD_PWM=y
CONFIG_HAVE_STRICT_TIMING=y
CONFIG_HAVE_CHIPID=y
CONFIG_HAVE_STEPPER_OPTIMIZED_BOTH_EDGE=y
CONFIG_HAVE_BOOTLOADER_REQUEST=y
CONFIG_INLINE_STEPPER_HACK=y
EOFCONFIG

make olddefconfig &&
make -j\$(nproc)
"

    cp "$KALICO_DIR/out/klipper.bin" "$OUTPUT_DIR/sht36-v3-max-can.bin"
    echo -e "${GREEN}Built: sht36-v3-max-can.bin${NC}"
}

# Parse arguments
case "${1:-all}" in
    octopus)
        build_octopus
        ;;
    sht36)
        build_sht36
        ;;
    all)
        build_octopus
        echo ""
        build_sht36
        ;;
    *)
        echo "Usage: $0 [octopus|sht36|all]"
        echo "  octopus - Build BTT Octopus V1.1 USB-to-CAN bridge firmware"
        echo "  sht36   - Build Fly SHT36 V3 Max CAN firmware"
        echo "  all     - Build both (default)"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}=== Build Complete ===${NC}"
ls -la "$OUTPUT_DIR"/*.bin
