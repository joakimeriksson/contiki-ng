# ESP32-C6-DevKitC Platform for Contiki-NG

This directory contains the Contiki-NG port for the ESP32-C6-DevKitC development board.

## Hardware

The ESP32-C6-DevKitC is a development board based on the ESP32-C6 SoC, which features:

- **CPU**: RISC-V 32-bit dual-core
  - High-Performance core: up to 160 MHz
  - Low-Power core: up to 20 MHz
- **Memory**: 400 KB SRAM, 320 KB ROM, 8 MB external Flash
- **Wireless**:
  - Wi-Fi 802.11ax (2.4 GHz)
  - Bluetooth 5.3 LE
  - **IEEE 802.15.4** (Zigbee/Thread support)
- **Peripherals**:
  - 2x UART
  - 1x I2C
  - 1x SPI
  - USB Serial/JTAG
  - 30 GPIO pins
  - WS2812 RGB LED on GPIO8

More information: [ESP32-C6-DevKitC User Guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c6/esp32-c6-devkitc-1/user_guide.html)

## Port Features

This Contiki-NG port implements:

- ✅ **UART driver** - Serial communication and debug output
- ✅ **Timer drivers** - Clock and rtimer for Contiki-NG scheduling
- ✅ **GPIO HAL** - General-purpose I/O control
- ✅ **LED driver** - Support for WS2812 RGB LED on GPIO8
- ✅ **IEEE 802.15.4 radio driver** - Wireless communication for 6LoWPAN/Thread
- ✅ **Platform initialization** - 3-stage boot process

## Toolchain Setup

You need the ESP-IDF RISC-V toolchain to build for this platform:

### Option 1: Install ESP-IDF (Recommended)

```bash
# Install ESP-IDF
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32c6

# Activate the environment
. ./export.sh
```

### Option 2: Standalone RISC-V Toolchain

```bash
# Download from Espressif's GitHub releases
# https://github.com/espressif/crosstool-NG/releases
# Look for riscv32-esp-elf packages

# Add to PATH
export PATH=$PATH:/path/to/riscv32-esp-elf/bin
```

## Building Examples

### Hello World Example

```bash
cd examples/hello-world
make TARGET=esp32c6-devkitc
```

### LED Blink Example

```bash
cd examples/dev/leds
make TARGET=esp32c6-devkitc
```

### 6LoWPAN Example

```bash
cd examples/rpl-udp
make TARGET=esp32c6-devkitc
```

## Flashing

The ESP32-C6 can be programmed via USB using esptool.py:

```bash
# Install esptool
pip install esptool

# Flash the firmware
esptool.py --chip esp32c6 \
  --port /dev/ttyUSB0 \
  --baud 460800 \
  write_flash 0x0 build/esp32c6-devkitc/app.bin
```

## Serial Console

Connect to the serial console to see debug output:

```bash
# Using screen
screen /dev/ttyUSB0 115200

# Using minicom
minicom -D /dev/ttyUSB0 -b 115200

# Using pyserial
python -m serial.tools.miniterm /dev/ttyUSB0 115200
```

## GPIO Pin Mapping

| Function | GPIO Pin | Notes |
|----------|----------|-------|
| RGB LED  | GPIO 8   | WS2812 addressable RGB LED |
| UART0 TX | GPIO 16  | Default UART for console |
| UART0 RX | GPIO 17  | Default UART for console |
| USB D-   | GPIO 12  | USB Serial/JTAG |
| USB D+   | GPIO 13  | USB Serial/JTAG |

## Network Configuration

The port supports IEEE 802.15.4 networking:

- Default channel: 26
- Default PAN ID: 0xABCD
- TX power range: -24 to +20 dBm

To change the channel:

```c
NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, 15);
```

## Power Management

The platform supports the Contiki-NG low-power framework:

```c
void platform_idle(void) {
  /* CPU enters light sleep, wakes on interrupt */
}
```

## Limitations

This is a lightweight port with the following limitations:

1. **Simplified hardware access** - Direct register access is provided as stubs. For production use, integrate with ESP-IDF HAL.
2. **No Wi-Fi/BLE** - Only IEEE 802.15.4 radio is implemented.
3. **Basic LED control** - WS2812 timing is simplified; for full RGB control, use RMT peripheral.
4. **No flash/storage** - Flash filesystem not yet implemented.

## Development

### Adding Features

To extend this port:

1. **Add new drivers** in `arch/cpu/esp32c6/dev/`
2. **Update Makefile** to include new source files
3. **Add configuration** in `esp32c6-conf.h`

### Debugging

Enable verbose logging:

```c
#define LOG_CONF_LEVEL_MAIN LOG_LEVEL_DBG
```

Or in Makefile:

```makefile
CFLAGS += -DLOG_CONF_LEVEL_MAIN=LOG_LEVEL_DBG
```

## References

- [ESP32-C6 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c6_technical_reference_manual_en.pdf)
- [ESP32-C6-DevKitC Schematic](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c6/esp32-c6-devkitc-1/user_guide.html)
- [Contiki-NG Documentation](https://docs.contiki-ng.org/)
- [Zephyr ESP32-C6 Port](https://docs.zephyrproject.org/latest/boards/espressif/esp32c6_devkitc/doc/index.html)

## License

This port is released under the same BSD license as Contiki-NG.

## Authors

- ESP32-C6 port for Contiki-NG (2025)

## Support

For issues and questions:
- Contiki-NG GitHub: https://github.com/contiki-ng/contiki-ng
- ESP32-C6 Forum: https://www.esp32.com/
