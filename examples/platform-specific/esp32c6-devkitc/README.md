# ESP32-C6-DevKitC Examples

This directory contains examples specific to the ESP32-C6-DevKitC platform.

## Hello ESP32-C6

A simple example demonstrating:
- Platform initialization
- LED control
- Timer usage
- Radio configuration
- Serial output

### Building

```bash
make TARGET=esp32c6-devkitc
```

### Running

1. Flash the firmware to your ESP32-C6-DevKitC board
2. Connect to the serial console (115200 baud)
3. You should see:
   - Platform information printed
   - Radio configuration details
   - LED blinking every 2 seconds
   - Heartbeat messages

### Expected Output

```
=================================
  ESP32-C6-DevKitC Demo
=================================

Platform: ESP32-C6-DevKitC
CPU: RISC-V @ 160 MHz
RAM: 400 KB

Radio channel: 26
Radio TX power: 5 dBm
Radio PAN ID: 0xABCD

Radio is ON

Starting LED blink demo...
LEDs will toggle every 2 seconds

Heartbeat 1 - LED toggled
Heartbeat 2 - LED toggled
...
```

## More Examples

You can also run general Contiki-NG examples with this platform:

```bash
# LED example
cd ../../dev/leds
make TARGET=esp32c6-devkitc

# RPL/6LoWPAN example
cd ../../rpl-udp
make TARGET=esp32c6-devkitc

# CoAP example
cd ../../coap
make TARGET=esp32c6-devkitc
```

## Troubleshooting

### Build fails with "riscv32-esp-elf-gcc: command not found"

Make sure you have installed the ESP-IDF toolchain and sourced the environment:

```bash
cd /path/to/esp-idf
. ./export.sh
```

### Cannot flash the board

Check that:
1. The board is connected via USB
2. You have permissions to access /dev/ttyUSB0 (or appropriate port)
3. The correct port is specified

```bash
# Add yourself to dialout group (Linux)
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

### No output on serial console

1. Verify baud rate is 115200
2. Check that you're connected to the correct port
3. Try resetting the board (press RESET button)
