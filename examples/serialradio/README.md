# Serial Radio Control Interface

A serial radio control interface for Contiki-NG nodes, providing spectrum analysis, packet sniffing, and radio parameter control over UART.

## Features

- **Radio Control**: Get/set channel, TX power, PAN ID, and other radio parameters
- **RSSI Scanning**: Single-channel and multi-channel spectrum scanning
- **Fast Scan**: Rapid all-channel RSSI sweeps for real-time spectrum visualization
- **Packet Sniffing**: Capture raw 802.15.4 frames with RSSI/LQI metadata
- **Frame Injection**: Send raw radio frames for testing
- **Jamming Mode**: Continuous transmission for interference testing
- **Web Interface**: Real-time spectrum visualization and control via browser
- **Python API**: High-level library for scripting and automation

## Protocol

Uses CBOR encoding over SLIP framing with CRC16 integrity checking:

```
SLIP frame: 0xC0 [CBOR message + CRC16] 0xC0
```

Debug output coexists with protocol messages on the same UART.

## Building

```bash
# For CC1352 LaunchPad (2.4 GHz)
make TARGET=simplelink BOARD=launchpad/cc1352r1

# For CC1352 SensorTag (Sub-GHz)
make TARGET=simplelink BOARD=sensortag/cc1352r1
```

## Python Tools

### Installation

```bash
cd tools
pip install pyserial cbor2 websockets
```

### CLI Usage

```bash
python -m tools.cli /dev/ttyACM0
```

CLI Commands:
- `ping` - Test connection
- `info` - Show radio info and parameters
- `channel [N]` - Get/set channel
- `power [N]` - Get/set TX power (dBm)
- `rssi` - Get current RSSI reading
- `scan [start] [end] [dwell_ms]` - Single RSSI scan
- `fastscan start [start_ch] [end_ch]` - Start continuous fast scanning
- `fastscan stop` - Stop fast scanning
- `sniff [channel]` - Start packet sniffing
- `sniff stop` - Stop sniffing
- `rx on|off` - Enable/disable radio receiver
- `tx <hex>` - Transmit raw frame
- `jam start [channel] [interval_ms]` - Start jamming
- `jam stop` - Stop jamming
- `webserver [port]` - Start web interface (default: 8080)

### Web Interface

Start the web server:
```bash
python -m tools.cli /dev/ttyACM0
> webserver
```

Then open http://localhost:8080 in your browser.

Features:
- **CLI Tab**: Console output, command input, radio info
- **RSSI Scan Tab**: 2D bar chart and 3D waterfall spectrum display
- **Packet Sniffer Tab**: Live packet capture with hex display

### Python API

```python
from tools import SerialRadio, RadioParam

radio = SerialRadio('/dev/ttyACM0')
radio.connect()

# Get/set parameters
channel = radio.get_channel()
radio.set_channel(26)
radio.set_tx_power(0)

# Packet sniffing
radio.set_rx_callback(lambda frame: print(f"RX: {frame.data.hex()}"))
radio.rx_on()

# Fast scanning
radio.set_fast_scan_callback(lambda scan: print(f"RSSI: {scan.rssi_values}"))
radio.start_fast_scan(11, 26)

radio.disconnect()
```

## Channel Ranges

| Band | Region | Channels |
|------|--------|----------|
| 2.4 GHz | Worldwide | 11-26 |
| 863 MHz | Europe | 0-33 |
| 915 MHz | US | 0-128 |
| 920 MHz | Japan | 0-37 |

## Files

```
serialradio/
├── serial-radio.c          # Main C implementation
├── serial-radio.h          # Protocol definitions
├── Makefile
├── project-conf.h
├── CLAUDE.md               # Development notes
├── PLAN.md                 # Implementation plan
└── tools/
    ├── __init__.py
    ├── cli.py              # Interactive CLI
    ├── serial_radio.py     # Python API
    ├── webserver.py        # Web interface server
    ├── protocol.py         # Protocol constants
    ├── slip.py             # SLIP encoder/decoder
    ├── crc16.py            # CRC16 implementation
    └── www/
        ├── index.html      # Web UI
        └── spectrum.js     # Visualization
```

## See Also

- [PLAN.md](PLAN.md) - Detailed implementation plan
- [CLAUDE.md](CLAUDE.md) - Development notes and troubleshooting
