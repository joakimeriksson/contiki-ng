# Serial Radio Project - Development Notes

## FORBIDDEN ACTIONS

**DO NOT DISABLE DEBUG OUTPUT TO FIX SLIP ISSUES!**

Setting `LOG_LEVEL` to `LOG_LEVEL_NONE` is **NOT** the solution to SLIP/debug coexistence problems. The correct fix is always on the Python side in `tools/slip.py`.

## SLIP and Debug Output Coexistence

### The Problem

When using SLIP framing for binary protocol (CBOR) communication over serial, debug output (LOG_INFO, LOG_DBG, printf) can get mixed with SLIP frames.

The byte flow looks like this:
```
Device sends: 0xC0 [cbor+crc] 0xC0 [debug text without 0xC0] 0xC0 [cbor+crc] 0xC0
```

The SLIP protocol buffers everything between 0xC0 (SLIP_END) markers. If debug text appears between two SLIP frames, the next 0xC0 causes that debug text to be delivered as a "frame".

### The CORRECT Solution (Python-side)

The fix is in `tools/slip.py` - validate that received frames look like valid CBOR before processing them:

1. Valid CBOR messages from this protocol start with a CBOR map (byte 0xA0-0xBF)
2. Debug text starts with printable ASCII (bytes 0x20-0x7E, like `[` for `[INFO:...]`)
3. If a "frame" starts with printable ASCII, treat it as debug text, not a CBOR frame

The `_is_valid_cbor_frame()` method in `SlipDecoder` implements this check.

### Why NOT to disable debug

1. Debug output is essential for development and troubleshooting
2. The slip-radio example has worked with debug for years
3. The Python decoder can easily distinguish CBOR from text
4. Disabling debug masks the real issue instead of fixing it

## Protocol Overview

### Framing Stack
```
Application: CBOR message
     |
     v
CRC16: CBOR + 2-byte CRC (little-endian)
     |
     v
SLIP: 0xC0 [escaped data] 0xC0
     |
     v
UART: raw bytes at 115200 baud
```

### CBOR Message Format

All messages use CBOR maps with single-character keys for compactness:

| Key | Meaning |
|-----|---------|
| `t` | Type/opcode |
| `i` | Message ID |
| `p` | Radio parameter |
| `v` | Value |
| `f` | Frame data |
| `r` | RSSI |
| `l` | LQI |
| `c` | Channel |
| `s` | Start channel / Sequence |
| `e` | End channel |
| `d` | Dwell time |
| `x` | Error code |
| `V` | Version string |
| `R` | RSSI array (fast scan) |
| `n` | Sequence number |

### CRC16 Algorithm

Uses Contiki-NG's `crc16_data()` from `lib/crc16.h`. The Python implementation in `tools/crc16.py` must match exactly:

```python
def crc16_add(byte: int, crc: int) -> int:
    crc = crc & 0xFFFF
    byte = byte & 0xFF
    crc ^= byte
    crc = ((crc >> 8) | (crc << 8)) & 0xFFFF
    crc ^= ((crc & 0xFF00) << 4) & 0xFFFF
    crc ^= (crc >> 8) >> 4
    crc ^= ((crc & 0xFF00) >> 5) & 0xFFFF
    return crc & 0xFFFF
```

## Radio Frequency Bands

The radio auto-detects its frequency band from channel range:

| Band | Region | Channel Range |
|------|--------|---------------|
| 2.4 GHz | Worldwide | 11-26 |
| Sub-GHz 863 MHz | Europe | 0-33 |
| Sub-GHz 915 MHz | US | 0-128 |
| Sub-GHz 920 MHz | Japan | 0-37 |

The `info` command in CLI displays the detected band.

## File Structure

```
serialradio/
├── serial-radio.c      # Main C implementation
├── serial-radio.h      # Protocol definitions (opcodes, keys)
├── serial-radio-main.c # Contiki-NG main entry point
├── Makefile
├── project-conf.h
├── CLAUDE.md           # This file
└── tools/
    ├── __init__.py
    ├── cli.py          # Interactive command-line interface
    ├── serial_radio.py # Python SerialRadio class
    ├── protocol.py     # Protocol constants (must match serial-radio.h)
    ├── slip.py         # SLIP encoder/decoder (THIS IS WHERE DEBUG/SLIP FIX GOES)
    └── crc16.py        # CRC16 (must match lib/crc16.h)
```

## Common Issues

### CRC verification failed with debug text
**WRONG:** Disable LOG_LEVEL
**RIGHT:** Fix `tools/slip.py` to filter out debug text using `_is_valid_cbor_frame()`

### CRC verification failed with binary data
1. Check that `tools/crc16.py` matches Contiki-NG's algorithm exactly
2. Enable hex dump in `serial_radio.py` `_handle_frame()` to diagnose

### No response to PING
1. Check serial port and baud rate (115200)
2. Check that firmware is running (should send heartbeat every 10s)
3. Check SLIP framing is correct on both sides

### Fast scan not working
1. Check channel range is valid for radio (use `info` command)
2. Maximum 32 channels per sweep due to buffer limits

## Border Router Integration

### How the Native Border Router Works

The native border router (`examples/rpl-border-router` compiled for `TARGET=native`) communicates with a slip-radio device over serial using a simple command protocol:

**Protocol Format:**
- Commands use prefix: `!` (directive) or `?` (query)
- Raw IP packets have NO prefix - sent directly as SLIP frames

**Key Commands:**
| Command | Direction | Purpose |
|---------|-----------|---------|
| `!S` | Host→Radio | Send packet: `[!S][SID][attrs][payload]` |
| `!R` | Radio→Host | TX status: `[!R][SID][status][retries]` |
| `!V` | Host→Radio | Set param: `[!V][type_hi][type_lo][val_hi][val_lo]` |
| `?V` | Host→Radio | Get param: `[?V][type_hi][type_lo]` |
| `?M` | Host→Radio | Query MAC address |
| `!M` | Radio→Host | MAC response: `[!M][8 bytes MAC]` |

**Packet Flow:**
```
TX: Host sends !S command → slip-radio transmits → sends !R status back
RX: Radio receives frame → slipnet_input() → slip_write() raw packet to host
```

### Key Source Files

**On the radio side (slip-radio):**
- `examples/slip-radio/slip-radio.c` - Command handler (`slip_radio_cmd_handler`)
- `examples/slip-radio/slip-net.c` - RX forwarding (`slipnet_input` calls `slip_write`)

**On the host side (native border router):**
- `os/services/rpl-border-router/native/slip-dev.c` - SLIP I/O, serial port handling
- `os/services/rpl-border-router/native/border-router-cmds.c` - Command processing
- `os/services/rpl-border-router/native/border-router-mac.c` - Builds `!S` commands

**Shared:**
- `os/services/slip-cmd/cmd.c` - Command dispatcher
- `os/services/slip-cmd/packetutils.c` - Attribute serialization

### Options for Using serialradio as Border Router

**Option 1: Make serialradio speak slip-radio protocol**
- Add `!S`, `!R`, `?M`, `!M` command handlers alongside CBOR commands
- Add RX forwarding via `slipnet_input()` pattern
- Can then use existing native border router unchanged

**Option 2: Create Python-based border router**
- Keep our CBOR protocol
- Write Python border router that:
  - Receives frames via `RX_FRAME` events
  - Sends frames via `TX_RAW_FRAME` command
  - Handles IPv6 routing (using scapy or similar)
  - Creates TUN/TAP interface for host integration

**Option 3: Modify native border router**
- Update `slip-dev.c` to speak CBOR/CRC16 instead of raw SLIP
- More invasive but cleaner long-term

### Adding RX Frame Forwarding to serialradio

To forward received radio frames to the host, add to `serial-radio.c`:

```c
/* In the radio input callback or MAC layer */
static void
forward_rx_frame(const uint8_t *data, uint16_t len, int8_t rssi, uint8_t lqi)
{
  cbor_writer_state_t writer;
  cbor_init_writer(&writer, tx_buf, TX_BUF_SIZE);

  cbor_open_map(&writer);
  cbor_write_text(&writer, "t", 1);
  cbor_write_unsigned(&writer, SRADIO_EVT_RX_FRAME);
  cbor_write_text(&writer, "f", 1);
  cbor_write_data(&writer, data, len);
  cbor_write_text(&writer, "r", 1);
  cbor_write_signed(&writer, rssi);
  cbor_write_text(&writer, "l", 1);
  cbor_write_unsigned(&writer, lqi);
  cbor_close_map(&writer);

  size_t msg_len = cbor_end_writer(&writer);
  if(msg_len > 0) {
    send_slip_frame(tx_buf, msg_len);
  }
}
```

Then hook this into the MAC/radio layer input path, similar to how `slip-net.c` does it with `NETSTACK_MAC.input`.
