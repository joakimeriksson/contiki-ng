# PLAN.md
## SLIP/Serial Radio Protocol & Tools – Implementation Plan

This document defines the architecture, design goals, and concrete implementation steps for a new **Serial Radio Control Interface** for Contiki-NG nodes using:

- **SLIP** framing  
- **CBOR** as payload encoding  
- **CRC16** integrity checking  
- **Radio parameter control** via Contiki-NG’s `RADIO_PARAM_*` API  
- **Raw radio frame send/receive support**  
- **RSSI scanning functionality**  
- **Full coexistence with debug-print output**  
- A corresponding **Python controller** for PC-side interaction

---

# 1. Goals

1. Provide a robust, extensible serial control protocol for radio experiments:
   - Driving PHY/MAC experiments
   - Channel scans
   - Raw packet injection and sniffing
   - Radio parameter tuning
2. Use existing Contiki-NG SLIP architecture for framing.
3. Use CBOR to encode structured commands and events compactly.
4. Add CRC16 to ensure UART reliability.
5. Maintain full compatibility with debug text output on the UART.
6. Offer a Python library + CLI tool to control the node.

---

# 2. System Architecture

```
PC  <—UART—>  Node (Contiki-NG)
```

### PC side
- SLIP decoder  
- CRC checking  
- CBOR decoding  
- High-level API for:
  - Setting radio params  
  - Starting/stopping scans  
  - Sending/receiving raw radio frames  
- CLI for interactive use  

### Node side
- SLIP receiver and sender  
- CBOR encoder/decoder (TinyCBOR)  
- CRC checking  
- Radio control logic  
- Scan process  
- Raw frame handling  
- Debug prints over same UART, outside SLIP frames  

---

# 3. Message Format

### Outer framing
- SLIP frame: `C0 … C0`
- SLIP escaping for `C0` and `DB`

### Payload
```
[ CBOR encoded map ][ CRC16-CCITT little endian ]
```

### CBOR structure
CBOR map with keys like:

| Key | Meaning |
|-----|---------|
| `"t"` | message type (opcode) |
| `"id"` | message ID (request/response matching) |
| `"p"` | radio parameter code (RADIO_PARAM_*) |
| `"v"` | parameter value (int, array, bytes) |
| `"f"` | raw radio frame (byte string) |
| `"rssi"` | RSSI value |
| `"lqi"` | link quality |
| `"ch"` | channel |
| `"start"`, `"end"`, `"dwell"` | scan configuration |
| `"err"` | error code |

This provides a fully self-describing, extensible protocol.

---

# 4. Command Set

### 4.1 Control commands (PC → Node)

| Opcode | Meaning |
|--------|---------|
| `0` | Ping |
| `1` | GET_PARAM |
| `2` | SET_PARAM |
| `3` | RSSI_SCAN_START |
| `4` | RSSI_SCAN_STOP |
| `50` | TX_RAW_FRAME |

### 4.2 Events (Node → PC)

| Opcode | Meaning |
|--------|---------|
| `51` | GET_PARAM_RESPONSE |
| `52` | RX_FRAME_EVENT |
| `53` | RSSI_SCAN_RESULT |
| `54` | TX_FRAME_RESPONSE |
| `255` | ERROR |

---

# 5. Node-Side Implementation (Contiki-NG)

## 5.1 Create module
```
project/slip-cbor-radio/
  slip-cbor-radio.c
  slip-cbor-radio.h
```

## 5.2 Initialize
- Set up SLIP (`slip_arch_init`)
- Start process:
  ```c
  PROCESS(slip_cbor_radio_process, "SLIP/CBOR Radio");
  ```

## 5.3 Receive path
1. Listen for `slip_event`
2. Extract SLIP frame → buffer
3. Verify CRC16
4. Decode CBOR with TinyCBOR
5. Handle command:
   - GET_PARAM → use `NETSTACK_RADIO.get_value()`
   - SET_PARAM → use `NETSTACK_RADIO.set_value()`
   - TX_RAW_FRAME → call `NETSTACK_RADIO.send()`
   - RSSI_SCAN_START → start scanning process

## 5.4 Send path
1. Create CBOR map  
2. Encode using TinyCBOR  
3. Compute CRC16  
4. Wrap in SLIP frame  
5. Send via `slip_arch_writeb()`

## 5.5 Radio Receive → Raw Frame Events
Hook into radio driver receive callback:
- Read raw frame bytes  
- Read RSSI + LQI  
- Emit CBOR event

## 5.6 RSSI Scanning
- Use a Contiki PROCESS_THREAD or rtimer loop  
- For each channel:
  - Set channel  
  - Wait dwell time  
  - Read RSSI  
  - Emit CBOR events  

## 5.7 Debug Prints
- All `printf()` output remains ASCII and coexists with SLIP frames.

---

# 6. PC-Side Implementation (Python)

Directory:

```
pc/
  serial_radio.py
  slip.py
  crc16.py
  cbor_codec.py
  cli.py
```

## 6.1 SLIP Decoder
- Detect SLIP frames  
- Everything outside frames → debug text  

## 6.2 CRC Checker
- Validate CRC  

## 6.3 CBOR Decoder
- Use `cbor2`

## 6.4 High-Level API
```python
get_param(param)
set_param(param, value)
start_scan(start, end, dwell, opts)
stop_scan()
send_raw_frame(bytes, ch=None)
listen(callback)
```

## 6.5 CLI Tool
Interactive commands:

```
> get channel
> set channel 26
> scan 11 26 5
> sniff
> tx 0102030405
> info
```

---

# 7. Phased Implementation Plan

## Phase 1 — Foundations ✅ COMPLETE
- SLIP TX/RX
- CRC16
- CBOR encode/decode
- Ping & version test

## Phase 2 — Radio Control ✅ COMPLETE
- GET_PARAM / SET_PARAM
- Channel & TX power control

## Phase 3 — Raw Frame TX/RX ✅ COMPLETE
- Radio RX events
- Frame injection

## Phase 4 — RSSI Scanner ✅ COMPLETE
- Multi-channel scanning (slow scan)
- Fast scan with batch RSSI results
- CBOR events

## Phase 5 — CLI Tool ✅ COMPLETE
- Full interactive controller
- RX on/off control
- Jamming mode

## Phase 6 — Web Interface ✅ PARTIAL
- HTTP server for static files
- WebSocket for real-time data streaming
- Spectrum/RSSI visualization
- RX frame display
- Heartbeat monitoring

## Phase 7 — Web Interface Enhancements 🚧 IN PROGRESS

### 7.1 Debug Output Streaming
Stream all device debug output (LOG_INFO, printf, etc.) to web clients.

**Current state:**
- `webserver.py` has `broadcast_debug()` method ready
- `serial_radio.py` has `_debug_callback` that receives debug text
- NOT connected: CLI doesn't forward debug to webserver

**Implementation:**
1. In `cli.py`, register a debug callback when webserver starts:
   ```python
   def _on_debug_text(self, text: str):
       if self._webserver:
           self._webserver.broadcast_debug(text, time.time())
       # Also print to console
       print(text, end='')

   # In do_webserver start:
   self.radio.set_debug_callback(self._on_debug_text)
   ```

2. In web UI (`www/index.html`), add debug console panel:
   - Scrolling text area for debug messages
   - Timestamp display
   - Optional filtering (INFO/DBG/WARN)
   - Clear button

### 7.2 CLI Command Execution via Web
Allow sending arbitrary CLI commands from web interface.

**Current state:**
- `_handle_web_command()` only handles specific structured commands
- No way to execute arbitrary CLI text like "scan 11 26" or "set channel 20"

**Implementation:**
1. Add 'cli_command' handler in `cli.py`:
   ```python
   elif cmd == 'cli_command':
       cli_text = params.get('text', '')
       # Capture output
       import io
       from contextlib import redirect_stdout
       output = io.StringIO()
       with redirect_stdout(output):
           self.onecmd(cli_text)
       return {'output': output.getvalue()}
   ```

2. In web UI, add command input:
   - Text input field for CLI commands
   - Send button / Enter key submit
   - Command history (up/down arrows)
   - Output display area

3. WebSocket message format:
   ```json
   // Request
   {"type": "command", "cmd": "cli_command", "params": {"text": "scan 11 26 5"}}

   // Response
   {"type": "command_result", "cmd": "cli_command", "success": true,
    "result": {"output": "Starting scan..."}}
   ```

### 7.3 Web UI Updates Required
- Add debug console panel (collapsible)
- Add CLI command input with history
- Add output display for command results
- Consider split-pane layout: spectrum | debug+cli

## Phase 8 — Documentation
- Protocol spec
- Testing guide
- Web interface usage guide

---

# 8. IPv6/6LoWPAN Integration

Extend the CBOR protocol to support full IPv6/6LoWPAN networking over IEEE 802.15.4.

## Goals
- Use the same CBOR-over-SLIP protocol for both radio control AND packet transfer
- Support either a Python 6LoWPAN stack or Contiki-NG native border router
- Keep radio control features (scanning, params, sniffing) alongside networking

## New CBOR Commands for Border Router Support

| Opcode | Name | Direction | Purpose |
|--------|------|-----------|---------|
| `50` | `TX_RAW_FRAME` | PC→Node | Send raw 802.15.4 frame (existing) |
| `52` | `RX_FRAME` | Node→PC | Received raw 802.15.4 frame (existing) |
| `64` | `GET_MAC_ADDR` | PC→Node | Query node's IEEE 802.15.4 MAC address |
| `65` | `MAC_ADDR_RESP` | Node→PC | MAC address response (8-byte EUI-64) |
| `66` | `SET_PAN_ID` | PC→Node | Set PAN ID |
| `67` | `SET_SHORT_ADDR` | PC→Node | Set short address |
| `68` | `TX_STATUS` | Node→PC | TX result (success/fail, retries) |

The existing `TX_RAW_FRAME` and `RX_FRAME` commands already handle raw 802.15.4 frames - we just need MAC address query and TX status feedback.

## Option A: Python 6LoWPAN Stack
Full IPv6/6LoWPAN implementation in Python, radio node just does raw TX/RX.

```
Linux Host                           │  Radio Node
─────────────────────────────────────┼────────────────
 ┌─────────┐    ┌──────────────┐     │   ┌──────────┐
 │ tun0    │◄──►│ Python       │     │   │ serial   │
 │ (IPv6)  │    │ 6LoWPAN      │ CBOR│   │ radio    │
 └─────────┘    │ Stack        │◄───UART──►│ (raw     │
                └──────────────┘     │   │  802.15.4)│
                      │              │   └──────────┘
                ┌─────▼─────┐        │
                │ serialradio│        │
                │ CBOR API   │        │
                └───────────┘        │
```

**Python stack handles:**
- IEEE 802.15.4 frame building/parsing
- 6LoWPAN compression/decompression (IPHC, NHC)
- Fragmentation/reassembly
- RPL routing (optional)
- TUN interface to Linux
- Uses `TX_RAW_FRAME` / `RX_FRAME` CBOR commands

## Option B: Contiki-NG Native Border Router (Modified)
Modify the native border router to speak CBOR. The 6LoWPAN stack runs on the host (native), radio node just does raw TX/RX.

```
Linux Host                           │  Radio Node
─────────────────────────────────────┼────────────────
 ┌─────────┐    ┌──────────────────┐ │   ┌──────────┐
 │ tun0    │◄──►│ Contiki-NG       │ │   │ serial   │
 │ (IPv6)  │    │ Native BR        │CBOR│   │ radio    │
 └─────────┘    │ (6LoWPAN + RPL)  │◄──UART─►│ (raw     │
                └──────────────────┘ │   │  802.15.4)│
                                     │   └──────────┘
```

**The native border router already has:**
- Full 6LoWPAN compression/fragmentation
- RPL routing
- TUN interface

**Changes needed to native border router:**
- Replace `slip-dev.c` raw SLIP with CBOR encode/decode
- Add CRC16 wrapper
- Map `!S` command → `TX_RAW_FRAME` CBOR
- Map received frames → `RX_FRAME` CBOR events
- Map `?M` query → `GET_MAC_ADDR` CBOR

## Implementation Steps

1. **Add MAC address query** - `GET_MAC_ADDR` / `MAC_ADDR_RESP`
2. **Add PAN ID / short address config** - `SET_PAN_ID`, `SET_SHORT_ADDR`
3. **Document frame format** - IEEE 802.15.4 frame structure expected
4. **Python stack OR modified border router** - choose one to implement first
5. **TUN interface integration** - bridge to Linux networking

---

# 9. Future Extensions
- Multi-radio support
- PHY mode switching
- Time-synced measurements
- Wi-SUN, BLE adv sniffer mode
- Web-based packet decoder (IEEE 802.15.4 frame parsing)
- Remote access (expose web interface beyond localhost)
- Integration with Wireshark (ZEP protocol or pcap export)  
