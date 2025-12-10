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

## Phase 1 — Foundations
- SLIP TX/RX  
- CRC16  
- CBOR encode/decode  
- Ping & version test

## Phase 2 — Radio Control
- GET_PARAM / SET_PARAM  
- Channel & TX power control  

## Phase 3 — Raw Frame TX/RX
- Radio RX events  
- Frame injection  

## Phase 4 — RSSI Scanner
- Multi-channel scanning  
- CBOR events  

## Phase 5 — CLI Tool
- Full interactive controller  

## Phase 6 — Documentation
- Protocol spec  
- Testing guide  

---

# 8. Future Extensions
- Multi-radio support  
- PHY mode switching  
- Time-synced measurements  
- Wi-SUN, BLE adv sniffer mode  
- UDP-over-SLIP tunneling  
