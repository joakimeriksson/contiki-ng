# Native Border Router CBOR Protocol Upgrade Plan

## Overview

Upgrade the native border router to use the CBOR-based protocol from serialradio, replacing the current ASCII command protocol (`!S`, `!R`, `?M`, etc.). This provides:

- **Reliable serial communication** - CRC16 on all frames detects corruption
- **Extensible protocol** - CBOR is self-describing and easy to extend
- **Better diagnostics** - Structured error responses, heartbeats
- **New features** - Spectrum scanning, RSSI monitoring available to host

## Current Architecture

### Files to Modify

| File | Purpose | Changes Needed |
|------|---------|----------------|
| `os/services/rpl-border-router/native/slip-dev.c` | SLIP I/O, serial handling | Add CRC16 verify/generate |
| `os/services/rpl-border-router/native/border-router-cmds.c` | Command parsing | Replace ASCII with CBOR decoder |
| `os/services/rpl-border-router/native/border-router-mac.c` | TX packet building | Build CBOR messages instead of `!S` |
| `os/services/rpl-border-router/native/border-router.h` | Public API | Add new CBOR-related declarations |

### Current Protocol (ASCII)

```
TX: !S [SID] [attrs] [payload]    Host→Radio: Send packet
    !R [SID] [status] [txcount]   Radio→Host: TX status

RX: Raw SLIP frame (no prefix)    Radio→Host: Received packet

Params: !V / ?V                   Set/Get radio parameters
MAC:    ?M / !M                   Query/Response MAC address
```

## New Protocol (CBOR)

### Framing

```
[SLIP_END] [CBOR message] [CRC16-LE] [SLIP_END]
```

- SLIP framing with escape sequences (0xC0 → 0xDB 0xDC, 0xDB → 0xDB 0xDD)
- CRC16 appended before final SLIP_END (little-endian, Contiki-NG algorithm)
- CBOR message is a map with single-character keys

### Message Format

All messages are CBOR maps with key `"t"` (type) indicating the message type.

#### Commands (Host → Radio)

**TX_RAW_FRAME (type=4)**
```cbor
{
  "t": 4,           // SRADIO_CMD_TX_RAW_FRAME
  "i": <msg_id>,    // Message ID for response correlation
  "f": <bytes>      // Raw 802.15.4 frame data
}
```

**GET_PARAM (type=2)**
```cbor
{
  "t": 2,           // SRADIO_CMD_GET_PARAM
  "i": <msg_id>,
  "p": <param>      // RADIO_PARAM_* constant
}
```

**SET_PARAM (type=3)**
```cbor
{
  "t": 3,           // SRADIO_CMD_SET_PARAM
  "i": <msg_id>,
  "p": <param>,
  "v": <value>
}
```

**RX_ON (type=5) / RX_OFF (type=6)**
```cbor
{"t": 5, "i": <msg_id>}   // Enable promiscuous RX
{"t": 6, "i": <msg_id>}   // Disable promiscuous RX
```

#### Events/Responses (Radio → Host)

**RX_FRAME (type=51)**
```cbor
{
  "t": 51,          // SRADIO_EVT_RX_FRAME
  "f": <bytes>,     // Raw 802.15.4 frame
  "r": <rssi>,      // RSSI in dBm (signed)
  "l": <lqi>        // Link Quality Indicator
}
```

**TX_DONE (type=52)**
```cbor
{
  "t": 52,          // SRADIO_EVT_TX_DONE
  "i": <msg_id>,    // Echoed from TX_RAW_FRAME
  "s": <status>     // MAC TX status (0=OK, etc.)
}
```

**PARAM_RESPONSE (type=54)**
```cbor
{
  "t": 54,
  "i": <msg_id>,
  "p": <param>,
  "v": <value>
}
```

**ERROR (type=59)**
```cbor
{
  "t": 59,
  "i": <msg_id>,
  "x": <error_code>
}
```

**HEARTBEAT (type=50)**
```cbor
{
  "t": 50,
  "n": <seq>,
  "V": "serial-radio-x.x"
}
```

### Key Mappings

| Key | Field | Type |
|-----|-------|------|
| `t` | Type/opcode | unsigned |
| `i` | Message ID | unsigned |
| `p` | Radio parameter | unsigned |
| `v` | Value | signed/unsigned |
| `f` | Frame data | bytes |
| `r` | RSSI | signed |
| `l` | LQI | unsigned |
| `c` | Channel | unsigned |
| `s` | Status/Start | unsigned |
| `e` | End channel | unsigned |
| `x` | Error code | unsigned |
| `V` | Version | text |
| `n` | Sequence | unsigned |

## Implementation Plan

### Phase 1: Add CBOR and CRC16 Support

**Step 1.1: Include CBOR library**
- Add `os/lib/cbor.c` to native border router build
- CBOR already exists in Contiki-NG at `os/lib/cbor.h`

**Step 1.2: Add CRC16 support**
- Use existing `os/lib/crc16.h` and `crc16_data()`
- Add CRC16 verification in `slip-dev.c` frame reception
- Add CRC16 generation in `slip-dev.c` frame transmission

**Files:**
- `os/services/rpl-border-router/native/Makefile.rpl-border-router` - Add CBOR module
- `os/services/rpl-border-router/native/slip-dev.c` - Add CRC16 handling

### Phase 2: Replace Command Protocol

**Step 2.1: Create CBOR message builder/parser**

New file: `os/services/rpl-border-router/native/border-router-cbor.c`
```c
/* Build TX_RAW_FRAME command */
size_t br_cbor_build_tx_frame(uint8_t *buf, size_t buf_size,
                               uint8_t msg_id, const uint8_t *frame, size_t frame_len);

/* Build GET_PARAM command */
size_t br_cbor_build_get_param(uint8_t *buf, size_t buf_size,
                                uint8_t msg_id, radio_param_t param);

/* Parse incoming CBOR message */
typedef struct {
  uint8_t type;
  uint8_t msg_id;
  int16_t rssi;
  uint8_t lqi;
  uint8_t status;
  const uint8_t *frame;
  size_t frame_len;
  radio_param_t param;
  radio_value_t value;
} br_cbor_msg_t;

bool br_cbor_parse(const uint8_t *data, size_t len, br_cbor_msg_t *msg);
```

**Step 2.2: Update border-router-mac.c**

Replace `send_packet()`:
```c
// OLD: Build "!S" + SID + attrs + payload
// NEW: Build CBOR TX_RAW_FRAME message

static void send_packet(mac_callback_t sent, void *ptr)
{
  uint8_t sid = setup_callback(sent, ptr);
  uint8_t buf[256];

  size_t len = br_cbor_build_tx_frame(buf, sizeof(buf), sid,
                                       packetbuf_hdrptr(),
                                       packetbuf_totlen());
  write_to_slip(buf, len);  // Will add CRC16 and SLIP framing
}
```

**Step 2.3: Update border-router-cmds.c**

Replace ASCII command parsing with CBOR:
```c
int border_router_cmd_handler(const uint8_t *data, int data_len)
{
  br_cbor_msg_t msg;

  if(!br_cbor_parse(data, data_len, &msg)) {
    return 0;  // Not a valid CBOR message
  }

  switch(msg.type) {
  case SRADIO_EVT_RX_FRAME:
    // Forward frame to network stack
    packetbuf_clear();
    packetbuf_copyfrom(msg.frame, msg.frame_len);
    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, msg.rssi);
    packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, msg.lqi);
    NETSTACK_MAC.input();
    break;

  case SRADIO_EVT_TX_DONE:
    packet_sent(msg.msg_id, msg.status, 1);
    break;

  case SRADIO_EVT_PARAM_RESPONSE:
    // Handle parameter response
    break;

  case SRADIO_EVT_HEARTBEAT:
    // Update connection status
    break;
  }
  return 1;
}
```

**Step 2.4: Update slip-dev.c**

Modify `serial_input()` to handle CRC16:
```c
static void slip_packet_input(unsigned char *data, int len)
{
  // Verify CRC16
  if(len < 2) return;

  uint16_t received_crc = data[len-2] | (data[len-1] << 8);
  uint16_t computed_crc = crc16_data(data, len-2, 0);

  if(received_crc != computed_crc) {
    LOG_WARN("CRC mismatch: got 0x%04x, expected 0x%04x\n",
             received_crc, computed_crc);
    return;
  }

  // Pass to command handler (without CRC bytes)
  cmd_input(data, len - 2);
}
```

Modify `write_to_slip()` to add CRC16:
```c
void write_to_slip(const uint8_t *buf, int len)
{
  // Compute CRC16
  uint16_t crc = crc16_data(buf, len, 0);

  // SLIP encode: data + CRC
  slip_send(buf, len);
  slip_send_byte(crc & 0xFF);
  slip_send_byte((crc >> 8) & 0xFF);
  slip_send_byte(SLIP_END);
}
```

### Phase 3: MAC Address Handling

**Step 3.1: Query MAC on startup**

In `border_router_start()`:
```c
// Send GET_PARAM for MAC address
uint8_t buf[32];
size_t len = br_cbor_build_get_param(buf, sizeof(buf), 0,
                                      RADIO_PARAM_64BIT_ADDR);
write_to_slip(buf, len);
```

**Step 3.2: Handle MAC response**

In command handler, when receiving PARAM_RESPONSE for 64BIT_ADDR:
```c
case SRADIO_EVT_PARAM_RESPONSE:
  if(msg.param == RADIO_PARAM_64BIT_ADDR) {
    memcpy(&uip_lladdr, msg.frame, 8);
    border_router_set_mac(msg.frame);
  }
  break;
```

### Phase 4: Testing

1. **Unit tests** for CBOR builder/parser
2. **Integration test** with serialradio firmware
3. **Stress test** serial communication reliability
4. **Compare** with old slip-radio to verify functionality

### Phase 5: Optional Enhancements

1. **Heartbeat monitoring** - Detect radio disconnection
2. **Spectrum API** - Expose fast scan to host applications
3. **Retry mechanism** - Resend commands on timeout
4. **Backwards compatibility** - Auto-detect protocol (ASCII vs CBOR)

## File Summary

### New Files
- `os/services/rpl-border-router/native/border-router-cbor.c` - CBOR message handling
- `os/services/rpl-border-router/native/border-router-cbor.h` - CBOR API

### Modified Files
- `os/services/rpl-border-router/native/slip-dev.c` - CRC16, CBOR framing
- `os/services/rpl-border-router/native/border-router-cmds.c` - CBOR parsing
- `os/services/rpl-border-router/native/border-router-mac.c` - CBOR TX building
- `os/services/rpl-border-router/native/Makefile.rpl-border-router` - Add modules

### Shared with serialradio
- `os/lib/cbor.c` / `os/lib/cbor.h` - CBOR encoder/decoder
- `os/lib/crc16.c` / `os/lib/crc16.h` - CRC16 algorithm
- Protocol constants (opcodes, keys) - Need shared header

## Protocol Constants (Shared Header)

Create `os/services/serial-radio/serial-radio-protocol.h`:
```c
/* Command types (Host → Radio) */
#define SRADIO_CMD_PING             0
#define SRADIO_CMD_GET_INFO         1
#define SRADIO_CMD_GET_PARAM        2
#define SRADIO_CMD_SET_PARAM        3
#define SRADIO_CMD_TX_RAW_FRAME     4
#define SRADIO_CMD_RX_ON            5
#define SRADIO_CMD_RX_OFF           6

/* Event types (Radio → Host) */
#define SRADIO_EVT_HEARTBEAT        50
#define SRADIO_EVT_RX_FRAME         51
#define SRADIO_EVT_TX_DONE          52
#define SRADIO_EVT_PONG             53
#define SRADIO_EVT_PARAM_RESPONSE   54
#define SRADIO_EVT_ERROR            59

/* CBOR map keys */
#define SRADIO_KEY_TYPE     't'
#define SRADIO_KEY_MSG_ID   'i'
#define SRADIO_KEY_PARAM    'p'
#define SRADIO_KEY_VALUE    'v'
#define SRADIO_KEY_FRAME    'f'
#define SRADIO_KEY_RSSI     'r'
#define SRADIO_KEY_LQI      'l'
#define SRADIO_KEY_CHANNEL  'c'
#define SRADIO_KEY_STATUS   's'
#define SRADIO_KEY_ERROR    'x'
#define SRADIO_KEY_VERSION  'V'
#define SRADIO_KEY_SEQ      'n'
```

## Timeline Estimate

| Phase | Description | Complexity |
|-------|-------------|------------|
| 1 | CBOR + CRC16 infrastructure | Low |
| 2 | Replace command protocol | Medium |
| 3 | MAC address handling | Low |
| 4 | Testing | Medium |
| 5 | Enhancements | Optional |

## Benefits Summary

1. **Reliability** - CRC16 catches serial corruption (USB disconnects, noise)
2. **Extensibility** - Easy to add new commands/events without breaking protocol
3. **Diagnostics** - Heartbeats detect radio failures, structured errors
4. **Features** - Spectrum scanning, jamming detection available to host
5. **Consistency** - Same protocol for CLI tools and border router
6. **Maintainability** - Single radio firmware works with multiple hosts
