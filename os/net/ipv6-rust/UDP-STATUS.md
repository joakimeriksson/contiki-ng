# UDP Port Status

## Current Status: Phase 3 - Partial Implementation

The UDP layer has been implemented in Rust but requires additional work to fully integrate with the native target.

## Completed ✅

### Rust Implementation (`src/udp.rs`)
- **UDP connection structure** (`UdpConn`) with 340+ lines
- **Connection management**:
  - `udp_new()` - Create new UDP connection
  - `udp_bind()` - Bind to local port
  - Automatic port allocation (4096-31999 range)
  - Connection matching logic
- **UDP packet processing**:
  - `process_udp_input()` - Parse and validate incoming UDP packets
  - Port demultiplexing
  - Connection lookup
  - Checksum validation support
- **UDP output preparation**:
  - `prepare_udp_output()` - Build UDP headers
  - IPv6 header construction
- **FFI exports**:
  - `uip_udp_new_rust()`
  - `uip_udp_bind_rust()`
  - `uip_udp_conn_rust()`
  - `uip_udp_conns_rust()`

### C Integration (`tcpip-rust.c`)
- **Wrapper functions**:
  - `udp_new()` - C wrapper with appstate support
  - `uip_udp_new()` - Standard C API
  - TTL initialization from ds6 interface
  - Application state management

### Error Handling
- Added UDP-specific error types:
  - `PacketTooShort`
  - `InvalidLength`
  - `InvalidPort`
  - `ChecksumMismatch`
  - `NoConnection`
  - `InvalidConnection`

## Remaining Work ⏳

### Symbol Conflicts
**Issue**: Multiple definition errors when linking with `uip6.o`:
```
multiple definition of `uip_udp_conn'
multiple definition of `uip_udp_new'
```

**Root Cause**: The native target still compiles `uip6.c` which provides:
- UDP implementation (we're replacing this)
- TCP implementation (we haven't ported yet)
- Buffer management (`uip_buf`, `uip_len`)

### Missing Symbols
```
undefined reference to `uip_buf'
undefined reference to `tcpip_uipcall'
```

These are needed by `uip6.c` for packet buffer access and application callbacks.

## Solutions

### Option 1: Complete Buffer Management Port (Recommended)
Port the buffer management from `uip6.c` to Rust:
- Define `uip_buf` in Rust
- Define `uip_len` in Rust
- Export to C via FFI
- Remove dependency on `uip6.c`

### Option 2: Selective Compilation (Medium effort)
Modify `uip6.c` to conditionally compile:
```c
#if !UIP6_RUST_CONF_ENABLE
/* UDP implementation */
#endif
```

Keep TCP/buffer parts, exclude UDP when Rust enabled.

### Option 3: Link-Time Resolution (Quick fix)
Use linker flags to resolve conflicts:
```makefile
LDFLAGS += -Wl,--allow-multiple-definition
```

Then provide stub implementations of missing symbols.

## Testing Status

### Compilation
- ✅ **Rust library**: Compiles successfully (with warnings)
- ✅ **C glue layer**: Compiles successfully
- ❌ **Native linking**: Fails due to symbol conflicts

### Target Support
- ✅ **Cooja simulator**: Should work (uses different symbol resolution)
- ❌ **Native target**: Blocked by linking issues
- ❓ **Embedded targets**: Unknown, likely same issues as native

## Code Statistics

| Component | Lines | Status |
|-----------|-------|--------|
| `udp.rs` | 340 | ✅ Complete |
| UDP FFI in `tcpip-rust.c` | 60 | ✅ Complete |
| Buffer management | 0 | ⏳ Not started |
| Integration tests | 0 | ⏳ Not started |

## Next Steps

To complete UDP for native target:

1. **Short term** (1-2 hours):
   - Add `uip_buf` and `uip_len` definitions in Rust
   - Provide `tcpip_uipcall()` stub
   - Use linker flag workaround

2. **Medium term** (4-6 hours):
   - Port buffer management (`uip6.c` buffer code)
   - Port `tcpip_uipcall()` properly
   - Add UDP input/output integration in `tcpip.rs`

3. **Long term** (full Phase 3):
   - Port TCP state machine
   - Port `simple-udp.c` to Rust
   - Complete transport layer

## Usage (Once Complete)

With Rust UDP, applications can use standard APIs:

```c
// Create UDP connection
struct uip_udp_conn *conn = udp_new(&remote_addr, UIP_HTONS(1234), NULL);

// Bind to local port
udp_bind(conn, 5678);

// Send/receive works through existing simple-udp API
simple_udp_sendto(&udp_conn, data, len, &dest);
```

The Rust implementation provides:
- **Memory safety**: No buffer overflows, use-after-free
- **Type safety**: Strong typing for ports, addresses
- **Same API**: Drop-in replacement for C UDP

## Performance

Expected performance (based on Phase 1 & 2):
- **Binary size**: +2-3KB for UDP code
- **Runtime overhead**: Negligible (<1%)
- **Memory usage**: Identical to C implementation

## Related Files

- `os/net/ipv6-rust/src/udp.rs` - Rust UDP implementation
- `os/net/ipv6-rust/tcpip-rust.c` - C wrappers
- `os/net/ipv6/uip6.c` - Original C implementation (conflicting)
- `os/net/ipv6/simple-udp.c` - Application-level UDP helpers

## References

- [RFC 768](https://www.rfc-editor.org/rfc/rfc768) - User Datagram Protocol
- [RFC 2460](https://www.rfc-editor.org/rfc/rfc2460) - IPv6 Specification
- PORT-STATUS.md - Overall port status
- DUAL-STACK-ARCHITECTURE.md - Architecture overview
