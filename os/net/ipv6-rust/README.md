# IPv6-Rust: Memory-Safe IPv6 Stack for Contiki-NG

## Overview

IPv6-Rust is a Rust implementation of the core IPv6 stack for Contiki-NG, designed to provide improved memory safety while maintaining compatibility with the existing C codebase. This implementation demonstrates how Rust's ownership system, type safety, and borrow checker can eliminate entire classes of vulnerabilities in IoT network stacks.

**Current Status**: ✅ **ICMPv6 Echo Request/Reply (ping6) is working end-to-end on native platform**

## Features

### Implemented ✅

- **IPv6 Packet Processing**: Complete IPv6 header parsing and validation
- **ICMPv6 Echo Request/Reply**: Functional ping6 with correct checksum calculation
- **Neighbor Discovery (ND6)**: Neighbor cache management with autofill support
- **Address Management**: IPv6 address lookup and validation via C DS6 integration
- **Buffer Management**: Dual-buffer architecture with proper C/Rust synchronization
- **TCP/IP Integration**: Event-driven packet processing with netstack callbacks
- **Checksum Calculation**: RFC-compliant IPv6 pseudo-header checksums
- **Routing Integration**: Next-hop lookup via C routing tables

### In Progress 🚧

- **ICMPv6 Neighbor Discovery**: Full NS/NA/RS/RA message processing
- **Extension Headers**: Hop-by-Hop, Routing, Fragment headers
- **UDP Protocol**: User Datagram Protocol processing
- **TCP Protocol**: Transmission Control Protocol processing

### Planned 📋

- **6LoWPAN Compression**: Header compression for IEEE 802.15.4
- **Fragment Reassembly**: IPv6 fragmentation support
- **RPL Integration**: Routing Protocol for Low-Power networks
- **Complete C Migration**: Full replacement of C IPv6 stack

## Architecture

### Dual-Stack Design

The implementation uses a **hybrid C/Rust architecture** where Rust handles core packet processing while interfacing with existing C subsystems:

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer (C)                    │
└────────────────────────────┬────────────────────────────────┘
                             │
                     ┌───────┴────────┐
                     │  tcpip-rust.c  │  ← C/Rust Bridge
                     │  (FFI Layer)   │
                     └───────┬────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
        ▼                    ▼                    ▼
┌──────────────┐   ┌──────────────────┐   ┌─────────────┐
│  C DS6 (Addr │   │  Rust IPv6 Core  │   │ C Routing   │
│  Management) │◄──┤                  ├──►│ (uip-ds6)   │
└──────────────┘   │  • ipv6.rs       │   └─────────────┘
                   │  • icmpv6.rs     │
┌──────────────┐   │  • tcpip.rs      │   ┌─────────────┐
│  C Neighbor  │   │  • nd6.rs        │   │ C Netstack  │
│  Discovery   │◄──┤  • checksum.rs   ├──►│ (callbacks) │
└──────────────┘   │  • uipbuf.rs     │   └─────────────┘
                   └─────────┬────────┘
        ┌──────────────────┬─┴────────────────┐
        │                  │                  │
        ▼                  ▼                  ▼
┌──────────────┐   ┌──────────────┐   ┌─────────────┐
│ Rust Buffer  │   │  C Buffer    │   │  Buffer     │
│ UIP_LEN      │◄─►│  uip_len     │◄─►│  Sync Layer │
│ UIP_BUF      │   │  uip_buf     │   │             │
└──────────────┘   └──────────────┘   └─────────────┘
```

### Critical Design Decisions

1. **Dual Buffer Architecture**: Separate Rust and C buffers require explicit synchronization before network callbacks
2. **C DS6 Integration**: Address management remains in C to minimize initial porting scope
3. **Neighbor Autofill**: Link-layer addresses derived from IPv6 IID for native/tun6 platforms (non-standard but convenient)
4. **Event-Driven**: Integrates with Contiki-NG PROCESS system for packet input/output

## Directory Structure

```
os/net/ipv6-rust/
├── Cargo.toml              # Rust project configuration
├── README.md               # This file
├── tcpip-rust.c            # C/Rust FFI bridge layer
├── tcpip-rust.h            # C header for integration
├── src/
│   ├── lib.rs              # Library entry point
│   ├── types.rs            # IPv6 types (Ipv6Addr, Ipv6Header, etc.)
│   ├── buffer.rs           # Packet buffer abstraction
│   ├── uipbuf.rs           # Global buffer management
│   ├── ipv6.rs             # IPv6 packet processing
│   ├── icmpv6.rs           # ICMPv6 implementation ✅ Echo working
│   ├── tcpip.rs            # TCP/IP stack core (input/output/routing)
│   ├── nd6.rs              # Neighbor Discovery cache
│   ├── ds6.rs              # Data Structures (address management)
│   ├── checksum.rs         # Checksum calculation/verification
│   ├── udp.rs              # UDP processing (stub)
│   └── tcp.rs              # TCP processing (stub)
└── target/
    └── x86_64-unknown-linux-gnu/
        └── release/
            └── libuip6_rust.a  # Static library
```

## Building

### Prerequisites

- Rust toolchain 1.70+ with `x86_64-unknown-linux-gnu` target
- Cargo package manager
- Contiki-NG build environment

### Build for Native Platform

```bash
cd os/net/ipv6-rust

# Build for x86_64 Linux (native platform)
cargo build --target x86_64-unknown-linux-gnu --release

# The static library will be at:
# target/x86_64-unknown-linux-gnu/release/libuip6_rust.a
```

### Build Configuration

The `Cargo.toml` is optimized for embedded systems:

```toml
[profile.release]
opt-level = "z"          # Optimize for size
lto = true               # Link-time optimization
codegen-units = 1        # Better optimization at cost of compile time
panic = "abort"          # No unwinding on panic
strip = true             # Strip symbols
```

## Integration

The Rust stack is integrated via the `tcpip-rust.c` bridge layer:

```c
// In your Contiki-NG application
#include "tcpip-rust.h"

PROCESS_THREAD(main_process, ev, data)
{
  PROCESS_BEGIN();

  // Initialize Rust IPv6 stack (called automatically by tcpip.c)
  tcpip_rust_init();

  // Packets are automatically routed to Rust via:
  // tcpip_rust_packet_input() - for incoming packets
  // tcpip_rust_ipv6_output() - for outgoing packets

  PROCESS_END();
}
```

## Testing

### Native Platform Testing

The implementation has been tested on the native platform with working ping6:

```bash
# Build and run
cd examples/hello-world
make TARGET=native
./build/native/hello-world.native

# In another terminal, ping the node
ping6 fd00::302:304:506:708

# Expected output:
# 16 bytes from fd00::302:304:506:708, icmp_seq=1 hlim=64 time=0.708 ms
# 16 bytes from fd00::302:304:506:708, icmp_seq=2 hlim=64 time=0.835 ms
```

### What's Working ✅

1. **ICMPv6 Echo Request → Echo Reply**
   - Checksum verification on incoming Echo Requests
   - Checksum calculation on outgoing Echo Replies
   - Address swapping (src ↔ dst)
   - ID and sequence number preservation

2. **Buffer Synchronization**
   - Rust buffer (`UIP_LEN`, `UIP_ALIGNED_BUF`) syncs to C (`uip_len`, `uip_buf`)
   - Both length and data copied before network callbacks

3. **Neighbor Cache Management**
   - Neighbor lookup from cache
   - Autofill: derive link-layer from IPv6 IID (for native/tun6)
   - Neighbor caching to avoid repeated autofill

4. **Routing Integration**
   - Next-hop lookup via C `uip_ds6_route_lookup()`
   - On-link detection
   - Loopback detection

5. **Network Stack Callbacks**
   - Proper `NETSTACK_IP_PROCESS`/`NETSTACK_IP_DROP` handling
   - Integration with IP processor chain

## Key Fixes and Lessons Learned

### Bug #1: ICMPv6 Checksum Verification ❌→✅

**Problem**: Checksum verification was comparing calculated vs. expected checksum, which is incorrect for one's complement.

**Fix**: Verify by calculating over entire packet (including checksum field) and checking result equals 0 or 0xFFFF.

```rust
// WRONG:
let calc = calculate_checksum(...);
calc == expected_checksum

// CORRECT:
let sum = checksum_over_entire_packet_including_checksum_field(...);
sum == 0 || sum == 0xFFFF
```

### Bug #2: ICMPv6 Echo Reply Checksum Calculation ❌→✅

**Problem**: Checksum field contained old Echo Request checksum during recalculation, resulting in incorrect checksums.

**Fix**: Zero out checksum field before calculating new checksum.

```rust
// Change type to Echo Reply
payload[0] = Icmpv6Type::EchoReply as u8;

// CRITICAL: Zero checksum before recalculating!
payload[2] = 0;
payload[3] = 0;

// Now calculate new checksum
let new_checksum = calculate_checksum(...);
```

### Bug #3: Buffer Length Not Preserved ❌→✅

**Problem**: After processing Echo Request and creating Echo Reply, buffer length was lost, so nothing was sent.

**Fix**: Preserve buffer length after processing.

```rust
match ipv6::process_input(buf) {
    Ok(_) => {
        // CRITICAL: Preserve length for output!
        uipbuf::set_len(len as u16);

        if uipbuf::get_len() > 0 {
            tcpip_rust_ipv6_output();
        }
    }
}
```

### Bug #4: Buffer Data Not Synced to C ❌→✅

**Problem**: Only buffer length was synced to C, not the actual data. Network callbacks check C's `uip_buf`.

**Fix**: Copy both length AND data from Rust to C.

```c
void sync_rust_len_to_c(void) {
  uint16_t *rust_len = uip_len_ptr();
  uint8_t *rust_buf = uip_buf_ptr();

  uip_len = *rust_len;
  if(uip_len > 0 && uip_len <= sizeof(uip_aligned_buf.u8)) {
    memcpy(uip_aligned_buf.u8, rust_buf, uip_len);  // Copy data!
  }
}
```

### Bug #5: Critical Constant Mismatch ❌→✅ (SHOWSTOPPER)

**Problem**: Rust had `NETSTACK_IP_PROCESS = 1`, but C enum defines it as `0`. All packets were being dropped!

**Fix**: Match C enum values exactly.

```rust
// C enum (os/net/netstack.h):
// NETSTACK_IP_PROCESS = 0
// NETSTACK_IP_DROP = 1

// Rust constants (must match!):
const NETSTACK_IP_PROCESS: u8 = 0;  // Was 1 - WRONG!
const NETSTACK_IP_DROP: u8 = 1;
```

## Security Improvements

### Memory Safety Guarantees

1. **No Buffer Overflows**: Rust's bounds checking prevents out-of-bounds access at compile time
2. **No Use-After-Free**: Ownership system ensures memory is not accessed after being freed
3. **No Data Races**: Borrow checker prevents concurrent mutable access
4. **No NULL Pointer Dereferences**: Option types make NULL states explicit

### Example: Safe Buffer Access

```rust
// C (unsafe):
uint8_t data = uip_buf[offset];  // No bounds check! 💥

// Rust (safe):
let data = buffer.get(offset).ok_or(Error::BufferTooSmall)?;  // ✅ Compile-time safety
```

## Performance Considerations

### Zero-Cost Abstractions

Rust's abstractions have no runtime overhead:
- Iterator chains compile to tight loops
- Bounds checks often optimized away by LLVM
- Inline functions eliminate function call overhead

### Memory Footprint

- Static allocation for embedded systems (no heap)
- Fixed-size arrays for neighbors, addresses
- Shared global buffers with C (no duplication except during sync)

## Known Limitations

1. **Dual Buffer Overhead**: Requires copying data from Rust to C buffers before network output
2. **Neighbor Autofill**: Non-standard behavior specific to native/tun6 platforms
3. **C Dependencies**: Still relies on C for routing (uip-ds6-route) and address management (uip-ds6)
4. **Debug Logging**: Extensive printf debugging still present (should be conditionally compiled)

## Future Work

### Short Term

1. Remove debug logging or make it conditional
2. Complete ICMPv6 Neighbor Discovery (NS/NA/RS/RA)
3. Implement extension header processing
4. Add comprehensive unit tests

### Medium Term

1. Port UDP fully to Rust
2. Port TCP fully to Rust
3. Migrate DS6 address management to Rust
4. Migrate routing tables to Rust

### Long Term

1. Full replacement of C IPv6 stack
2. 6LoWPAN compression in Rust
3. RPL routing protocol in Rust
4. Zero-copy buffer architecture

## Debug Logging

The current implementation includes extensive debug logging for troubleshooting:

```c
// In tcpip-rust.c
void rust_debug_log(const char *msg);
void rust_debug_log_int(const char *msg, int val);
```

These are used throughout the Rust code to trace:
- Packet flow (input → processing → output)
- Checksum calculations
- Buffer synchronization
- Neighbor lookup
- Routing decisions

**Note**: This should be made conditional with a compile-time flag for production builds.

## Contributing

Contributions welcome! Please ensure:

- Code follows Rust best practices (clippy warnings addressed)
- Unsafe code is minimized and clearly documented
- FFI boundary is well-defined
- Tests are added for new features
- Documentation is updated

## References

- [RFC 2460](https://www.rfc-editor.org/rfc/rfc2460.html) - IPv6 Specification
- [RFC 4291](https://www.rfc-editor.org/rfc/rfc4291.html) - IPv6 Addressing Architecture
- [RFC 4861](https://www.rfc-editor.org/rfc/rfc4861.html) - Neighbor Discovery for IPv6
- [RFC 4443](https://www.rfc-editor.org/rfc/rfc4443.html) - ICMPv6 for IPv6
- [RFC 4944](https://www.rfc-editor.org/rfc/rfc4944.html) - IPv6 over IEEE 802.15.4
- [The Rust Programming Language](https://doc.rust-lang.org/book/)
- [Contiki-NG Documentation](https://docs.contiki-ng.org/)

## License

3-clause BSD (same as Contiki-NG)

---

**Status Summary**: The Rust IPv6 stack successfully handles ICMPv6 Echo Request/Reply with correct checksums, demonstrating viability of memory-safe IoT networking. Further development will expand protocol support and reduce C dependencies.
