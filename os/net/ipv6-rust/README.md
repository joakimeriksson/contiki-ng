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

## Integration with Contiki-NG Netstack

### Overview

The Rust IPv6 stack integrates with Contiki-NG through the **tcpip-rust.c** FFI bridge, which exposes Rust functions to C and provides Rust with access to C subsystems. The integration uses a **hybrid architecture** where Rust handles packet processing while C manages platform-specific I/O, routing tables, and address management.

### Packet Input Flow

```
1. Physical Interface (tun6)
   ↓
2. tun6_net_input() [arch/cpu/native/net/tun6-net.c]
   - Reads packet from /dev/tun0
   - Stores in C uip_buf
   - Sets C uip_len
   ↓
3. tcpip_input() [os/net/ipv6/tcpip.c]
   - Entry point to TCP/IP stack
   ↓
4. tcpip_rust_packet_input() [os/net/ipv6-rust/tcpip-rust.c] ← FFI BOUNDARY
   - Copies C buffer to Rust buffer (uip_buf → UIP_ALIGNED_BUF)
   - Copies C length to Rust length (uip_len → UIP_LEN)
   ↓
5. tcpip_rust_input() [Rust: src/tcpip.rs]
   - Rust packet processing begins
   ↓
6. ipv6::process_input() [Rust: src/ipv6.rs]
   - Parse IPv6 header
   - Validate addresses
   - Check destination (is it for us?)
   - Dispatch to protocol handler
   ↓
7. icmpv6::process_input() [Rust: src/icmpv6.rs]
   - Verify ICMPv6 checksum
   - Parse ICMP type (Echo Request, NS, NA, etc.)
   - Generate Echo Reply in place
   - Swap src/dst addresses
   - Recalculate checksum
   ↓
8. tcpip_rust_ipv6_output() [Rust: src/tcpip.rs]
   - Called if there's a reply to send
```

### Packet Output Flow

```
1. tcpip_rust_ipv6_output() [Rust: src/tcpip.rs]
   - Checks if destination is our own address (loopback)
   - Determines if destination is on-link or needs routing
   ↓
2. tcpip_rust_get_nexthop() [C: tcpip-rust.c] ← FFI CALL TO C
   - Calls C uip_ds6_route_lookup()
   - Returns next-hop IPv6 address
   ↓
3. nd6::lookup_neighbor() [Rust: src/nd6.rs]
   - Look up link-layer address for next hop
   - If not found: autofill from IPv6 IID (native platform)
   - Cache neighbor for future use
   ↓
4. tcpip_rust_output() [Rust: src/tcpip.rs]
   ↓
5. sync_rust_len_to_c() [C: tcpip-rust.c] ← FFI BOUNDARY
   - CRITICAL: Copy Rust buffer to C buffer
   - memcpy(uip_buf, UIP_ALIGNED_BUF, UIP_LEN)
   - uip_len = UIP_LEN
   ↓
6. netstack_process_ip_callback() [C: os/net/netstack.c]
   - Iterate through IP processor list
   - Each processor can return PROCESS or DROP
   - Returns NETSTACK_IP_PROCESS if all processors allow
   ↓
7. tcpip_rust_network_output() [C: tcpip-rust.c]
   - NETSTACK_NETWORK.output() (e.g., tun6_net_driver)
   ↓
8. tun6_net_output() [arch/cpu/native/net/tun6-net.c]
   - write(tunfd, uip_buf, uip_len)
   - Packet sent to /dev/tun0
   ↓
9. Physical Interface (tun6)
```

### FFI Boundary Functions

#### C Functions Called from Rust

Defined in `tcpip-rust.c` and declared in Rust's `extern "C"` blocks:

```c
// Buffer access (Rust needs pointers to C buffers)
uint8_t* uip_buf_ptr(void);
uint16_t* uip_len_ptr(void);

// Buffer synchronization
void sync_rust_len_to_c(void);  // Copy Rust buffer → C buffer

// Routing integration
int tcpip_rust_get_nexthop(const uip_ipaddr_t *dest, uip_ipaddr_t *out);
int tcpip_rust_is_addr_onlink(const uip_ipaddr_t *addr);
int tcpip_rust_is_my_addr(const uip_ipaddr_t *addr);

// Neighbor discovery integration (C DS6)
int tcpip_rust_nbr_lookup(const uip_ipaddr_t *ipaddr, uip_lladdr_t *lladdr);
int tcpip_rust_nbr_add(const uip_ipaddr_t *ipaddr, const uip_lladdr_t *lladdr,
                       int is_router, int state);

// Netstack callbacks
int netstack_process_ip_callback(uint8_t type, const linkaddr_t *localdest);
int tcpip_rust_network_output(const uint8_t *lladdr);

// Debug logging
void rust_debug_log(const char *msg);
void rust_debug_log_int(const char *msg, int val);
```

#### Rust Functions Called from C

Exported from Rust via `#[no_mangle] pub extern "C"`:

```rust
// Initialization
pub extern "C" fn tcpip_rust_init();

// Packet input entry point
pub extern "C" fn tcpip_rust_packet_input() -> i32;

// IPv6 output entry point
pub extern "C" fn tcpip_rust_ipv6_output() -> i32;

// Link-layer output
pub extern "C" fn tcpip_rust_output(lladdr: *const u8) -> i32;
```

### C Subsystem Integration

#### 1. Address Management (DS6)

The Rust stack uses C's `uip-ds6` for address management:

```
Rust → tcpip_rust_is_my_addr() → [C] uip_ds6_is_my_addr() → DS6 Address Table
```

**Why**: DS6 is deeply integrated with Contiki-NG. Porting it to Rust would require significant effort without immediate security benefit.

**How it works**:
- C maintains the address table in `uip_ds6_if.addr_list[]`
- Rust calls through FFI to check if addresses are local
- Future work: Migrate DS6 to Rust for full memory safety

#### 2. Routing Tables (uip-ds6-route)

The Rust stack queries C routing tables for next-hop determination:

```
Rust → tcpip_rust_get_nexthop() → [C] uip_ds6_route_lookup() → Routing Table
```

**Why**: Routing tables are platform-specific and interact with RPL routing protocol.

**How it works**:
- C maintains routes in `uip_ds6_routing_table[]`
- Rust queries for next hop given a destination
- C returns either the destination (if on-link) or gateway address

#### 3. Neighbor Discovery (ND6)

Hybrid approach - Rust has its own neighbor cache but integrates with C for certain operations:

```
Rust nd6::lookup_neighbor() → Check Rust cache
                            ↓ (if miss)
                            → tcpip_rust_nbr_lookup() → [C] uip_ds6_nbr_lookup()
```

**Special behavior on native platform**:
- **Autofill**: If neighbor not in cache, derive link-layer address from IPv6 IID
- Formula: `lladdr[0..7] = ipv6_addr[8..15]`, flip bit 0x02 in first byte
- Cache the autofilled neighbor for future lookups
- This is **non-standard** but convenient for native/tun6 testing

#### 4. Netstack Callback Chain

Before sending packets to the network, the stack must call `netstack_process_ip_callback()`:

```c
enum netstack_ip_action
netstack_process_ip_callback(uint8_t type, const linkaddr_t *localdest)
{
  enum netstack_ip_action action = NETSTACK_IP_PROCESS;

  for(each processor in ip_processor_list) {
    action = processor->process_output(localdest);
    if(action != NETSTACK_IP_PROCESS)
      return action;  // DROP/IGNORE
  }
  return NETSTACK_IP_PROCESS;  // OK to send
}
```

**Important**:
- Processors can be added by routing protocols (RPL), firewalls, hooks
- Each processor can return `NETSTACK_IP_PROCESS` (0) or `NETSTACK_IP_DROP` (1)
- If ANY processor returns DROP, the packet is discarded
- **Critical bug we fixed**: Rust had these constants backwards!

### Buffer Synchronization

The dual-buffer architecture requires careful synchronization:

#### Why Dual Buffers?

- **C buffer**: `uip_buf[]` and `uip_len` - used by network drivers and callbacks
- **Rust buffer**: `UIP_ALIGNED_BUF[]` and `UIP_LEN` - owned by Rust for safe processing

**Why not share?**: Rust needs ownership to guarantee safety. Sharing mutable global state between C and Rust is unsafe.

#### Synchronization Points

**Input (C → Rust)**:
```c
void tcpip_rust_packet_input(void) {
  uint16_t *rust_len = uip_len_ptr();
  uint8_t *rust_buf = uip_buf_ptr();

  // Copy C buffer to Rust
  *rust_len = uip_len;
  memcpy(rust_buf, uip_buf, uip_len);

  // Process in Rust
  tcpip_rust_input();
}
```

**Output (Rust → C)**:
```c
void sync_rust_len_to_c(void) {
  uint16_t *rust_len = uip_len_ptr();
  uint8_t *rust_buf = uip_buf_ptr();

  // Copy Rust buffer back to C
  uip_len = *rust_len;
  if(uip_len > 0 && uip_len <= sizeof(uip_aligned_buf.u8)) {
    memcpy(uip_aligned_buf.u8, rust_buf, uip_len);
  }
}
```

**Critical**: `sync_rust_len_to_c()` must be called BEFORE `netstack_process_ip_callback()` because:
- The callback checks C's `uip_len` to see if there's data to send
- Network drivers (tun6) read from C's `uip_buf[]`
- **Bug we fixed**: Initially only synced length, not data!

### Application Integration

For applications using the Rust IPv6 stack:

```c
// In your Contiki-NG application
#include "tcpip-rust.h"

PROCESS_THREAD(main_process, ev, data)
{
  PROCESS_BEGIN();

  // Initialize Rust IPv6 stack (called automatically by tcpip.c if enabled)
  tcpip_rust_init();

  // Packets are automatically routed to Rust via:
  // - tcpip_rust_packet_input() - for incoming packets
  // - tcpip_rust_ipv6_output() - for outgoing packets

  // No application changes needed for basic IPv6 operation!

  PROCESS_END();
}
```

### Build System Integration

The Rust library is built as a static archive and linked with the C code:

```makefile
# In Contiki-NG Makefile
LDFLAGS += -L$(CONTIKI)/os/net/ipv6-rust/target/x86_64-unknown-linux-gnu/release
LDFLAGS += -luip6_rust

# The Rust library is built before C compilation
$(OBJECTDIR)/tcpip-rust.o: $(CONTIKI)/os/net/ipv6-rust/target/.../libuip6_rust.a
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
