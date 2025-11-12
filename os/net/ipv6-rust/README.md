# uIP6-Rust: A Memory-Safe IPv6 Stack for Contiki-NG

## Overview

uIP6-Rust is a Rust implementation of the IPv6 stack for Contiki-NG, designed to provide improved memory safety and security compared to the original C implementation. This implementation leverages Rust's ownership system, type safety, and borrow checker to eliminate entire classes of vulnerabilities such as buffer overflows, use-after-free, and data races.

## Features

### Core IPv6 Functionality
- **IPv6 Packet Processing**: Full IPv6 header parsing and validation
- **Extension Headers**: Support for Hop-by-Hop, Destination Options, Routing, and Fragment headers
- **Checksum Calculation**: RFC-compliant pseudo-header checksum for upper-layer protocols
- **Address Management**: IPv6 address assignment, lookup, and removal

### ICMPv6 Support
- **Echo Request/Reply**: Ping functionality
- **Neighbor Discovery (ND6)**: RFC 4861 compliant
  - Neighbor Solicitation (NS)
  - Neighbor Advertisement (NA)
  - Router Solicitation (RS)
  - Router Advertisement (RA)
- **Error Messages**: Time Exceeded, Destination Unreachable, Packet Too Big

### Neighbor Management
- **Neighbor Cache**: Efficient neighbor discovery and caching
- **Link-Layer Address Resolution**: IPv6 to MAC address mapping
- **Neighbor Unreachability Detection**: Automatic stale neighbor cleanup

### Protocol Support
- **UDP**: User Datagram Protocol
- **TCP**: Transmission Control Protocol (basic processing)
- **ICMPv6**: Internet Control Message Protocol for IPv6

## Security Improvements

### Memory Safety
1. **No Buffer Overflows**: Rust's bounds checking prevents out-of-bounds access
2. **No Use-After-Free**: Ownership system ensures memory is not accessed after being freed
3. **No Data Races**: Borrow checker prevents concurrent mutable access
4. **Type Safety**: Strong typing prevents type confusion vulnerabilities

### Attack Surface Reduction
- Input validation at every layer
- Safe parsing of packets and headers
- Protected against malformed packets
- Bounds-checked array accesses

## Architecture

```
┌─────────────────────────────────────────────────┐
│              C Application Layer                │
└──────────────────┬──────────────────────────────┘
                   │
                   │ C FFI
                   ▼
┌─────────────────────────────────────────────────┐
│           uip6-rust.h (C Header)                │
└──────────────────┬──────────────────────────────┘
                   │
                   │ FFI Boundary
                   ▼
┌─────────────────────────────────────────────────┐
│              Rust IPv6 Stack                    │
│  ┌───────────────────────────────────────────┐  │
│  │  ipv6.rs - IPv6 Packet Processing        │  │
│  ├───────────────────────────────────────────┤  │
│  │  icmpv6.rs - ICMPv6 & Echo               │  │
│  ├───────────────────────────────────────────┤  │
│  │  nd6.rs - Neighbor Discovery             │  │
│  ├───────────────────────────────────────────┤  │
│  │  ds6.rs - IPv6 Data Structures           │  │
│  ├───────────────────────────────────────────┤  │
│  │  checksum.rs - Checksum Calculation      │  │
│  ├───────────────────────────────────────────┤  │
│  │  types.rs - Core IPv6 Types              │  │
│  └───────────────────────────────────────────┘  │
└─────────────────────────────────────────────────┘
```

## Directory Structure

```
os/net/ipv6-rust/
├── Cargo.toml              # Rust project configuration
├── README.md               # This file
├── uip6-rust.h             # C header for FFI
├── src/
│   ├── lib.rs              # Main library entry point
│   ├── types.rs            # IPv6 data types (addresses, headers)
│   ├── buffer.rs           # Packet buffer management
│   ├── ipv6.rs             # IPv6 packet processing
│   ├── icmpv6.rs           # ICMPv6 implementation
│   ├── nd6.rs              # Neighbor Discovery (ND6)
│   ├── ds6.rs              # IPv6 data structures (DS6)
│   ├── checksum.rs         # Checksum calculation
│   └── ffi.rs              # Foreign Function Interface (C bindings)
└── target/
    └── release/
        └── libuip6_rust.a  # Static library (after build)
```

## Building

### Prerequisites
- Rust toolchain (1.70 or later)
- Cargo package manager

### Build Commands

```bash
# Build release version (optimized for size)
cd os/net/ipv6-rust
cargo build --release

# The static library will be at:
# target/release/libuip6_rust.a
```

## Integration with Contiki-NG

### Including the Rust Stack

1. **Add to Makefile**:
```makefile
# Link with the Rust IPv6 stack
LDFLAGS += -L$(CONTIKI)/os/net/ipv6-rust/target/release
LDFLAGS += -luip6_rust

# Include header path
CFLAGS += -I$(CONTIKI)/os/net/ipv6-rust
```

2. **Initialize in your application**:
```c
#include "uip6-rust.h"

PROCESS_THREAD(main_process, ev, data)
{
  PROCESS_BEGIN();

  // Initialize the Rust IPv6 stack
  uip6_rust_init();

  // Add a link-local address
  uip_ip6addr_t addr;
  uip_ip6addr(&addr, 0xfe80, 0, 0, 0, 0, 0, 0, 0x1);
  uip6_rust_addr_add(&addr, 2); // Manual address

  PROCESS_END();
}
```

3. **Process incoming packets**:
```c
// In packet reception callback
void packet_received(uint8_t *buf, uint16_t len)
{
  int result = uip6_rust_input(buf, len);
  if (result < 0) {
    printf("Error processing packet\n");
  }
}
```

## API Usage Examples

### Address Management

```c
// Add an autoconfigured address
uip_ip6addr_t addr;
uip_ip6addr(&addr, 0x2001, 0xdb8, 0, 0, 0, 0, 0, 0x1);
uip6_rust_addr_add(&addr, 0); // Autoconf

// Check if address exists
if (uip6_rust_addr_lookup(&addr)) {
  printf("Address exists\n");
}

// Remove an address
uip6_rust_addr_rm(&addr);

// Select source address for destination
uip_ip6addr_t dest, src;
uip_ip6addr(&dest, 0x2001, 0xdb8, 0, 0, 0, 0, 0, 0x2);
if (uip6_rust_select_src(&dest, &src) == 0) {
  printf("Selected source address\n");
}
```

### Neighbor Management

```c
// Add a neighbor
uip_ip6addr_t nbr_ip;
linkaddr_t nbr_ll;
uip_ip6addr(&nbr_ip, 0xfe80, 0, 0, 0, 0, 0, 0, 0x2);
memcpy(nbr_ll.u8, "\x02\x00\x00\x00\x00\x00\x00\x02", 8);
uip6_rust_nbr_add(&nbr_ip, &nbr_ll, 0); // Not a router

// Look up neighbor
linkaddr_t ll_out;
if (uip6_rust_nbr_lookup(&nbr_ip, &ll_out) == 0) {
  printf("Neighbor found\n");
}
```

### Address Utilities

```c
uip_ip6addr_t addr;

// Check if multicast
uip_ip6addr(&addr, 0xff02, 0, 0, 0, 0, 0, 0, 1);
if (uip6_rust_is_multicast(&addr)) {
  printf("Address is multicast\n");
}

// Check if link-local
uip_ip6addr(&addr, 0xfe80, 0, 0, 0, 0, 0, 0, 1);
if (uip6_rust_is_link_local(&addr)) {
  printf("Address is link-local\n");
}
```

## Performance Considerations

### Build Optimizations
The Cargo.toml is configured for size optimization:
- `opt-level = "z"` - Optimize for size
- `lto = true` - Link-time optimization
- `codegen-units = 1` - Better optimization
- `panic = "abort"` - Reduce code size

### Memory Usage
The Rust implementation uses static allocation for embedded systems:
- Fixed-size arrays for addresses, neighbors, and prefixes
- No dynamic memory allocation
- Predictable memory footprint

### Runtime Performance
- Zero-cost abstractions: Rust's abstractions have no runtime overhead
- Optimized checksum calculation
- Efficient packet parsing with bounds checking

## Testing

### Unit Tests
```bash
# Run tests (requires std environment)
cargo test
```

### Integration Tests
Create a test application in `examples/` to verify:
- Packet reception and processing
- Address management
- Neighbor discovery
- ICMPv6 echo (ping)

## Future Enhancements

1. **6LoWPAN Compression**: Port sicslowpan compression/decompression
2. **Fragment Reassembly**: Full support for IPv6 fragmentation
3. **UDP/TCP Sockets**: Full socket API in Rust
4. **RPL Integration**: Routing Protocol for Low-Power networks
5. **Multicast**: Enhanced multicast support (MPL, SMRF)
6. **Security**: IPsec and secure neighbor discovery

## License

This implementation follows the Contiki-NG licensing (3-clause BSD).

## Contributing

Contributions are welcome! Please ensure:
- Code follows Rust best practices
- All tests pass
- Documentation is updated
- Unsafe code is minimized and justified

## References

- RFC 2460: Internet Protocol, Version 6 (IPv6) Specification
- RFC 4861: Neighbor Discovery for IP version 6 (IPv6)
- RFC 4291: IP Version 6 Addressing Architecture
- RFC 4944: Transmission of IPv6 Packets over IEEE 802.15.4 Networks
- RFC 6282: Compression Format for IPv6 Datagrams over IEEE 802.15.4-Based Networks
