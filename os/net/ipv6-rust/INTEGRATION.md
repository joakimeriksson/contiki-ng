# Integration Guide for uIP6-Rust

This document provides step-by-step instructions for integrating the Rust-based IPv6 stack into your Contiki-NG project.

## Quick Start

### 1. Prerequisites

Ensure you have the Rust toolchain installed:

```bash
# Check if Rust is installed
rustc --version
cargo --version

# If not installed, install from https://rustup.rs/
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

### 2. Build the Rust Library

```bash
cd os/net/ipv6-rust
cargo build --release
```

The compiled static library will be at: `target/release/libuip6_rust.a`

### 3. Include in Your Application

#### Option A: Using the Makefile Fragment

Add to your application's Makefile:

```makefile
# Include uIP6-Rust
include $(CONTIKI)/os/net/ipv6-rust/Makefile.uip6-rust

# Make sure to build the Rust library
all: uip6-rust-build
```

#### Option B: Manual Configuration

Add to your Makefile:

```makefile
# Include header
CFLAGS += -I$(CONTIKI)/os/net/ipv6-rust

# Link with Rust library
LDFLAGS += -L$(CONTIKI)/os/net/ipv6-rust/target/release
LDFLAGS += -luip6_rust

# Build the library
$(CONTIKI)/os/net/ipv6-rust/target/release/libuip6_rust.a:
	cd $(CONTIKI)/os/net/ipv6-rust && cargo build --release
```

### 4. Initialize in Your Application

```c
#include "uip6-rust.h"

PROCESS_THREAD(your_process, ev, data)
{
  PROCESS_BEGIN();

  /* Initialize the Rust IPv6 stack */
  uip6_rust_init();

  /* Your application code */

  PROCESS_END();
}
```

## Integration Patterns

### Pattern 1: Replacing the C IPv6 Stack

To completely replace the C stack with the Rust implementation:

1. **Disable the C IPv6 stack** in your `project-conf.h`:
```c
#define UIP_CONF_IPV6 0
```

2. **Use Rust for all IPv6 operations**:
```c
/* Instead of uip_ds6_addr_add() */
uip6_rust_addr_add(&addr, addr_type);

/* Instead of uip_ds6_addr_lookup() */
uip6_rust_addr_lookup(&addr);
```

### Pattern 2: Hybrid Approach (Gradual Migration)

Use both stacks side-by-side during migration:

1. **Keep C stack enabled** but use Rust for new features
2. **Process packets in Rust first**, fallback to C if needed:

```c
void packet_input(uint8_t *buf, uint16_t len)
{
  /* Try Rust stack first */
  int result = uip6_rust_input(buf, len);

  if(result < 0) {
    /* Fallback to C stack */
    uip_input();
  }
}
```

### Pattern 3: Testing and Validation

Use Rust stack in parallel for testing:

1. **Process packets in both stacks**
2. **Compare results** for validation:

```c
void packet_input(uint8_t *buf, uint16_t len)
{
  /* Process in Rust */
  int rust_result = uip6_rust_input(buf, len);

  /* Process in C */
  memcpy(uip_buf, buf, len);
  uip_len = len;
  uip_input();
  int c_result = (uip_len > 0) ? 0 : -1;

  /* Compare and log differences */
  if(rust_result != c_result) {
    LOG_WARN("Stack mismatch: Rust=%d, C=%d\n", rust_result, c_result);
  }
}
```

## Platform-Specific Notes

### ARM Cortex-M

For ARM Cortex-M platforms, you may need to specify the target:

```bash
# Install the target
rustup target add thumbv7em-none-eabi

# Build for the target
cd os/net/ipv6-rust
cargo build --release --target thumbv7em-none-eabi
```

Update your Makefile:
```makefile
LDFLAGS += -L$(CONTIKI)/os/net/ipv6-rust/target/thumbv7em-none-eabi/release
```

### RISC-V

For RISC-V platforms:

```bash
rustup target add riscv32imac-unknown-none-elf
cargo build --release --target riscv32imac-unknown-none-elf
```

### Native (Linux/macOS)

For native platforms, the default target works:

```bash
cargo build --release
```

## Memory Configuration

### Stack Size

The Rust implementation uses static allocation with fixed-size arrays. You can adjust the sizes by modifying `src/ds6.rs` and `src/nd6.rs`:

```rust
// In ds6.rs
const MAX_ADDRS: usize = 4;      // Max IPv6 addresses
const MAX_PREFIXES: usize = 2;   // Max prefixes
const MAX_MADDRS: usize = 4;     // Max multicast addresses

// In nd6.rs
const MAX_NEIGHBORS: usize = 8;  // Max neighbor cache entries
```

After modifying, rebuild the library:
```bash
cargo clean
cargo build --release
```

## Troubleshooting

### Build Errors

**Error: `cargo: command not found`**
- Install Rust: https://rustup.rs/

**Error: `linking with cc failed`**
- Ensure you have a C compiler installed
- Check that LDFLAGS are correct in your Makefile

**Error: `cannot find -luip6_rust`**
- Build the Rust library first: `cd os/net/ipv6-rust && cargo build --release`
- Check that LDFLAGS points to the correct directory

### Runtime Issues

**Packet processing fails**
- Check buffer size: must be at least 1280 bytes (IPv6 minimum MTU)
- Verify packet is valid IPv6
- Enable logging in Rust code for debugging

**Address not found**
- Ensure `uip6_rust_init()` was called
- Check that address was added with correct type
- Verify address format (network byte order)

**Neighbor lookup fails**
- Neighbors must be added before lookup
- Check that Neighbor Discovery is processing correctly
- Verify link-layer address format

## Performance Tuning

### Size Optimization

The default configuration optimizes for code size. To optimize for speed instead, edit `Cargo.toml`:

```toml
[profile.release]
opt-level = 3          # Optimize for speed (was "z")
lto = true
codegen-units = 1
```

### Link-Time Optimization

For maximum optimization, enable LTO in your C compiler flags:

```makefile
CFLAGS += -flto
LDFLAGS += -flto
```

## Testing

### Unit Tests

Run Rust unit tests:
```bash
cd os/net/ipv6-rust
cargo test
```

### Integration Tests

1. **Build the example**:
```bash
cd examples/your-test-app
make TARGET=native
```

2. **Run the example**:
```bash
./example-app.native
```

3. **Test ping**:
```bash
# In another terminal
ping6 fe80::200:0:0:1%tap0
```

## Migration Checklist

- [ ] Build Rust library successfully
- [ ] Include header in application
- [ ] Initialize Rust stack on startup
- [ ] Add test addresses
- [ ] Process incoming packets
- [ ] Verify ICMPv6 echo (ping)
- [ ] Test neighbor discovery
- [ ] Validate UDP packet processing
- [ ] Validate TCP packet processing
- [ ] Performance testing
- [ ] Memory usage profiling
- [ ] Remove C stack (if full replacement)

## Support

For issues and questions:
- Check the README.md for API documentation
- Review example-usage.c for usage patterns
- File an issue on GitHub
- Contact the Contiki-NG mailing list

## Next Steps

After successful integration:

1. **Extend functionality**: Add UDP/TCP socket APIs
2. **Add 6LoWPAN**: Port compression/decompression
3. **Implement RPL**: Add routing protocol support
4. **Optimize**: Profile and optimize hot paths
5. **Document**: Add your platform-specific notes to this guide
