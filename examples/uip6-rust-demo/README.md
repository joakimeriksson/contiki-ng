# uIP6-Rust Demo Application

This demo application shows how to use the configuration system to switch between the C and Rust IPv6 stacks.

## Overview

The demo demonstrates:
- Configuration-based stack selection
- Rust stack initialization
- Address management
- Hybrid mode operation
- Status monitoring

## Building

### Method 1: Using Makefile Variable (Recommended)

```bash
# Build with Rust stack
make UIP6_RUST_CONF_ENABLE=1 TARGET=native

# Build with C stack
make UIP6_RUST_CONF_ENABLE=0 TARGET=native

# or simply:
make TARGET=native  # Uses default from Makefile
```

### Method 2: Using project-conf.h

Edit `project-conf.h` and uncomment:

```c
#define UIP6_RUST_CONF_ENABLE 1
```

Then build:
```bash
make TARGET=native
```

### Method 3: Using Compiler Flag

```bash
make CFLAGS+=-DUIP6_RUST_CONF_ENABLE=1 TARGET=native
```

## Running

### Native Platform

```bash
# Build
make UIP6_RUST_CONF_ENABLE=1 TARGET=native

# Run
sudo ./build/native/uip6-rust-demo.native
```

Expected output with Rust enabled:
```
**********************************
  uIP6-Rust Demo Application
**********************************

=================================
  IPv6 Stack Configuration
=================================
Stack: Rust-based IPv6
Mode: Rust only
Logging: Disabled
Rust IPv6 Stack Status:
  Version: uIP6-Rust v0.1.0
  Initialized: yes
  Mode: Rust only
=================================

Initializing Rust IPv6 stack...
Adding link-local address: fe80::200:0:0:1
✓ Address added successfully
Adding global address: 2001:db8::1
✓ Address added successfully

Demo application running.
The application will print status every 10 seconds.
```

Expected output with C stack:
```
**********************************
  uIP6-Rust Demo Application
**********************************

=================================
  IPv6 Stack Configuration
=================================
Stack: C-based IPv6 (uip6)
Mode: Standard C
=================================

Using standard C IPv6 stack
✓ Link-local address added
✓ Global address added

Demo application running.
```

## Configuration Options

### Basic Options

| Option | Values | Description |
|--------|--------|-------------|
| `UIP6_RUST_CONF_ENABLE` | 0 or 1 | Enable Rust stack |
| `UIP6_RUST_CONF_HYBRID_MODE` | 0 or 1 | Run both C and Rust |
| `UIP6_RUST_CONF_LOG_LEVEL` | LOG_LEVEL_* | Rust logging level |

### Testing Different Modes

**Rust Only**:
```bash
make UIP6_RUST_CONF_ENABLE=1 TARGET=native clean
make UIP6_RUST_CONF_ENABLE=1 TARGET=native
```

**C Only**:
```bash
make UIP6_RUST_CONF_ENABLE=0 TARGET=native clean
make UIP6_RUST_CONF_ENABLE=0 TARGET=native
```

**Hybrid Mode** (edit project-conf.h):
```c
#define UIP6_RUST_CONF_ENABLE 1
#define UIP6_RUST_CONF_HYBRID_MODE 1
```

## Testing Connectivity

### Test IPv6 Ping

After starting the application:

```bash
# In another terminal
ping6 fe80::200:0:0:1%tap0
ping6 2001:db8::1%tap0
```

### Monitor Packets

```bash
# Watch IPv6 traffic
sudo tcpdump -i tap0 -n ip6
```

## Platform-Specific Builds

### Zoul Platform

```bash
make UIP6_RUST_CONF_ENABLE=1 TARGET=zoul
```

### NRF52840

```bash
make UIP6_RUST_CONF_ENABLE=1 TARGET=nrf52840
```

### CC2538

```bash
make UIP6_RUST_CONF_ENABLE=1 TARGET=cc2538dk
```

## Comparing C vs Rust

To compare performance and behavior:

1. Build with Rust:
   ```bash
   make UIP6_RUST_CONF_ENABLE=1 TARGET=native
   cp build/native/uip6-rust-demo.native demo-rust
   ```

2. Build with C:
   ```bash
   make clean
   make UIP6_RUST_CONF_ENABLE=0 TARGET=native
   cp build/native/uip6-rust-demo.native demo-c
   ```

3. Run and compare:
   ```bash
   ./demo-rust
   # vs
   ./demo-c
   ```

## Code Size Comparison

```bash
# Rust version
make UIP6_RUST_CONF_ENABLE=1 TARGET=native
size build/native/uip6-rust-demo.native

# C version
make clean
make UIP6_RUST_CONF_ENABLE=0 TARGET=native
size build/native/uip6-rust-demo.native
```

## Troubleshooting

### Issue: Rust stack not initializing

Check that:
1. `UIP6_RUST_CONF_ENABLE` is set to 1
2. Rust library was built successfully
3. Glue layer is included in build

### Issue: Addresses not showing up

In hybrid mode, use:
```c
uip6_rust_glue_sync_addresses();
```

### Issue: Configuration not taking effect

Always clean before rebuilding after changing configuration:
```bash
make clean
```

## Next Steps

- Try enabling hybrid mode for testing
- Add custom packet processing
- Integrate with your application
- Test on embedded hardware
- Measure performance differences

## See Also

- [../../os/net/ipv6-rust/CONFIGURATION.md](../../os/net/ipv6-rust/CONFIGURATION.md) - Complete configuration guide
- [../../os/net/ipv6-rust/README.md](../../os/net/ipv6-rust/README.md) - Feature documentation
- [../../tests/uip6-rust-test/](../../tests/uip6-rust-test/) - Test suite
