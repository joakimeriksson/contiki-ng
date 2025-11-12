# Configuration Guide for uIP6-Rust

This guide explains how to configure your Contiki-NG application to use the Rust-based IPv6 stack.

## Quick Start

### Method 1: Using project-conf.h (Recommended)

Add to your `project-conf.h`:

```c
/* Enable Rust IPv6 stack */
#define UIP6_RUST_CONF_ENABLE 1
```

Then include the Makefile fragment in your application's `Makefile`:

```makefile
# Include Rust IPv6 stack
include $(CONTIKI)/os/net/ipv6-rust/Makefile.uip6-rust
```

That's it! The build system will automatically:
- Build the Rust library
- Include necessary headers
- Link with the Rust library
- Add glue layer code

### Method 2: Using Compiler Flags

In your `Makefile`:

```makefile
# Enable Rust via compiler flag
CFLAGS += -DUIP6_RUST_CONF_ENABLE=1

# Include Rust stack
include $(CONTIKI)/os/net/ipv6-rust/Makefile.uip6-rust
```

## Configuration Options

### UIP6_RUST_CONF_ENABLE

**Type**: Boolean (0 or 1)
**Default**: 0 (disabled)
**Description**: Master switch to enable/disable Rust IPv6 stack

```c
/* Enable Rust stack */
#define UIP6_RUST_CONF_ENABLE 1

/* Disable Rust stack (use C) */
#define UIP6_RUST_CONF_ENABLE 0
```

### UIP6_RUST_CONF_HYBRID_MODE

**Type**: Boolean (0 or 1)
**Default**: 0 (disabled)
**Requires**: `UIP6_RUST_CONF_ENABLE 1`
**Description**: Run both C and Rust stacks in parallel

```c
/* Enable hybrid mode */
#define UIP6_RUST_CONF_ENABLE 1
#define UIP6_RUST_CONF_HYBRID_MODE 1
```

In hybrid mode:
- Packets are processed by Rust first
- If Rust fails, falls back to C stack
- Useful for testing and gradual migration
- Both stacks maintain separate state

### UIP6_RUST_CONF_LOG_LEVEL

**Type**: Log level constant
**Default**: `LOG_LEVEL_NONE`
**Description**: Control logging verbosity from Rust stack

```c
/* Enable debug logging */
#define UIP6_RUST_CONF_LOG_LEVEL LOG_LEVEL_DBG

/* Enable info logging */
#define UIP6_RUST_CONF_LOG_LEVEL LOG_LEVEL_INFO

/* Disable logging */
#define UIP6_RUST_CONF_LOG_LEVEL LOG_LEVEL_NONE
```

## Usage Modes

### Mode 1: Rust Only (Full Replacement)

Replace the C IPv6 stack completely with Rust.

**project-conf.h**:
```c
#define UIP6_RUST_CONF_ENABLE 1
#define UIP_CONF_IPV6 1
#define UIP_CONF_BUFFER_SIZE 1280
```

**Pros**:
- Smallest code size
- Best performance
- Full memory safety

**Cons**:
- No fallback to C
- Must be fully tested

### Mode 2: Hybrid Mode (Testing/Migration)

Run both stacks side-by-side.

**project-conf.h**:
```c
#define UIP6_RUST_CONF_ENABLE 1
#define UIP6_RUST_CONF_HYBRID_MODE 1
#define UIP6_RUST_CONF_LOG_LEVEL LOG_LEVEL_DBG
```

**Pros**:
- Safe gradual migration
- Can compare C vs Rust behavior
- Automatic fallback

**Cons**:
- Larger code size
- Higher memory usage
- Duplicate state

### Mode 3: Platform-Specific Selection

Use Rust on some platforms, C on others.

**project-conf.h**:
```c
/* Use Rust on native, C on embedded */
#ifdef CONTIKI_TARGET_NATIVE
  #define UIP6_RUST_CONF_ENABLE 1
#else
  #define UIP6_RUST_CONF_ENABLE 0
#endif
```

## Application Integration

### Initialization

When using the glue layer, initialize the Rust stack in your application:

```c
#include "uip6-rust-glue.h"

PROCESS_THREAD(my_process, ev, data)
{
  PROCESS_BEGIN();

  /* Initialize Rust IPv6 stack */
  uip6_rust_glue_init();

  /* Rest of your code */

  PROCESS_END();
}
```

### Processing Packets

The glue layer can intercept packet processing:

```c
/* In your packet input callback */
void packet_input(uint8_t *buf, uint16_t len)
{
#if UIP6_RUST_CONF_ENABLE
  /* Process with Rust (may fall back to C in hybrid mode) */
  uip6_rust_glue_input(buf, len);
#else
  /* Process with C stack */
  memcpy(uip_buf, buf, len);
  uip_len = len;
  tcpip_input();
#endif
}
```

### Address Management

Addresses can be synchronized between stacks:

```c
/* Sync addresses from C to Rust */
uip6_rust_glue_sync_addresses();

/* Or use hooks for automatic sync */
uip_ipaddr_t addr;
uip_ip6addr(&addr, 0xfe80, 0, 0, 0, 0, 0, 0, 0x1);

/* Add to C stack */
uip_ds6_addr_add(&addr, 0, ADDR_MANUAL);

/* Hook automatically adds to Rust if enabled */
uip6_rust_glue_addr_add_hook(&addr, 2);
```

## Build Examples

### Example 1: Simple Application

**Makefile**:
```makefile
CONTIKI_PROJECT = my-app
all: $(CONTIKI_PROJECT)

CONTIKI = ../..
include $(CONTIKI)/os/net/ipv6-rust/Makefile.uip6-rust
include $(CONTIKI)/Makefile.include
```

**project-conf.h**:
```c
#define UIP6_RUST_CONF_ENABLE 1
```

### Example 2: Testing Application

**Makefile**:
```makefile
CONTIKI_PROJECT = test-app
all: $(CONTIKI_PROJECT)

# Enable Rust via Makefile
CFLAGS += -DUIP6_RUST_CONF_ENABLE=1
CFLAGS += -DUIP6_RUST_CONF_HYBRID_MODE=1

CONTIKI = ../..
include $(CONTIKI)/os/net/ipv6-rust/Makefile.uip6-rust
include $(CONTIKI)/Makefile.include
```

### Example 3: Conditional Build

**Makefile**:
```makefile
CONTIKI_PROJECT = my-app
all: $(CONTIKI_PROJECT)

# Allow enabling Rust via environment variable
ifdef USE_RUST
  CFLAGS += -DUIP6_RUST_CONF_ENABLE=1
endif

CONTIKI = ../..
include $(CONTIKI)/os/net/ipv6-rust/Makefile.uip6-rust
include $(CONTIKI)/Makefile.include
```

Build with:
```bash
# Use Rust
make USE_RUST=1 TARGET=native

# Use C
make TARGET=native
```

## Verification

### Check if Rust is Enabled

Add this to your application:

```c
#if UIP6_RUST_ENABLE
  printf("Using Rust IPv6 stack\n");
  uip6_rust_glue_get_stats();
#else
  printf("Using C IPv6 stack\n");
#endif
```

### Build-Time Check

The Makefile will print:
```
Building with Rust IPv6 stack
```
or
```
Building with C IPv6 stack (Rust disabled)
```

## Troubleshooting

### Issue: Rust library not found

**Error**: `cannot find -luip6_rust`

**Solution**: Build the Rust library manually:
```bash
cd os/net/ipv6-rust
cargo build --release
```

### Issue: Configuration not taking effect

**Problem**: Changed `UIP6_RUST_CONF_ENABLE` but still using C stack

**Solution**: Clean and rebuild:
```bash
make clean
make TARGET=yourplatform
```

### Issue: Duplicate symbol errors

**Problem**: Linking errors about duplicate functions

**Solution**: Make sure you're not including both C and Rust implementations directly. Use the glue layer instead.

## Performance Tuning

### Optimize for Size

In `os/net/ipv6-rust/Cargo.toml`:
```toml
[profile.release]
opt-level = "z"     # Optimize for size
lto = true          # Link-time optimization
```

### Optimize for Speed

```toml
[profile.release]
opt-level = 3       # Optimize for speed
lto = "fat"         # Aggressive LTO
```

## Migration Strategy

### Phase 1: Testing (Hybrid Mode)
```c
#define UIP6_RUST_CONF_ENABLE 1
#define UIP6_RUST_CONF_HYBRID_MODE 1
#define UIP6_RUST_CONF_LOG_LEVEL LOG_LEVEL_DBG
```
- Test both stacks in parallel
- Compare behavior
- Identify issues

### Phase 2: Validation (Rust Primary)
```c
#define UIP6_RUST_CONF_ENABLE 1
#define UIP6_RUST_CONF_HYBRID_MODE 0
#define UIP6_RUST_CONF_LOG_LEVEL LOG_LEVEL_INFO
```
- Use Rust as primary stack
- Full feature testing
- Performance validation

### Phase 3: Production (Rust Only)
```c
#define UIP6_RUST_CONF_ENABLE 1
#define UIP6_RUST_CONF_LOG_LEVEL LOG_LEVEL_NONE
```
- Remove hybrid mode
- Disable debug logging
- Deploy to production

## Best Practices

1. **Always test in hybrid mode first** before switching to Rust-only
2. **Enable logging during development** to catch issues early
3. **Synchronize addresses** when using hybrid mode
4. **Clean build** after changing configuration
5. **Test on actual hardware** before deployment
6. **Monitor memory usage** when running hybrid mode

## See Also

- [README.md](README.md) - Feature documentation
- [INTEGRATION.md](INTEGRATION.md) - Integration guide
- [example-usage.c](example-usage.c) - Example application
- [tests/uip6-rust-test/](../../tests/uip6-rust-test/) - Test suite
