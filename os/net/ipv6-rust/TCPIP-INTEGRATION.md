# TCP/IP Layer Integration for Rust IPv6 Stack

This patch integrates the Rust IPv6 stack directly into the Contiki-NG TCP/IP layer (`os/net/ipv6/tcpip.c`), making packet processing automatic and transparent to applications.

## What This Does

Instead of requiring applications to manually call Rust functions, this patch modifies the core `packet_input()` function to:

1. **When `UIP6_RUST_ENABLE=1`**: Process packets with the Rust stack
2. **When `UIP6_RUST_HYBRID_MODE=1`**: Try Rust first, fall back to C if needed
3. **When disabled**: Use standard C stack (no changes)

## Benefits

- ✅ **Automatic**: No application changes needed
- ✅ **Transparent**: Works with existing UDP/TCP/RPL code
- ✅ **Safe fallback**: Hybrid mode provides safety net
- ✅ **Zero overhead**: Conditional compilation removes code when disabled

## How to Apply

### Option 1: Manual Patch

```bash
cd /path/to/contiki-ng
patch -p1 < os/net/ipv6-rust/tcpip-rust-integration.patch
```

### Option 2: Manual Edit

Edit `os/net/ipv6/tcpip.c`:

1. **Add includes** after line 48:
```c
/* Include Rust IPv6 stack integration if enabled */
#if UIP6_RUST_ENABLE
#include "uip6-rust-conf.h"
#include "uip6-rust-glue.h"
#endif
```

2. **Replace packet processing** at line 186:

Replace:
```c
    uip_input();
```

With:
```c
#if UIP6_RUST_ENABLE
    /* Process packet with Rust IPv6 stack */
    int rust_result = uip6_rust_glue_input(uip_buf, uip_len);

#if UIP6_RUST_HYBRID_MODE
    /* In hybrid mode, fall back to C if Rust fails */
    if(rust_result < 0) {
      LOG_DBG("Rust stack failed, using C fallback\n");
      uip_input();
    }
#endif /* UIP6_RUST_HYBRID_MODE */

#else /* !UIP6_RUST_ENABLE */
    /* Use standard C IPv6 stack */
    uip_input();
#endif /* UIP6_RUST_ENABLE */
```

## Testing

After applying the patch:

```bash
# Rebuild with Rust stack
cd examples/rpl-udp-rust-interop
make clean
make UIP6_RUST_CONF_ENABLE=1 udp-server.native TARGET=native

# The server will now use Rust stack automatically
# No code changes needed in udp-server.c!
```

## Impact

**Before patch**: Applications must manually call Rust functions
```c
// Application code
if(UIP6_RUST_ENABLE) {
  uip6_rust_glue_input(buf, len);
} else {
  uip_input();
}
```

**After patch**: Automatic based on configuration
```c
// No changes needed - handled by tcpip.c
tcpip_input(); // Uses Rust or C based on config
```

## Backwards Compatibility

- ✅ No impact when `UIP6_RUST_ENABLE=0` (default)
- ✅ Works with all existing applications
- ✅ Can be enabled per-project via project-conf.h
- ✅ Hybrid mode provides safety during migration

## Performance

- **Zero overhead** when disabled (conditional compilation)
- **Single function call** overhead when enabled
- **Same performance** as direct Rust calls

## Configuration

Enable in `project-conf.h`:
```c
/* Use Rust stack at TCP/IP layer */
#define UIP6_RUST_CONF_ENABLE 1

/* Optional: Enable hybrid mode with C fallback */
#define UIP6_RUST_CONF_HYBRID_MODE 1
```

Or via Makefile:
```makefile
CFLAGS += -DUIP6_RUST_CONF_ENABLE=1
```

## Logging

When enabled, you'll see:
```
TCP/IP: Using Rust IPv6 stack for packet processing
```

In hybrid mode with fallback:
```
TCP/IP: Rust stack failed, using C fallback
```

## Future Enhancements

This patch provides the foundation for deeper integration:
- [ ] UDP socket creation through Rust
- [ ] TCP connection handling in Rust
- [ ] ICMPv6 directly in Rust
- [ ] ND6 fully in Rust

## See Also

- [Configuration Guide](CONFIGURATION.md)
- [Integration Guide](INTEGRATION.md)
- [RPL-UDP Interop Example](../../examples/rpl-udp-rust-interop/)
