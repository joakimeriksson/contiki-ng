CONTIKI_PROJECT = udp-client udp-server
all: $(CONTIKI_PROJECT)

CONTIKI = ../..

# Use RPL-Lite routing
MAKE_ROUTING = MAKE_ROUTING_RPL_LITE

# Use CSMA MAC (simpler, more compatible)
MAKE_MAC = MAKE_MAC_CSMA

# Check if UIP6_RUST_CONF_ENABLE is set (from command line or environment)
ifdef UIP6_RUST_CONF_ENABLE
  # Pass it as a compiler flag
  CFLAGS += -DUIP6_RUST_CONF_ENABLE=$(UIP6_RUST_CONF_ENABLE)
endif

# Include Rust stack support
include $(CONTIKI)/os/net/ipv6-rust/Makefile.uip6-rust

# Include Contiki-NG build system
include $(CONTIKI)/Makefile.include
