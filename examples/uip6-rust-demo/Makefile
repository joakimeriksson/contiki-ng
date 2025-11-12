CONTIKI_PROJECT = uip6-rust-demo
all: $(CONTIKI_PROJECT)

# Set to 1 to enable Rust IPv6 stack
# Set to 0 to use C IPv6 stack
UIP6_RUST_CONF_ENABLE ?= 1

# Contiki-NG root directory
CONTIKI = ../..

# Include Rust IPv6 stack if enabled
# This will automatically handle building and linking
include $(CONTIKI)/os/net/ipv6-rust/Makefile.uip6-rust

# Include Contiki-NG build system
include $(CONTIKI)/Makefile.include
