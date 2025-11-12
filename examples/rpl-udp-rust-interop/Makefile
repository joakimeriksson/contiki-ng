CONTIKI_PROJECT = udp-client udp-server
all: $(CONTIKI_PROJECT)

CONTIKI = ../..

# Use RPL-Lite routing
MAKE_ROUTING = MAKE_ROUTING_RPL_LITE

# Use CSMA MAC (simpler, more compatible)
MAKE_MAC = MAKE_MAC_CSMA

# Include Rust stack support
include $(CONTIKI)/os/net/ipv6-rust/Makefile.uip6-rust

# Include Contiki-NG build system
include $(CONTIKI)/Makefile.include
