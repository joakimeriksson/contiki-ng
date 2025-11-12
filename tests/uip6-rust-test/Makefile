# Makefile for uIP6-Rust test application

CONTIKI_PROJECT = uip6-rust-test
all: $(CONTIKI_PROJECT)

# Contiki-NG configuration
CONTIKI = ../..

# Build the Rust library first
RUST_LIB = $(CONTIKI)/os/net/ipv6-rust/target/release/libuip6_rust.a

.PHONY: rust-build
rust-build:
	@echo "Building Rust IPv6 library..."
	@cd $(CONTIKI)/os/net/ipv6-rust && cargo build --release

# Include Rust IPv6 stack headers
CFLAGS += -I$(CONTIKI)/os/net/ipv6-rust

# Link with Rust library
TARGET_LIBFILES += $(RUST_LIB)

# Make sure Rust is built first
$(CONTIKI_PROJECT): rust-build

include $(CONTIKI)/Makefile.include

rust-build: $(RUST_LIB)

rust-clean:
	cd $(CONTIKI)/os/net/ipv6-rust && cargo clean
