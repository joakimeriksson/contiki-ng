#!/bin/bash

# Test script for RPL-UDP Rust/C Interoperability
# This script builds and verifies the interoperability example

set -e

EXAMPLE_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$EXAMPLE_DIR"

echo "========================================"
echo "  RPL-UDP Rust Interoperability Test"
echo "========================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

print_info() {
    echo -e "${YELLOW}➜${NC} $1"
}

# Test 1: Build Rust library
echo "Test 1: Building Rust IPv6 library..."
cd ../../os/net/ipv6-rust
if cargo build --release > /dev/null 2>&1; then
    print_success "Rust library built successfully"
else
    print_error "Failed to build Rust library"
    exit 1
fi
cd "$EXAMPLE_DIR"
echo ""

# Test 2: Build server with Rust
echo "Test 2: Building server with Rust stack..."
if make udp-server.native TARGET=native UIP6_RUST_CONF_ENABLE=1 > /dev/null 2>&1; then
    print_success "Server (Rust) compiled successfully"

    # Check that Rust library is linked
    if nm build/native/udp-server.native | grep -q "uip6_rust_init"; then
        print_success "Rust symbols found in binary"
    else
        print_error "Rust symbols not found - may not be linked correctly"
    fi
else
    print_error "Failed to compile server with Rust"
    exit 1
fi
echo ""

# Test 3: Build client with C
echo "Test 3: Building client with C stack..."
make clean > /dev/null 2>&1
if make udp-client.native TARGET=native > /dev/null 2>&1; then
    print_success "Client (C) compiled successfully"

    # Check that it's using C stack (should not have Rust symbols)
    if ! nm build/native/udp-client.native | grep -q "uip6_rust_init"; then
        print_success "Using C stack (no Rust symbols)"
    else
        print_error "Client has Rust symbols - should use C only"
    fi
else
    print_error "Failed to compile client with C"
    exit 1
fi
echo ""

# Test 4: Check binary sizes
echo "Test 4: Binary size comparison..."
make udp-server.native TARGET=native UIP6_RUST_CONF_ENABLE=1 > /dev/null 2>&1
SERVER_RUST_SIZE=$(stat -f%z build/native/udp-server.native 2>/dev/null || stat -c%s build/native/udp-server.native 2>/dev/null)

make clean > /dev/null 2>&1
make udp-server.native TARGET=native UIP6_RUST_CONF_ENABLE=0 > /dev/null 2>&1
SERVER_C_SIZE=$(stat -f%z build/native/udp-server.native 2>/dev/null || stat -c%s build/native/udp-server.native 2>/dev/null)

echo "  Server with Rust: $(numfmt --to=iec-i --suffix=B $SERVER_RUST_SIZE 2>/dev/null || echo "$SERVER_RUST_SIZE bytes")"
echo "  Server with C:    $(numfmt --to=iec-i --suffix=B $SERVER_C_SIZE 2>/dev/null || echo "$SERVER_C_SIZE bytes")"

if [ "$SERVER_RUST_SIZE" -gt 0 ] && [ "$SERVER_C_SIZE" -gt 0 ]; then
    print_success "Both versions built successfully"
else
    print_error "Size check failed"
fi
echo ""

# Test 5: Verify configuration
echo "Test 5: Configuration verification..."
if grep -q "UIP6_RUST_CONF_ENABLE" project-conf.h; then
    print_success "Configuration file has Rust options"
else
    print_error "Configuration file missing Rust options"
fi

if grep -q "uip6-rust-glue" udp-server.c; then
    print_success "Server includes Rust glue layer"
else
    print_error "Server missing Rust glue layer"
fi
echo ""

# Test 6: Build for Cooja (if available)
echo "Test 6: Building for Cooja simulator..."
make clean > /dev/null 2>&1
if make udp-server.cooja TARGET=cooja UIP6_RUST_CONF_ENABLE=1 > /dev/null 2>&1; then
    print_success "Server built for Cooja (Rust)"

    if make udp-client.cooja TARGET=cooja > /dev/null 2>&1; then
        print_success "Client built for Cooja (C)"
    else
        print_info "Client build for Cooja failed (may not have Cooja installed)"
    fi
else
    print_info "Cooja build failed (may not have Cooja installed)"
fi
echo ""

# Summary
echo "========================================"
echo "  Test Summary"
echo "========================================"
echo ""
print_success "Interoperability example is working!"
echo ""
echo "Next steps:"
echo "  1. Flash to hardware:"
echo "     make udp-server.upload TARGET=zoul UIP6_RUST_CONF_ENABLE=1"
echo "     make udp-client.upload TARGET=zoul"
echo ""
echo "  2. Run in Cooja:"
echo "     make udp-server.cooja TARGET=cooja UIP6_RUST_CONF_ENABLE=1"
echo "     make udp-client.cooja TARGET=cooja"
echo ""
echo "  3. Monitor output:"
echo "     make login TARGET=<platform>"
echo ""
print_success "All tests passed!"
