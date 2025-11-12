#!/bin/bash

# Simple ping test for uIP6-Rust
# This script tests IPv6 connectivity using ping6

set -e

CONTIKI_DIR="../../"
TEST_DIR="$(pwd)"

echo "========================================"
echo "  uIP6-Rust Ping Test"
echo "========================================"
echo ""

# Build and run the application in background
echo "Building application..."
make TARGET=native clean > /dev/null 2>&1
make TARGET=native > /dev/null 2>&1

echo "Starting application in background..."
sudo ./uip6-rust-test.native &
APP_PID=$!

# Wait for application to start
echo "Waiting for application to initialize..."
sleep 2

# Get the tap interface
TAP_IF=$(ip link show | grep tap | cut -d: -f2 | tr -d ' ' | head -1)

if [ -z "$TAP_IF" ]; then
    echo "✗ No tap interface found"
    kill $APP_PID 2>/dev/null || true
    exit 1
fi

echo "Using interface: $TAP_IF"

# Try to ping the link-local address
echo ""
echo "Testing ICMPv6 echo (ping)..."
LINK_LOCAL="fe80::200:0:0:1"

if ping6 -c 3 -W 1 ${LINK_LOCAL}%${TAP_IF} > /dev/null 2>&1; then
    echo "✓ Ping successful to $LINK_LOCAL"
    RESULT=0
else
    echo "✗ Ping failed to $LINK_LOCAL"
    RESULT=1
fi

# Cleanup
echo ""
echo "Cleaning up..."
kill $APP_PID 2>/dev/null || true
wait $APP_PID 2>/dev/null || true

if [ $RESULT -eq 0 ]; then
    echo "✓ PING TEST PASSED"
else
    echo "✗ PING TEST FAILED"
fi

exit $RESULT
