#!/bin/bash

# Test script for uIP6-Rust on native platform
# This script builds and runs the test on the native (Linux) platform

set -e

CONTIKI_DIR="../../"
TEST_DIR="$(pwd)"

echo "========================================"
echo "  uIP6-Rust Native Platform Test"
echo "========================================"
echo ""

# Build the Rust library
echo "Step 1: Building Rust IPv6 library..."
cd "$CONTIKI_DIR/os/net/ipv6-rust"
cargo build --release
cd "$TEST_DIR"
echo "✓ Rust library built successfully"
echo ""

# Build the test application
echo "Step 2: Building test application for native platform..."
make TARGET=native clean
make TARGET=native
echo "✓ Test application built successfully"
echo ""

# Run the test
echo "Step 3: Running tests..."
echo "========================================"
echo ""

# Run with timeout
timeout 30s ./uip6-rust-test.native > test_output.log 2>&1 || true

# Display output
cat test_output.log

# Check results
echo ""
echo "========================================"
echo "Analyzing results..."
echo ""

PASSED=$(grep -c "✓ PASS:" test_output.log || true)
FAILED=$(grep -c "✗ FAIL:" test_output.log || true)
TOTAL=$((PASSED + FAILED))

echo "Total tests:  $TOTAL"
echo "Passed:       $PASSED"
echo "Failed:       $FAILED"

if [ $FAILED -eq 0 ] && [ $PASSED -gt 0 ]; then
    echo ""
    echo "✓ ALL TESTS PASSED"
    exit 0
else
    echo ""
    echo "✗ SOME TESTS FAILED"
    exit 1
fi
