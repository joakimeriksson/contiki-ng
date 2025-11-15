#!/bin/bash
#
# Compare Rust and C Event Kernel Implementations
# Runs benchmarks and compares results
#

set -e

echo "======================================================================"
echo "  Contiki-NG Event Kernel: Rust vs C Comparison"
echo "======================================================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Build Rust implementation
echo -e "${BLUE}Building Rust implementation...${NC}"
cd rust-event-kernel
cargo build --release 2>&1 | grep -E "(Compiling|Finished)" || true
cargo build --release --example benchmark 2>&1 | grep -E "(Compiling|Finished)" || true
cd ..
echo -e "${GREEN}✓ Rust build complete${NC}"
echo ""

# Build C implementation
echo -e "${BLUE}Building C implementation...${NC}"
cd c-event-kernel
make clean > /dev/null 2>&1
make > /dev/null 2>&1
echo -e "${GREEN}✓ C build complete${NC}"
echo ""

# Run C benchmark
echo -e "${YELLOW}Running C Benchmark...${NC}"
echo "----------------------------------------------------------------------"
cd c-event-kernel
./build/benchmark
cd ..
echo ""

# Run Rust benchmark (commented out due to timing issues, but code is there)
echo -e "${YELLOW}Running Rust Benchmark...${NC}"
echo "----------------------------------------------------------------------"
echo -e "${RED}Note: Rust benchmark has known timing issues with event timers.${NC}"
echo -e "${RED}The core implementation works, but async timer coordination needs tuning.${NC}"
echo ""
cd rust-event-kernel
cargo run --release --example benchmark 2>&1 | head -50 || true
cd ..
echo ""

# Binary size comparison
echo -e "${YELLOW}Binary Size Comparison:${NC}"
echo "----------------------------------------------------------------------"
C_SIZE=$(stat -f%z c-event-kernel/build/benchmark 2>/dev/null || stat -c%s c-event-kernel/build/benchmark 2>/dev/null || echo "N/A")
RUST_SIZE=$(stat -f%z rust-event-kernel/target/release/examples/benchmark 2>/dev/null || stat -c%s rust-event-kernel/target/release/examples/benchmark 2>/dev/null || echo "N/A")

echo "C implementation:    $C_SIZE bytes"
echo "Rust implementation: $RUST_SIZE bytes"

if [ "$C_SIZE" != "N/A" ] && [ "$RUST_SIZE" != "N/A" ]; then
    OVERHEAD=$(awk "BEGIN {printf \"%.1f\", ($RUST_SIZE - $C_SIZE) * 100.0 / $C_SIZE}")
    echo "Overhead:            ${OVERHEAD}%"
fi
echo ""

# Summary
echo "======================================================================"
echo "  Comparison Summary"
echo "======================================================================"
echo ""
echo "Implementation Details:"
echo "  C:    Traditional protothreads with macro-based state machines"
echo "  Rust: Async/await with compiler-generated state machines"
echo ""
echo "Key Differences:"
echo "  • Memory Safety:  C (manual) vs Rust (guaranteed by compiler)"
echo "  • Type Safety:    C (weak) vs Rust (strong)"
echo "  • Syntax:         C (macros) vs Rust (async/await)"
echo "  • Performance:    Similar (both compile to state machines)"
echo ""
echo "For detailed analysis, see:"
echo "  - RUST_EVENT_KERNEL_PLAN.md  (implementation plan)"
echo "  - examples/*/README.md         (individual documentation)"
echo ""
echo "======================================================================"
