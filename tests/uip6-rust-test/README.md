# uIP6-Rust Test Suite

This directory contains automated tests for the Rust-based IPv6 stack implementation.

## Overview

The test suite verifies the following functionality:
- Stack initialization
- Address management (add, lookup, remove)
- Neighbor cache management
- Address type detection (multicast, link-local)
- Packet processing (ICMPv6 Echo Request/Reply)
- Error handling (invalid packets)

## Test Files

- `uip6-rust-test.c` - Main test application
- `Makefile` - Build configuration
- `project-conf.h` - Project configuration
- `test.py` - Python test script (for Cooja)
- `run-native-test.sh` - Shell script for native platform testing
- `ping-test.sh` - ICMPv6 ping test

## Running Tests

### Quick Test (Native Platform)

```bash
cd tests/uip6-rust-test
./run-native-test.sh
```

This will:
1. Build the Rust library
2. Build the test application for native platform
3. Run the tests
4. Display results

Expected output:
```
====================================
  uIP6-Rust Test Suite
====================================

Test 1: Stack Initialization
✓ PASS: Stack initialization

Test 2: Address Management
✓ PASS: Add link-local address
✓ PASS: Lookup link-local address
...

=== Test Summary ===
Total tests:  20
Passed:       20
Failed:       0
Success rate: 100%
==================
✓ ALL TESTS PASSED
```

### Manual Build and Run

```bash
# Build the Rust library
cd ../../os/net/ipv6-rust
cargo build --release
cd -

# Build for native platform
make TARGET=native

# Run the test
./uip6-rust-test.native
```

### Build for Different Platforms

```bash
# Native (Linux/macOS)
make TARGET=native

# Zoul platform
make TARGET=zoul

# CC2538DK
make TARGET=cc2538dk

# NRF52840
make TARGET=nrf52840
```

## Test Coverage

### Test 1: Stack Initialization
- Initialize the Rust IPv6 stack
- Verify version string

### Test 2: Address Management
- Add link-local address (fe80::/10)
- Add global address (2001:db8::/32)
- Lookup addresses
- Source address selection
- Remove addresses
- Verify address removal

### Test 3: Neighbor Management
- Add neighbor with link-layer address
- Lookup neighbor
- Verify link-layer address mapping
- Add router neighbor

### Test 4: Address Type Detection
- Multicast address detection (ff00::/8)
- Link-local address detection (fe80::/10)
- Global address detection

### Test 5: Packet Processing
- Process valid ICMPv6 Echo Request
- Reject too-short packets
- Reject NULL buffer
- Reject zero-length packets

## ICMPv6 Ping Test

To test actual network connectivity:

```bash
./ping-test.sh
```

This will:
1. Start the application with a tap interface
2. Send ICMPv6 echo requests to the link-local address
3. Verify echo replies are received

## Continuous Integration

### Adding to CI Pipeline

Add to `.github/workflows/build.yml`:

```yaml
- name: Run uIP6-Rust tests
  run: |
    cd tests/uip6-rust-test
    ./run-native-test.sh
```

## Expected Test Results

All 20+ tests should pass:

| Test Category | Tests | Status |
|---------------|-------|--------|
| Initialization | 1 | ✓ PASS |
| Address Management | 7 | ✓ PASS |
| Neighbor Management | 3 | ✓ PASS |
| Address Types | 4 | ✓ PASS |
| Packet Processing | 4 | ✓ PASS |

## Debugging Failed Tests

If tests fail, check:

1. **Build issues**:
   ```bash
   cargo build --release --verbose
   ```

2. **Link issues**:
   ```bash
   make TARGET=native V=1
   ```

3. **Runtime errors**:
   ```bash
   # Enable debug logging
   LOG_LEVEL=DBG ./uip6-rust-test.native
   ```

4. **Memory issues**:
   ```bash
   # Run with valgrind
   valgrind --leak-check=full ./uip6-rust-test.native
   ```

## Adding New Tests

To add a new test:

1. Add a test function in `uip6-rust-test.c`:
   ```c
   static void test_my_feature(void)
   {
     LOG_INFO("Test N: My Feature\n");

     // Test code here
     int result = my_function();

     print_test_result("My test", result == expected);
   }
   ```

2. Call it from the main process:
   ```c
   test_my_feature();
   ```

3. Update this README with the new test description

## Performance Benchmarks

The test suite also measures:
- Stack initialization time
- Address lookup performance
- Packet processing throughput

Results are printed at the end of the test run.

## Platform-Specific Notes

### Native Platform
- Uses tap interface for networking
- Requires root/sudo for interface setup
- Best for development and quick testing

### Embedded Platforms
- Requires serial output for test results
- May need longer timeouts
- Use Cooja simulator for automated testing

## Troubleshooting

**Q: Tests fail with "cannot find -luip6_rust"**
A: Build the Rust library first:
```bash
cd ../../os/net/ipv6-rust && cargo build --release
```

**Q: Application crashes on startup**
A: Check that UIP_CONF_BUFFER_SIZE is at least 1280 bytes in project-conf.h

**Q: Ping test fails**
A: Ensure tap interface is up and has proper routing:
```bash
ip -6 route show
```

## License

Same as Contiki-NG (3-clause BSD)
