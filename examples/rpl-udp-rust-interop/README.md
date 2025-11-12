# RPL-UDP Rust Interoperability Demo

This example demonstrates interoperability between the Rust-based IPv6 stack and the C-based IPv6 stack in a real RPL network scenario.

## Overview

- **Server (RPL Root)**: Uses **Rust IPv6 stack**
- **Clients (RPL Nodes)**: Use **C IPv6 stack**
- **Network**: RPL-Lite with TSCH
- **Communication**: UDP messages between clients and server

This proves that the Rust and C implementations are fully compatible and can communicate seamlessly.

## Architecture

```
┌─────────────────────────────────────────┐
│  Client Node 1 (C IPv6 Stack)          │
│  - Uses standard C uIP6                 │
│  - Sends UDP packets every 10s          │
└────────────┬────────────────────────────┘
             │ RPL + TSCH
             │
┌────────────▼────────────────────────────┐
│  Client Node 2 (C IPv6 Stack)          │
│  - Uses standard C uIP6                 │
│  - Sends UDP packets every 10s          │
└────────────┬────────────────────────────┘
             │ RPL + TSCH
             │
┌────────────▼────────────────────────────┐
│  Server (RPL Root) - RUST IPv6 STACK   │
│  - Uses Rust uIP6-Rust implementation  │
│  - Receives and replies to UDP packets │
│  - Demonstrates Rust/C interoperability │
└─────────────────────────────────────────┘
```

## Building

### Server (with Rust)

```bash
# Build server with Rust IPv6 stack
make udp-server.upload TARGET=<platform> UIP6_RUST_CONF_ENABLE=1

# Examples:
make udp-server.upload TARGET=zoul UIP6_RUST_CONF_ENABLE=1
make udp-server.upload TARGET=nrf52840 UIP6_RUST_CONF_ENABLE=1
```

### Client (with C)

```bash
# Build client with C IPv6 stack (default)
make udp-client.upload TARGET=<platform>

# Examples:
make udp-client.upload TARGET=zoul
make udp-client.upload TARGET=nrf52840
```

## Running in Cooja

### Step 1: Compile Firmware

```bash
# Compile server (Rust)
make udp-server.cooja TARGET=cooja UIP6_RUST_CONF_ENABLE=1

# Compile client (C)
make udp-client.cooja TARGET=cooja
```

### Step 2: Create Simulation

1. Open Cooja: `java -jar tools/cooja/dist/cooja.jar`
2. Create new simulation
3. Add motes:
   - **1 server mote**: Load `udp-server.cooja` (Rust)
   - **2-5 client motes**: Load `udp-client.cooja` (C)
4. Start simulation

### Expected Output

**Server (Rust) Console**:
```
======================================
  RPL-UDP Server with Rust IPv6
======================================
Using Rust IPv6 stack
Mode: Rust only
Rust IPv6 Stack Status:
  Version: uIP6-Rust v0.1.0
  Initialized: yes
======================================
Initializing as RPL DAG root...
Server ready! Waiting for clients...

[RUST] Received request #1: 'hello from C client #0' from fe80::202:2:2:2
[RUST] Sending response #0
[RUST] Received request #2: 'hello from C client #1' from fe80::203:3:3:3
[RUST] Sending response #1
[RUST] Statistics - Received: 10, Sent: 10
```

**Client (C) Console**:
```
======================================
  RPL-UDP Client with C IPv6
======================================
Using C IPv6 stack
Connecting to Rust server...
======================================

[C] Sending request 0 to Rust server fd00::201:1:1:1
[C] Received response 'hello from C client #0' from fd00::201:1:1:1 (Rust server)
[C] Sending request 1 to Rust server fd00::201:1:1:1
[C] Received response 'hello from C client #1' from fd00::201:1:1:1 (Rust server)
[C] Statistics - Tx/Rx/MissedTx: 10/10/0
```

## Running on Hardware

### Setup with Zoul Firefly

**Hardware Required**:
- 1x Zoul Firefly (server with Rust)
- 2x Zoul Firefly (clients with C)
- 1x USB programmer per device

**Flash Devices**:

```bash
# Flash server (node 1)
make udp-server.upload TARGET=zoul UIP6_RUST_CONF_ENABLE=1 MOTES=/dev/ttyUSB0

# Flash client (node 2)
make udp-client.upload TARGET=zoul MOTES=/dev/ttyUSB1

# Flash client (node 3)
make udp-client.upload TARGET=zoul MOTES=/dev/ttyUSB2
```

**Monitor Output**:

```bash
# Monitor server
make login TARGET=zoul MOTES=/dev/ttyUSB0

# Monitor clients (in separate terminals)
make login TARGET=zoul MOTES=/dev/ttyUSB1
make login TARGET=zoul MOTES=/dev/ttyUSB2
```

## Verification

### Test 1: Basic Connectivity

Verify that clients can reach the server:
- Clients should log "Sending request X to Rust server"
- Server should log "Received request #X"
- Clients should receive responses

### Test 2: Packet Exchange

Monitor packet statistics:
- Server: Should show increasing Received/Sent counts
- Clients: Should show increasing Tx/Rx counts
- MissedTx should remain low

### Test 3: RPL Network Formation

Check that RPL forms correctly:
- Server initializes as DAG root
- Clients join the DAG
- Routes are established

### Test 4: Long-term Stability

Run for extended period (hours):
- No packet loss
- No memory leaks
- No crashes

## Configuration Options

### Enable Hybrid Mode on Server

In `project-conf.h`, uncomment:
```c
#define UIP6_RUST_CONF_HYBRID_MODE 1
```

This runs both Rust and C stacks on the server for validation.

### Enable Rust Logging

```c
#define UIP6_RUST_CONF_LOG_LEVEL LOG_LEVEL_DBG
```

### Adjust Send Interval

In `project-conf.h`:
```c
#define SEND_INTERVAL (5 * CLOCK_SECOND)  // Send every 5 seconds
```

## Troubleshooting

### Issue: Clients can't reach server

**Symptoms**: "Server not reachable yet"

**Solutions**:
1. Wait for RPL network formation (30-60 seconds)
2. Check that server is running as RPL root
3. Verify TSCH is synchronized
4. Check radio power settings

### Issue: Server not receiving packets

**Symptoms**: Server doesn't log received packets

**Solutions**:
1. Verify Rust stack initialized: Look for "Rust IPv6 stack initialized"
2. Check UDP port configuration (should be 5678)
3. Enable debug logging
4. Verify buffer sizes in project-conf.h

### Issue: Build errors

**Symptoms**: Cannot compile server

**Solutions**:
```bash
# Ensure Rust library is built
cd ../../os/net/ipv6-rust
cargo build --release
cd -

# Clean and rebuild
make clean
make udp-server.cooja TARGET=cooja UIP6_RUST_CONF_ENABLE=1
```

### Issue: No responses from server

**Symptoms**: Clients send but don't receive

**Solutions**:
1. Verify `WITH_SERVER_REPLY` is enabled
2. Check server callback is registered
3. Monitor server console for "Sending response"

## Performance Comparison

To compare Rust vs C stack performance:

### Test 1: Build Both Servers

```bash
# Server with Rust
make udp-server.cooja TARGET=cooja UIP6_RUST_CONF_ENABLE=1
cp udp-server.cooja udp-server-rust.cooja

# Server with C
make clean
make udp-server.cooja TARGET=cooja UIP6_RUST_CONF_ENABLE=0
cp udp-server.cooja udp-server-c.cooja
```

### Test 2: Run Simulations

Run separate simulations with each server version and compare:
- Packet delivery ratio
- Response latency
- Memory usage
- CPU usage

## Code Size Comparison

```bash
# Rust server
make udp-server.cooja TARGET=cooja UIP6_RUST_CONF_ENABLE=1
size udp-server.cooja

# C server
make clean
make udp-server.cooja TARGET=cooja UIP6_RUST_CONF_ENABLE=0
size udp-server.cooja
```

## Extending the Example

### Add More Complex Messages

Modify the client to send structured data:
```c
struct sensor_data {
  uint32_t seq;
  int16_t temperature;
  int16_t humidity;
};
```

### Add Downlink Commands

Server can send commands to clients:
```c
simple_udp_sendto(&udp_conn, "LED_ON", 6, sender_addr);
```

### Test with Multiple RPL Instances

Create multiple DAGs with different configurations.

## Next Steps

1. **Test on actual hardware**: Deploy to real sensor network
2. **Add more clients**: Scale up to 10+ nodes
3. **Measure performance**: Compare Rust vs C in production
4. **Add security**: Test with DTLS over UDP
5. **Long-term testing**: Run for days/weeks to verify stability

## References

- [Rust IPv6 Stack Documentation](../../os/net/ipv6-rust/README.md)
- [Configuration Guide](../../os/net/ipv6-rust/CONFIGURATION.md)
- [Test Suite](../../tests/uip6-rust-test/)
- [Original RPL-UDP Example](../rpl-udp/)
