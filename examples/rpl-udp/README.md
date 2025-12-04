# RPL-UDP Interactive Example

An interactive RPL/UDP demonstration with button events, LED feedback, sensor data, and RSSI monitoring for Contiki-NG.

## Features

- Button press detection and notification between devices
- JSON-based messaging protocol
- Periodic keepalive messages with sensor data
- Bidirectional RSSI (signal strength) reporting
- LED feedback for visual status
- Shell support for debugging

## Hardware Requirements

You need at least two devices:

| Role | Device | Notes |
|------|--------|-------|
| Server (DAG root) | CC1352R LaunchPad or LPSTK | Receives data, sends ACKs |
| Client | CC1352R LaunchPad or LPSTK | Sends keepalives and button events |

The LPSTK SensorTag includes HDC-1000 (temperature/humidity) sensor for richer keepalive data.

## Building and Flashing

### Server (DAG Root)

```bash
# Build
make TARGET=simplelink BOARD=launchpad/cc1352r1 udp-server

# Flash
make TARGET=simplelink BOARD=launchpad/cc1352r1 udp-server.upload PORT=/dev/tty.usbmodemXXXX
```

### Client

```bash
# Build
make TARGET=simplelink BOARD=launchpad/cc1352r1 udp-client

# Flash
make TARGET=simplelink BOARD=launchpad/cc1352r1 udp-client.upload PORT=/dev/tty.usbmodemXXXX
```

### Serial Console

Connect to view logs and use shell commands:

```bash
make TARGET=simplelink BOARD=launchpad/cc1352r1 login PORT=/dev/tty.usbmodemXXXX
```

Useful shell commands: `ip-addr`, `rpl-status`, `routes`

## Testing

1. Flash `udp-server` to one device (this becomes the DAG root)
2. Flash `udp-client` to another device
3. Power both devices - client will join the RPL network
4. Observe keepalive messages every 10 seconds
5. Press button on client - server receives button event
6. Press button on server - client receives button event

## Protocol

Compact JSON messages optimized for constrained devices:

### Message Types

| Type | Code | Example |
|------|------|---------|
| Keepalive | `k` | `{"t":"k","s":5,"r":-70,"tmp":2350,"hum":6520,"bat":3300}` |
| Button | `b` | `{"t":"b","s":3,"b":0}` |
| Ack | `a` | `{"t":"a","s":3,"r":-65}` |

### Fields

| Field | Description |
|-------|-------------|
| `t` | Message type (`k`, `b`, `a`) |
| `s` | Sequence number |
| `b` | Button ID (button messages only) |
| `r` | RSSI in dBm |
| `tmp` | Temperature in C * 100 (e.g., 2350 = 23.50C) |
| `hum` | Humidity in % * 100 (e.g., 6520 = 65.20%) |
| `bat` | Battery voltage in mV |

### RSSI Monitoring

- Client keepalives include RSSI of last ACK received from server
- Server ACKs include RSSI of received message
- This provides bidirectional link quality monitoring

## LED Behavior

| Event | LED Action |
|-------|------------|
| Button pressed locally | LED2 ON |
| Button released | LED2 OFF |
| Message received | LED1 flash |
| Remote button event | LED2 toggle |

## Configuration

Optional settings in `project-conf.h`:

```c
#define KEEPALIVE_INTERVAL (10 * CLOCK_SECOND)
#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678
```

## Implementation Notes

### Reading RSSI

```c
#include "net/ipv6/uipbuf.h"

int16_t rssi = (int16_t)uipbuf_get_attr(UIPBUF_ATTR_RSSI);
```

### Reading Sensors (LPSTK)

```c
#include "board-peripherals.h"
#include "batmon-sensor.h"

SENSORS_ACTIVATE(hdc_1000_sensor);
int temp = hdc_1000_sensor.value(HDC_1000_SENSOR_TYPE_TEMP);  // C * 100
int hum = hdc_1000_sensor.value(HDC_1000_SENSOR_TYPE_HUMID); // % * 100

int bat_raw = batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT);
int bat_mv = (bat_raw * 125) >> 5;  // Convert to mV
```

### Button Handling

```c
#include "dev/button-hal.h"

PROCESS_WAIT_EVENT();
if(ev == button_hal_press_event) {
  button_hal_button_t *btn = (button_hal_button_t *)data;
  // Send button message
}
```

## Files

| File | Description |
|------|-------------|
| `udp-client.c` | Client implementation |
| `udp-server.c` | Server/DAG root implementation |
| `Makefile` | Build configuration (includes shell module) |
| `project-conf.h` | Optional configuration overrides |
