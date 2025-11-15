# Contiki-NG Event Kernel in C (Simplified)

A simplified, standalone implementation of Contiki-NG's event-driven kernel for benchmarking purposes.

## Overview

This is a minimal C implementation of Contiki-NG's core concepts:
- Protothreads using local continuations (switch-statement trick)
- Process system with event queue
- Simple timing abstraction (POSIX-based for native platforms)

**Purpose**: This implementation serves as a baseline for comparing the Rust implementation.
It's intentionally simplified to focus on core concepts without the full Contiki-NG complexity.

## Architecture

```
┌─────────────────────────────────────────┐
│      Application Processes              │
│   (PROCESS_THREAD macros)               │
└──────────────┬──────────────────────────┘
               │
┌──────────────▼──────────────────────────┐
│      Process System (process.c)         │
│  • Event queue (fixed 32 entries)       │
│  • Process list                         │
│  • Poll handler                         │
└──────────────┬──────────────────────────┘
               │
┌──────────────▼──────────────────────────┐
│      Protothreads (pt.h)                │
│  • Local continuations (lc.h)           │
│  • Switch-based state resumption        │
└──────────────┬──────────────────────────┘
               │
┌──────────────▼──────────────────────────┐
│      Clock (clock.h/c)                  │
│  • POSIX monotonic clock                │
│  • Millisecond resolution               │
└─────────────────────────────────────────┘
```

## Building

### Requirements
- GCC or Clang
- POSIX-compliant system (Linux, macOS, BSD)

### Build
```bash
make
```

### Run Benchmark
```bash
make benchmark
```

### Clean
```bash
make clean
```

## Usage Example

```c
#include "src/process.h"
#include "src/clock.h"

PROCESS(my_process, "my_process");

PROCESS_THREAD(my_process, ev, data) {
    PROCESS_BEGIN();

    while(1) {
        printf("Working...\\n");

        static clock_time_t start;
        start = clock_time();

        PROCESS_WAIT_EVENT();

        // Do work
    }

    PROCESS_END();
}

int main(void) {
    clock_init();
    process_init();

    process_start(&my_process, NULL);

    while(1) {
        process_run();
    }

    return 0;
}
```

## API Overview

### Process Macros
```c
// Declare a process
PROCESS(name, "string_name");

// Process thread implementation
PROCESS_THREAD(name, ev, data) {
    PROCESS_BEGIN();

    // Process code here
    PROCESS_WAIT_EVENT();  // Wait for any event
    PROCESS_YIELD();       // Yield to scheduler
    PROCESS_EXIT();        // Exit process

    PROCESS_END();
}
```

### Process Functions
```c
void process_init(void);
void process_start(struct process *p, process_data_t data);
int process_post(struct process *p, process_event_t ev, process_data_t data);
void process_poll(struct process *p);
unsigned char process_run(void);
```

### Protothread Macros
```c
PT_INIT(pt);                      // Initialize
PT_BEGIN(pt);                     // Begin protothread
PT_END(pt);                       // End protothread
PT_WAIT_UNTIL(pt, condition);     // Wait for condition
PT_YIELD(pt);                     // Yield execution
PT_EXIT(pt);                      // Exit protothread
```

## Performance

### Memory Footprint
- `struct pt`: 2 bytes (just a line counter)
- `struct process`: 32 bytes
- `process_event_t`: 1 byte
- `clock_time_t`: 4 bytes
- Event queue: ~384 bytes (32 × 12 bytes)

### Benchmark Results (Example)
```
Events posted:     10000
Post rate:         ~670M events/sec
Process rate:      Variable (depends on work)
```

**Note**: Post rate is artificially high because it's just memory operations.
The real metric is end-to-end latency which includes processing time.

## Key Characteristics

### Advantages
- **Small Code Size**: Minimal overhead
- **Well-Understood**: Decades of use in embedded systems
- **Portable**: Works on any C99+ compiler
- **Fast Compilation**: Compiles in milliseconds

### Limitations
- **No Safety**: Manual memory management, easy to introduce bugs
- **Macro Magic**: PT macros hide control flow, debugging is difficult
- **Type Weak**: No enforcement of time units, event types, etc.
- **Limited Locals**: Local variables not preserved across PT_WAIT_*

## Implementation Notes

### Protothread Trick
The core trick uses Duff's device to implement stackless coroutines:

```c
#define PT_BEGIN(pt) { \
    unsigned short _pt_line = (pt)->lc; \
    switch(_pt_line) { case 0:

#define PT_WAIT_UNTIL(pt, condition) \
    do { \
        (pt)->lc = __LINE__; case __LINE__: \
        if(!(condition)) return PT_WAITING; \
    } while(0)

#define PT_END(pt) } \
    _pt_line = 0; (pt)->lc = 0; \
    return PT_ENDED; }
```

This generates code like:
```c
switch(lc) {
    case 0:  // Start of function
    case 42: // Resume after line 42
        if(condition) ...
    case 57: // Resume after line 57
        ...
}
```

### Event Queue
Simple circular buffer:
```c
static struct event_data events[32];
static unsigned char nevents, fevent;  // Count and front index
```

## Comparison with Rust Implementation

| Aspect | C | Rust |
|--------|---|------|
| Syntax | Macros (PT_WAIT_*) | async/await |
| Safety | Manual | Compiler-enforced |
| Binary Size | Baseline | +10-15% |
| Performance | Fast | Comparable |
| Debugging | Difficult (macros) | Better (async trace) |
| Type Safety | Weak | Strong |

## License

BSD-3-Clause

## References

- [Original Protothreads](http://dunkels.com/adam/pt/)
- [Contiki-NG Documentation](https://docs.contiki-ng.org/)
- [Duff's Device](https://en.wikipedia.org/wiki/Duff%27s_device)
