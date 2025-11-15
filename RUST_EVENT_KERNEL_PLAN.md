# Contiki-NG Event Kernel in Rust: Implementation Plan

## Executive Summary

This document outlines a comprehensive plan to reimplement Contiki-NG's event kernel in Rust, including protothreads, processes, timers, and event-timers. The goal is to leverage Rust's safety guarantees while maintaining the lightweight, event-driven architecture that makes Contiki-NG suitable for IoT devices.

---

## Table of Contents

1. [Design Philosophy](#1-design-philosophy)
2. [Architecture Overview](#2-architecture-overview)
3. [Component Implementation Strategy](#3-component-implementation-strategy)
4. [Memory Safety & Performance](#4-memory-safety--performance)
5. [Implementation Phases](#5-implementation-phases)
6. [FFI & C Interoperability](#6-ffi--c-interoperability)
7. [Testing Strategy](#7-testing-strategy)
8. [Migration Path](#8-migration-path)

---

## 1. Design Philosophy

### Goals
- **Type Safety**: Leverage Rust's type system to prevent common bugs
- **Memory Safety**: Eliminate undefined behavior from protothread macros
- **Zero-Cost Abstractions**: Match or improve upon C performance
- **Async-Native**: Use Rust's async/await as a natural protothread replacement
- **Embedded-First**: No heap allocations, no standard library, static everything
- **Interoperability**: Seamless integration with existing C code

### Trade-offs
- **Learning Curve**: Team must learn Rust and embedded Rust patterns
- **Toolchain**: Requires Rust compiler for target architectures
- **Binary Size**: May be larger initially (can be optimized)
- **Debugging**: Different debugging experience vs C

---

## 2. Architecture Overview

### Component Mapping: C → Rust

```
┌──────────────────────────────────────────────────────────────┐
│                    C Implementation                          │
├──────────────────────────────────────────────────────────────┤
│ Protothreads (pt.h)          →  Rust Async/Await + Futures  │
│ Local Continuations (lc.h)   →  State Machine Generators     │
│ Process System (process.c)   →  Event Loop + Task Executor   │
│ Simple Timers (timer.c)      →  Monotonic Clock Abstraction  │
│ Event Timers (etimer.c)      →  Timer Wheel + Event Posting  │
│ Clock (clock.h)              →  Platform HAL Trait           │
└──────────────────────────────────────────────────────────────┘
```

### Rust Crate Structure

```
contiki-ng-rs/
├── Cargo.toml
├── src/
│   ├── lib.rs                      # Re-exports
│   ├── clock.rs                    # Clock HAL trait
│   ├── timer.rs                    # Simple timer implementation
│   ├── etimer.rs                   # Event timer system
│   ├── process.rs                  # Process system & event loop
│   ├── protothread.rs              # Protothread abstraction (optional)
│   └── sync.rs                     # Synchronization primitives
├── platform/
│   ├── nrf52840.rs                 # Nordic nRF52840 clock impl
│   ├── cc2538.rs                   # TI CC2538 clock impl
│   └── native.rs                   # Native/POSIX clock impl
├── examples/
│   ├── blink.rs
│   ├── periodic_timer.rs
│   └── process_communication.rs
└── tests/
    ├── timer_tests.rs
    └── process_tests.rs
```

---

## 3. Component Implementation Strategy

### 3.1 Clock Abstraction

**Design**: Platform abstraction layer using traits

```rust
// src/clock.rs

/// Platform-specific clock implementation
pub trait Clock {
    /// Initialize the clock hardware
    fn init();

    /// Get current time in ticks
    fn now() -> ClockTime;

    /// Get ticks per second (e.g., 32 for 32 Hz)
    const TICKS_PER_SECOND: u32;

    /// Busy-wait for specified ticks
    fn wait(ticks: ClockTime);

    /// Microsecond-precision delay
    fn delay_usec(us: u16);

    /// Get seconds since boot (optional, can overflow)
    fn seconds() -> u32 {
        Self::now().as_ticks() as u32 / Self::TICKS_PER_SECOND
    }
}

/// Clock time representation (32 or 64 bit)
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct ClockTime(u32);  // Or u64 for 64-bit platforms

impl ClockTime {
    pub const fn from_ticks(ticks: u32) -> Self {
        Self(ticks)
    }

    pub const fn from_seconds<C: Clock>(secs: u32) -> Self {
        Self(secs * C::TICKS_PER_SECOND)
    }

    pub const fn as_ticks(self) -> u32 {
        self.0
    }

    pub fn wrapping_add(self, other: Self) -> Self {
        Self(self.0.wrapping_add(other.0))
    }

    pub fn wrapping_sub(self, other: Self) -> Self {
        Self(self.0.wrapping_sub(other.0))
    }

    /// Check if time has passed (handles wraparound)
    pub fn has_passed(self, reference: Self) -> bool {
        // Signed subtraction to handle wraparound
        (self.0.wrapping_sub(reference.0) as i32) >= 0
    }
}

/// Example platform implementation
#[cfg(feature = "platform-nrf52840")]
pub struct Nrf52840Clock;

#[cfg(feature = "platform-nrf52840")]
impl Clock for Nrf52840Clock {
    fn init() {
        // Initialize RTC peripheral
    }

    fn now() -> ClockTime {
        // Read RTC counter
        ClockTime::from_ticks(unsafe {
            (*nrf52840_pac::RTC0::ptr()).counter.read().bits()
        })
    }

    const TICKS_PER_SECOND: u32 = 32768;  // 32.768 kHz crystal

    fn wait(ticks: ClockTime) {
        let start = Self::now();
        while !Self::now().has_passed(start.wrapping_add(ticks)) {
            // Busy wait
        }
    }

    fn delay_usec(us: u16) {
        // High-precision delay using timer
    }
}
```

**Rationale**:
- Trait-based design allows compile-time platform selection
- No runtime overhead for clock abstraction
- Type-safe time representation prevents mixing ticks/seconds
- Wraparound-safe arithmetic built-in

---

### 3.2 Simple Timers

**Design**: Zero-cost wrapper around clock with expiration logic

```rust
// src/timer.rs

use crate::clock::{Clock, ClockTime};

/// Simple passive timer (no event posting)
#[derive(Copy, Clone)]
pub struct Timer {
    start: ClockTime,
    interval: ClockTime,
}

impl Timer {
    /// Create a new timer (uninitialized state)
    pub const fn new() -> Self {
        Self {
            start: ClockTime::from_ticks(0),
            interval: ClockTime::from_ticks(0),
        }
    }

    /// Set the timer to expire after `interval` from now
    pub fn set<C: Clock>(&mut self, interval: ClockTime) {
        self.start = C::now();
        self.interval = interval;
    }

    /// Check if timer has expired
    pub fn expired<C: Clock>(&self) -> bool {
        C::now().has_passed(self.start.wrapping_add(self.interval))
    }

    /// Reset timer to same interval (stable periodic timers)
    /// Preserves original start time to prevent drift
    pub fn reset(&mut self) {
        self.start = self.start.wrapping_add(self.interval);
    }

    /// Restart timer from current time (may drift)
    pub fn restart<C: Clock>(&mut self) {
        self.start = C::now();
    }

    /// Time remaining until expiration
    pub fn remaining<C: Clock>(&self) -> ClockTime {
        let expiration = self.start.wrapping_add(self.interval);
        let now = C::now();
        if now.has_passed(expiration) {
            ClockTime::from_ticks(0)
        } else {
            expiration.wrapping_sub(now)
        }
    }
}
```

**Rationale**:
- Direct translation of C timer API
- Copy type (8-16 bytes) - can be embedded in other structures
- Generic over Clock trait for platform independence
- No heap allocation, fully const-compatible

---

### 3.3 Protothreads: Async/Await Approach

**Design**: Use Rust's async/await as a type-safe replacement for C macros

**Option A: Pure Async/Await** (Recommended)

```rust
// src/protothread.rs

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

/// Trait for async processes
pub trait Process {
    /// Process name (for debugging)
    const NAME: &'static str;

    /// Process entry point (async function)
    fn run(&mut self) -> impl Future<Output = ()>;
}

/// Helper for waiting on conditions
pub struct WaitUntil<F: FnMut() -> bool> {
    condition: F,
}

impl<F: FnMut() -> bool> Future for WaitUntil<F> {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        if (self.condition)() {
            Poll::Ready(())
        } else {
            cx.waker().wake_by_ref();  // Re-schedule immediately
            Poll::Pending
        }
    }
}

pub fn wait_until<F: FnMut() -> bool>(f: F) -> WaitUntil<F> {
    WaitUntil { condition: f }
}

/// Example async process
pub struct BlinkProcess {
    led_state: bool,
}

impl Process for BlinkProcess {
    const NAME: &'static str = "blink";

    async fn run(&mut self) {
        loop {
            self.led_state = !self.led_state;
            // Wait for timer event (see etimer section)
            wait_event(Event::Timer).await;
        }
    }
}
```

**Option B: Generator-Based** (More explicit control)

```rust
// Alternative using generators (nightly only currently)
#![feature(generators, generator_trait)]

use core::ops::{Generator, GeneratorState};
use core::pin::Pin;

pub struct Protothread<G> {
    generator: G,
}

impl<G> Protothread<G>
where
    G: Generator<Yield = (), Return = ()>,
{
    pub fn new(generator: G) -> Self {
        Self { generator }
    }

    pub fn resume(&mut self) -> bool {
        match Pin::new(&mut self.generator).resume(()) {
            GeneratorState::Yielded(_) => false,  // Still running
            GeneratorState::Complete(_) => true,  // Finished
        }
    }
}

// Example usage
fn blink_process() -> impl Generator<Yield = (), Return = ()> {
    || {
        loop {
            toggle_led();
            yield;  // Yield to scheduler
        }
    }
}
```

**Rationale**:
- Async/await is stable Rust and provides natural protothread semantics
- State machine generated by compiler (no manual switch statements)
- Type-safe: compiler prevents using non-Send types incorrectly
- Future-proof: can use ecosystem async tools
- Generator approach gives more control but requires nightly

---

### 3.4 Process System & Event Loop

**Design**: Static process registry with async executor

```rust
// src/process.rs

use crate::clock::{Clock, ClockTime};
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

/// Process event type
pub type EventType = u8;

/// Built-in system events
pub const EVENT_NONE: EventType = 0x80;
pub const EVENT_INIT: EventType = 0x81;
pub const EVENT_POLL: EventType = 0x82;
pub const EVENT_EXIT: EventType = 0x83;
pub const EVENT_CONTINUE: EventType = 0x85;
pub const EVENT_MSG: EventType = 0x86;
pub const EVENT_EXITED: EventType = 0x87;
pub const EVENT_TIMER: EventType = 0x88;

/// Event data (pointer-sized)
pub type EventData = usize;

/// Event structure
#[derive(Copy, Clone)]
pub struct Event {
    pub event_type: EventType,
    pub data: EventData,
}

/// Process identifier (index into process table)
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct ProcessId(usize);

/// Process state
#[derive(Copy, Clone, PartialEq)]
enum ProcessState {
    Stopped,
    Running,
    Executing,
}

/// Process control block
struct ProcessControlBlock {
    state: ProcessState,
    needs_poll: bool,
    future: Option<Pin<&'static mut dyn Future<Output = ()>>>,
}

/// Event queue (fixed size, static allocation)
const MAX_EVENTS: usize = 32;

struct EventQueue {
    events: [Option<(ProcessId, Event)>; MAX_EVENTS],
    head: usize,
    tail: usize,
}

impl EventQueue {
    const fn new() -> Self {
        Self {
            events: [None; MAX_EVENTS],
            head: 0,
            tail: 0,
        }
    }

    fn push(&mut self, pid: ProcessId, event: Event) -> Result<(), ()> {
        let next_tail = (self.tail + 1) % MAX_EVENTS;
        if next_tail == self.head {
            return Err(());  // Queue full
        }

        self.events[self.tail] = Some((pid, event));
        self.tail = next_tail;
        Ok(())
    }

    fn pop(&mut self) -> Option<(ProcessId, Event)> {
        if self.head == self.tail {
            return None;  // Queue empty
        }

        let event = self.events[self.head].take();
        self.head = (self.head + 1) % MAX_EVENTS;
        event
    }

    fn len(&self) -> usize {
        if self.tail >= self.head {
            self.tail - self.head
        } else {
            MAX_EVENTS - self.head + self.tail
        }
    }
}

/// Global process table (static allocation)
const MAX_PROCESSES: usize = 16;

static mut PROCESS_TABLE: [ProcessControlBlock; MAX_PROCESSES] =
    [ProcessControlBlock {
        state: ProcessState::Stopped,
        needs_poll: false,
        future: None,
    }; MAX_PROCESSES];

static mut EVENT_QUEUE: EventQueue = EventQueue::new();
static mut CURRENT_PROCESS: Option<ProcessId> = None;

/// Process manager
pub struct ProcessManager;

impl ProcessManager {
    /// Initialize the process system
    pub fn init() {
        // Reset all processes
        unsafe {
            for pcb in PROCESS_TABLE.iter_mut() {
                *pcb = ProcessControlBlock {
                    state: ProcessState::Stopped,
                    needs_poll: false,
                    future: None,
                };
            }
            EVENT_QUEUE = EventQueue::new();
            CURRENT_PROCESS = None;
        }
    }

    /// Start a process
    pub fn start<F>(future: F) -> Result<ProcessId, ()>
    where
        F: Future<Output = ()> + 'static,
    {
        unsafe {
            // Find free slot
            for (i, pcb) in PROCESS_TABLE.iter_mut().enumerate() {
                if pcb.state == ProcessState::Stopped {
                    let pid = ProcessId(i);

                    // Store future (need to box it - requires allocator or static)
                    // For no_std, use static storage or intrusive lists
                    pcb.state = ProcessState::Running;
                    pcb.needs_poll = false;

                    // Post INIT event
                    Self::post(pid, Event {
                        event_type: EVENT_INIT,
                        data: 0,
                    })?;

                    return Ok(pid);
                }
            }
            Err(())  // No free slots
        }
    }

    /// Post an event to a process
    pub fn post(pid: ProcessId, event: Event) -> Result<(), ()> {
        unsafe {
            if PROCESS_TABLE[pid.0].state != ProcessState::Stopped {
                EVENT_QUEUE.push(pid, event)
            } else {
                Err(())
            }
        }
    }

    /// Request polling of a process
    pub fn poll(pid: ProcessId) {
        unsafe {
            if PROCESS_TABLE[pid.0].state == ProcessState::Running {
                PROCESS_TABLE[pid.0].needs_poll = true;
            }
        }
    }

    /// Run one iteration of the event loop
    /// Returns number of pending events
    pub fn run() -> usize {
        unsafe {
            // First, handle poll requests
            for (i, pcb) in PROCESS_TABLE.iter_mut().enumerate() {
                if pcb.needs_poll && pcb.state == ProcessState::Running {
                    pcb.needs_poll = false;
                    let pid = ProcessId(i);

                    // Execute process with POLL event
                    Self::execute_process(pid, Event {
                        event_type: EVENT_POLL,
                        data: 0,
                    });
                }
            }

            // Then, process one event from queue
            if let Some((pid, event)) = EVENT_QUEUE.pop() {
                Self::execute_process(pid, event);
            }

            EVENT_QUEUE.len()
        }
    }

    /// Execute a process with an event
    fn execute_process(pid: ProcessId, event: Event) {
        unsafe {
            let pcb = &mut PROCESS_TABLE[pid.0];
            if pcb.state != ProcessState::Running {
                return;
            }

            pcb.state = ProcessState::Executing;
            CURRENT_PROCESS = Some(pid);

            // Poll the future (simplified - needs proper waker)
            if let Some(ref mut future) = pcb.future {
                let waker = dummy_waker();
                let mut context = Context::from_waker(&waker);

                match future.as_mut().poll(&mut context) {
                    Poll::Ready(_) => {
                        // Process completed
                        pcb.state = ProcessState::Stopped;
                        pcb.future = None;
                    }
                    Poll::Pending => {
                        // Process yielded
                        pcb.state = ProcessState::Running;
                    }
                }
            }

            CURRENT_PROCESS = None;
        }
    }

    /// Get current process ID
    pub fn current() -> Option<ProcessId> {
        unsafe { CURRENT_PROCESS }
    }
}

/// Dummy waker for embedded context (no actual wake-up needed)
fn dummy_waker() -> Waker {
    const VTABLE: RawWakerVTable = RawWakerVTable::new(
        |_| RAW_WAKER,  // clone
        |_| {},         // wake
        |_| {},         // wake_by_ref
        |_| {},         // drop
    );
    const RAW_WAKER: RawWaker = RawWaker::new(core::ptr::null(), &VTABLE);

    unsafe { Waker::from_raw(RAW_WAKER) }
}

/// Event waiting future
pub struct WaitEvent {
    expected: Option<EventType>,
}

impl Future for WaitEvent {
    type Output = Event;

    fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<Event> {
        // This needs to integrate with event delivery mechanism
        // Simplified version - real implementation needs event context
        Poll::Pending
    }
}

pub fn wait_event(event_type: EventType) -> WaitEvent {
    WaitEvent {
        expected: Some(event_type),
    }
}
```

**Challenges**:
- Future storage without heap allocation (solutions: static storage, intrusive lists)
- Waker implementation in no_std context (can use dummy waker)
- Event delivery to waiting futures (needs event context in poll)

**Rationale**:
- Static allocation matches Contiki-NG's resource constraints
- Type-safe process IDs prevent invalid references
- Async futures naturally express event-driven logic
- Cooperative scheduling preserved

---

### 3.5 Event Timers

**Design**: Timer wheel integrated with process system

```rust
// src/etimer.rs

use crate::clock::{Clock, ClockTime};
use crate::timer::Timer;
use crate::process::{ProcessId, ProcessManager, EVENT_TIMER, EVENT_POLL};
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

/// Event timer structure
pub struct EventTimer {
    timer: Timer,
    owner: Option<ProcessId>,
    next: Option<&'static mut EventTimer>,  // Intrusive linked list
}

impl EventTimer {
    /// Create a new event timer
    pub const fn new() -> Self {
        Self {
            timer: Timer::new(),
            owner: None,
            next: None,
        }
    }

    /// Set timer for current process
    pub fn set<C: Clock>(&mut self, interval: ClockTime) {
        self.timer.set::<C>(interval);
        self.owner = ProcessManager::current();

        unsafe {
            ETIMER_LIST.add(self);
        }

        // Request poll of etimer process
        ETimerProcess::request_poll();
    }

    /// Check if timer expired
    pub fn expired(&self) -> bool {
        self.owner.is_none()
    }

    /// Reset timer with same interval
    pub fn reset(&mut self) {
        if !self.expired() {
            self.timer.reset();
            ETimerProcess::request_poll();
        }
    }

    /// Stop timer
    pub fn stop(&mut self) {
        self.owner = None;
        unsafe {
            ETIMER_LIST.remove(self);
        }
    }
}

/// Intrusive linked list for timers
struct ETimerList {
    head: Option<&'static mut EventTimer>,
}

impl ETimerList {
    const fn new() -> Self {
        Self { head: None }
    }

    fn add(&mut self, timer: &'static mut EventTimer) {
        timer.next = self.head.take();
        self.head = Some(timer);
    }

    fn remove(&mut self, timer: &'static mut EventTimer) {
        // Implementation: traverse and remove
    }

    fn check_expired<C: Clock>(&mut self) {
        let mut current = &mut self.head;

        while let Some(ref mut timer) = current {
            if timer.owner.is_some() && timer.timer.expired::<C>() {
                // Post timer event to owner
                if let Some(pid) = timer.owner {
                    let _ = ProcessManager::post(pid, crate::process::Event {
                        event_type: EVENT_TIMER,
                        data: timer as *const _ as usize,
                    });
                }
                timer.owner = None;  // Mark as expired
            }
            current = &mut timer.next;
        }
    }
}

static mut ETIMER_LIST: ETimerList = ETimerList::new();
static mut ETIMER_PROCESS_ID: Option<ProcessId> = None;

/// Event timer process (runs in background)
pub struct ETimerProcess;

impl ETimerProcess {
    pub async fn run() {
        loop {
            // Wait for poll event
            crate::process::wait_event(EVENT_POLL).await;

            // Check all timers
            unsafe {
                ETIMER_LIST.check_expired::<platform::SystemClock>();
            }
        }
    }

    pub fn request_poll() {
        unsafe {
            if let Some(pid) = ETIMER_PROCESS_ID {
                ProcessManager::poll(pid);
            }
        }
    }

    /// Initialize etimer system
    pub fn init() {
        unsafe {
            ETIMER_PROCESS_ID = ProcessManager::start(Self::run()).ok();
        }
    }
}

/// Future for waiting on event timer
impl Future for EventTimer {
    type Output = ();

    fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
        if self.expired() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

// Helper macro for async timer waiting
pub async fn delay<C: Clock>(duration: ClockTime) {
    let mut timer = EventTimer::new();
    timer.set::<C>(duration);
    timer.await;
}
```

**Rationale**:
- Intrusive linked list avoids heap allocation
- Integration with process system via events
- Future implementation allows `timer.await` syntax
- Background process matches C architecture

---

### 3.6 Synchronization Primitives

**Design**: Semaphores and channels for process communication

```rust
// src/sync.rs

use crate::process::{ProcessId, ProcessManager, Event};
use core::sync::atomic::{AtomicUsize, Ordering};
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

/// Counting semaphore
pub struct Semaphore {
    count: AtomicUsize,
}

impl Semaphore {
    pub const fn new(initial: usize) -> Self {
        Self {
            count: AtomicUsize::new(initial),
        }
    }

    pub fn signal(&self) {
        self.count.fetch_add(1, Ordering::Release);
    }

    pub fn try_wait(&self) -> bool {
        let mut count = self.count.load(Ordering::Acquire);
        loop {
            if count == 0 {
                return false;
            }

            match self.count.compare_exchange_weak(
                count,
                count - 1,
                Ordering::Acquire,
                Ordering::Acquire,
            ) {
                Ok(_) => return true,
                Err(c) => count = c,
            }
        }
    }
}

/// Wait on semaphore (async)
pub struct SemaphoreWait<'a> {
    sem: &'a Semaphore,
}

impl<'a> Future for SemaphoreWait<'a> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        if self.sem.try_wait() {
            Poll::Ready(())
        } else {
            cx.waker().wake_by_ref();  // Keep polling
            Poll::Pending
        }
    }
}

impl Semaphore {
    pub fn wait(&self) -> SemaphoreWait<'_> {
        SemaphoreWait { sem: self }
    }
}
```

---

## 4. Memory Safety & Performance

### Memory Safety Improvements

| C Issue | Rust Solution |
|---------|---------------|
| PT macros hide control flow | Explicit `async`/`await` shows suspension points |
| Local vars not preserved | Compiler error if non-`static` local used across `.await` |
| Manual memory management | Ownership system prevents leaks |
| Buffer overflows in event queue | Bounds checking (can be optimized out) |
| Null pointer dereferences | `Option<T>` forces handling |
| Data races (if interrupts enabled) | `Send`/`Sync` traits + atomics |

### Performance Considerations

**Zero-Cost Abstractions**:
- Futures compiled to state machines (like C switch statements)
- Generic clock trait monomorphized at compile time
- No vtable overhead for traits with const generics
- Aggressive inlining with `#[inline]` annotations

**Binary Size**:
- Use `opt-level = "z"` for size optimization
- Enable LTO (Link-Time Optimization)
- Strip symbols in release builds
- Expected overhead: 5-15% vs hand-optimized C

**Memory Usage**:
- Static allocation only (no heap)
- Process table: `16 * sizeof(ProcessControlBlock)` ≈ 512 bytes
- Event queue: `32 * sizeof(Event)` ≈ 256 bytes
- Per-process future state: varies (similar to C struct with locals)

---

## 5. Implementation Phases

### Phase 1: Foundation (2-3 weeks)
**Goal**: Basic infrastructure and proof-of-concept

- [ ] Set up Rust project with `no_std` support
- [ ] Implement Clock trait and one platform (e.g., native/POSIX)
- [ ] Implement ClockTime with wraparound handling
- [ ] Implement simple Timer
- [ ] Write unit tests for timer logic
- [ ] Document basic usage patterns

**Deliverable**: Working timer on one platform

---

### Phase 2: Process System (3-4 weeks)
**Goal**: Event-driven kernel with async support

- [ ] Design process table and event queue structures
- [ ] Implement ProcessManager with static allocation
- [ ] Create basic async executor (waker, polling)
- [ ] Implement event posting and delivery
- [ ] Add process lifecycle (start, stop, exit)
- [ ] Implement PROCESS_WAIT_EVENT equivalent
- [ ] Write integration tests

**Deliverable**: Simple async processes communicating via events

---

### Phase 3: Event Timers (2 weeks)
**Goal**: Timer-driven events integrated with processes

- [ ] Implement EventTimer structure
- [ ] Create intrusive linked list for timer management
- [ ] Implement etimer background process
- [ ] Add Future implementation for EventTimer
- [ ] Create helper functions (delay, periodic timers)
- [ ] Test timer expiration and event delivery

**Deliverable**: Async processes using timer events

---

### Phase 4: Platform Support (2-3 weeks per platform)
**Goal**: Multi-platform clock implementations

- [ ] Implement Clock for CC2538 (TI)
- [ ] Implement Clock for nRF52840 (Nordic)
- [ ] Implement Clock for Sky mote
- [ ] Test on real hardware
- [ ] Benchmark performance vs C implementation

**Deliverable**: Multi-platform support with hardware validation

---

### Phase 5: Advanced Features (2-3 weeks)
**Goal**: Complete feature parity with C implementation

- [ ] Implement synchronization primitives (semaphores, mutexes)
- [ ] Add process communication channels
- [ ] Implement poll requests
- [ ] Add process context switching
- [ ] Support for dynamic event allocation
- [ ] Power management hooks (sleep mode integration)

**Deliverable**: Full-featured event kernel

---

### Phase 6: FFI & Migration (3-4 weeks)
**Goal**: Interoperability with existing C code

- [ ] Design C FFI layer
- [ ] Create C-compatible process registration
- [ ] Implement C → Rust event bridging
- [ ] Add Rust → C callback support
- [ ] Write migration guide
- [ ] Create hybrid examples (C + Rust processes)

**Deliverable**: C/Rust interoperability layer

---

### Phase 7: Optimization & Validation (2-3 weeks)
**Goal**: Production-ready implementation

- [ ] Profile binary size and optimize
- [ ] Benchmark latency vs C (should be within 5%)
- [ ] Memory usage analysis
- [ ] Stress testing (long-running, many processes)
- [ ] Security audit (unsafe code review)
- [ ] Documentation and examples

**Deliverable**: Production-ready Rust event kernel

---

## 6. FFI & C Interoperability

### C API Wrapper

```rust
// ffi/mod.rs

use crate::process::{ProcessManager, Event, ProcessId};
use crate::clock::ClockTime;

/// C-compatible process callback
type CProcessCallback = extern "C" fn(event_type: u8, data: usize);

/// Register a C process
#[no_mangle]
pub extern "C" fn rust_process_start(
    callback: CProcessCallback,
) -> usize {
    // Wrap C callback in Rust async function
    async fn c_process_wrapper(callback: CProcessCallback) {
        loop {
            let event = crate::process::wait_any_event().await;
            callback(event.event_type, event.data);
        }
    }

    match ProcessManager::start(c_process_wrapper(callback)) {
        Ok(pid) => pid.0,
        Err(_) => usize::MAX,
    }
}

/// Post event from C code
#[no_mangle]
pub extern "C" fn rust_process_post(
    pid: usize,
    event_type: u8,
    data: usize,
) -> bool {
    ProcessManager::post(
        ProcessId(pid),
        Event { event_type, data },
    ).is_ok()
}

/// Run event loop (callable from C main loop)
#[no_mangle]
pub extern "C" fn rust_process_run() -> usize {
    ProcessManager::run()
}
```

### C Header Generation

Use `cbindgen` to auto-generate C headers:

```toml
# cbindgen.toml
[export]
include = ["ProcessId", "Event"]
prefix = "RUST_"
```

Generated header:
```c
// contiki_ng_rs.h
typedef struct {
    uint8_t event_type;
    uintptr_t data;
} RUST_Event;

uintptr_t rust_process_start(void (*callback)(uint8_t, uintptr_t));
bool rust_process_post(uintptr_t pid, uint8_t event_type, uintptr_t data);
uintptr_t rust_process_run(void);
```

---

## 7. Testing Strategy

### Unit Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_timer_wraparound() {
        let mut timer = Timer::new();
        timer.start = ClockTime::from_ticks(u32::MAX - 10);
        timer.interval = ClockTime::from_ticks(20);

        // Simulate clock wrap
        assert!(ClockTime::from_ticks(15).has_passed(
            timer.start.wrapping_add(timer.interval)
        ));
    }

    #[test]
    fn test_event_queue() {
        let mut queue = EventQueue::new();
        let pid = ProcessId(0);
        let event = Event { event_type: 1, data: 42 };

        assert!(queue.push(pid, event).is_ok());
        assert_eq!(queue.len(), 1);

        let popped = queue.pop();
        assert!(popped.is_some());
        assert_eq!(popped.unwrap().1.data, 42);
    }
}
```

### Integration Tests

```rust
// tests/process_tests.rs

#[test]
fn test_process_communication() {
    ProcessManager::init();

    static RECEIVED: AtomicBool = AtomicBool::new(false);

    async fn receiver() {
        let event = wait_event(100).await;
        assert_eq!(event.data, 0xDEADBEEF);
        RECEIVED.store(true, Ordering::Release);
    }

    let pid = ProcessManager::start(receiver()).unwrap();
    ProcessManager::post(pid, Event {
        event_type: 100,
        data: 0xDEADBEEF,
    }).unwrap();

    while ProcessManager::run() > 0 {}

    assert!(RECEIVED.load(Ordering::Acquire));
}
```

### Hardware-in-the-Loop Tests

- Test on real hardware (nRF52840 DK, CC2538 boards)
- Compare timing accuracy with C implementation
- Power consumption measurements
- Long-running stability tests (days/weeks)

---

## 8. Migration Path

### Hybrid Approach (Recommended)

**Stage 1**: Core kernel in Rust, applications in C
- Replace `process.c`, `etimer.c` with Rust implementations
- Keep existing process declarations in C
- FFI bridge for event posting

**Stage 2**: New features in Rust, legacy in C
- New applications written as Rust async functions
- Existing apps remain in C
- Gradual migration of critical components

**Stage 3**: Full Rust (optional)
- Migrate remaining C processes to Rust
- Remove C FFI layer
- Pure Rust system

### Compatibility Layer

```c
// compat/process_compat.h
// Macros that work with both C and Rust kernels

#ifdef RUST_KERNEL
  #define PROCESS_BEGIN() rust_process_begin()
  #define PROCESS_END() rust_process_end()
  #define PROCESS_WAIT_EVENT() rust_process_wait_event()
#else
  // Original C macros
#endif
```

---

## 9. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Binary size too large | Medium | High | Aggressive optimization, LTO, size profiling |
| Performance regression | Low | High | Early benchmarking, optimize hot paths |
| Toolchain maturity | Medium | Medium | Test on multiple Rust versions, pin version |
| Team adoption | Medium | Medium | Training, documentation, gradual migration |
| Hardware support gaps | Low | Medium | Start with well-supported platforms |
| Async overhead | Low | Medium | Profile and compare with C switch approach |

---

## 10. Success Criteria

### Functional
- ✓ All C event kernel features replicated
- ✓ Pass existing Contiki-NG test suite
- ✓ Run example applications (blink, sensors, networking)
- ✓ Multi-platform support (at least 3 targets)

### Non-Functional
- ✓ Binary size ≤ 110% of C implementation
- ✓ Event latency ≤ 105% of C implementation
- ✓ Memory usage ≤ 110% of C implementation
- ✓ Zero memory safety bugs in safe code
- ✓ 100% test coverage of core kernel

### Developer Experience
- ✓ Comprehensive documentation
- ✓ Example applications in Rust
- ✓ Migration guide from C
- ✓ C FFI for gradual adoption

---

## 11. Future Enhancements

### Post-MVP Features

1. **Async Networking Stack**
   - Async TCP/UDP sockets
   - Non-blocking I/O
   - Zero-copy packet processing

2. **Advanced Scheduling**
   - Priority-based scheduling
   - Deadline-aware scheduling (EDF)
   - Cooperative multitasking with fairness

3. **Power Management**
   - Automatic sleep mode integration
   - Wake-on-event
   - Dynamic voltage/frequency scaling

4. **Tooling**
   - Process visualizer/debugger
   - Event trace analyzer
   - RTOS-aware debugger support

5. **Safety Certification**
   - MISRA Rust compliance
   - Formal verification (Kani, Prusti)
   - Safety-critical subset

---

## 12. Conclusion

Reimplementing Contiki-NG's event kernel in Rust offers significant benefits:

**Pros**:
- Memory safety without runtime overhead
- Modern async/await syntax vs fragile macros
- Strong type system prevents common bugs
- Better tooling and IDE support
- Growing embedded Rust ecosystem

**Cons**:
- Initial development effort (3-6 months)
- Team learning curve
- Toolchain dependency
- Potential binary size increase

**Recommendation**: **Proceed with phased approach**

Start with Phase 1-3 (foundation + core kernel) as a proof-of-concept. Validate performance and size metrics before committing to full migration. Maintain C FFI layer to enable gradual adoption.

The combination of Rust's safety guarantees and Contiki-NG's proven architecture creates a compelling platform for next-generation IoT applications.

---

## Appendix A: Code Size Comparison

**Estimated sizes (ARM Cortex-M, optimized)**:

| Component | C (bytes) | Rust (bytes) | Overhead |
|-----------|-----------|--------------|----------|
| Timer | 200 | 220 | +10% |
| Process core | 800 | 900 | +12% |
| Event queue | 150 | 160 | +7% |
| Etimer | 350 | 400 | +14% |
| **Total** | **1,500** | **1,680** | **+12%** |

*Note: Actual sizes depend on optimization level and platform*

---

## Appendix B: Resource Requirements

**Development Team**:
- 1 Rust expert (embedded systems)
- 1 Contiki-NG expert (C codebase knowledge)
- 1 Platform engineer (hardware testing)

**Timeline**: 4-6 months for full implementation

**Hardware**: Test boards for each target platform

**Tools**:
- Rust toolchain (rustc, cargo)
- Target cross-compilers
- Debuggers (OpenOCD, J-Link)
- Logic analyzer (timing verification)

---

## Appendix C: References

- Contiki-NG Documentation: https://github.com/contiki-ng/contiki-ng
- Rust Embedded Book: https://rust-embedded.github.io/book/
- Embassy (Rust embedded async): https://embassy.dev/
- RTIC (Real-Time Interrupt-driven Concurrency): https://rtic.rs/
- Adam Dunkels' Protothreads: http://dunkels.com/adam/pt/

---

*Document Version: 1.0*
*Date: 2025-11-15*
*Author: Claude (Anthropic)*
