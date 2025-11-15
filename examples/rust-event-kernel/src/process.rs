//! Process system with async executor
//!
//! Event-driven cooperative multitasking using async/await.

use core::cell::Cell;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

#[cfg(feature = "std")]
use std::sync::Mutex;

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
#[derive(Copy, Clone, Debug)]
pub struct Event {
    pub event_type: EventType,
    pub data: EventData,
}

impl Event {
    pub const fn new(event_type: EventType, data: EventData) -> Self {
        Self { event_type, data }
    }
}

/// Process identifier
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct ProcessId(pub usize);

/// Process state
#[derive(Copy, Clone, PartialEq, Debug)]
enum ProcessState {
    Stopped,
    Running,
    Executing,
}

/// Process context for event delivery
#[derive(Copy, Clone, Debug)]
pub struct ProcessContext {
    pub current_event: Option<Event>,
    pub current_process: Option<ProcessId>,
}

thread_local! {
    static PROCESS_CONTEXT: Cell<ProcessContext> = Cell::new(ProcessContext {
        current_event: None,
        current_process: None,
    });
}

/// Get current event in process context
pub fn current_event() -> Option<Event> {
    PROCESS_CONTEXT.with(|ctx| ctx.get().current_event)
}

/// Set current event in process context
fn set_current_event(event: Option<Event>) {
    PROCESS_CONTEXT.with(|ctx| {
        let mut c = ctx.get();
        c.current_event = event;
        ctx.set(c);
    });
}

/// Get current process ID
pub fn current_process() -> Option<ProcessId> {
    PROCESS_CONTEXT.with(|ctx| ctx.get().current_process)
}

/// Set current process in process context
fn set_current_process(pid: Option<ProcessId>) {
    PROCESS_CONTEXT.with(|ctx| {
        let mut c = ctx.get();
        c.current_process = pid;
        ctx.set(c);
    });
}

/// Process future type
type ProcessFuture = Pin<Box<dyn Future<Output = ()> + Send>>;

/// Process control block
struct ProcessControlBlock {
    state: ProcessState,
    needs_poll: bool,
    future: Option<ProcessFuture>,
    name: &'static str,
}

impl ProcessControlBlock {
    const fn new() -> Self {
        Self {
            state: ProcessState::Stopped,
            needs_poll: false,
            future: None,
            name: "",
        }
    }
}

/// Event queue entry
#[derive(Copy, Clone)]
struct EventQueueEntry {
    pid: ProcessId,
    event: Event,
}

/// Fixed-size event queue
struct EventQueue {
    events: [Option<EventQueueEntry>; MAX_EVENTS],
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
            return Err(()); // Queue full
        }

        self.events[self.tail] = Some(EventQueueEntry { pid, event });
        self.tail = next_tail;
        Ok(())
    }

    fn pop(&mut self) -> Option<EventQueueEntry> {
        if self.head == self.tail {
            return None; // Queue empty
        }

        let entry = self.events[self.head].take();
        self.head = (self.head + 1) % MAX_EVENTS;
        entry
    }

    fn len(&self) -> usize {
        if self.tail >= self.head {
            self.tail - self.head
        } else {
            MAX_EVENTS - self.head + self.tail
        }
    }

    fn is_empty(&self) -> bool {
        self.head == self.tail
    }
}

/// Maximum number of processes
const MAX_PROCESSES: usize = 16;

/// Maximum number of events in queue
const MAX_EVENTS: usize = 32;

/// Global process state
#[cfg(feature = "std")]
static PROCESS_STATE: Mutex<ProcessState_> = Mutex::new(ProcessState_::new());

#[cfg(feature = "std")]
struct ProcessState_ {
    process_table: [ProcessControlBlock; MAX_PROCESSES],
    event_queue: EventQueue,
}

#[cfg(feature = "std")]
impl ProcessState_ {
    const fn new() -> Self {
        const PCB: ProcessControlBlock = ProcessControlBlock::new();
        Self {
            process_table: [PCB; MAX_PROCESSES],
            event_queue: EventQueue::new(),
        }
    }
}

/// Process manager
pub struct ProcessManager;

impl ProcessManager {
    /// Initialize the process system
    #[cfg(feature = "std")]
    pub fn init() {
        let mut state = PROCESS_STATE.lock().unwrap();
        state.event_queue = EventQueue::new();
        for pcb in &mut state.process_table {
            pcb.state = ProcessState::Stopped;
            pcb.needs_poll = false;
            pcb.future = None;
            pcb.name = "";
        }
    }

    /// Start a process with a name
    #[cfg(feature = "std")]
    pub fn start<F>(name: &'static str, future: F) -> Result<ProcessId, ()>
    where
        F: Future<Output = ()> + Send + 'static,
    {
        let mut state = PROCESS_STATE.lock().unwrap();

        // Find free slot
        for (i, pcb) in state.process_table.iter_mut().enumerate() {
            if pcb.state == ProcessState::Stopped {
                let pid = ProcessId(i);

                pcb.state = ProcessState::Running;
                pcb.needs_poll = false;
                pcb.future = Some(Box::pin(future));
                pcb.name = name;

                // Post INIT event
                drop(state);
                Self::post(pid, Event::new(EVENT_INIT, 0))?;

                return Ok(pid);
            }
        }
        Err(()) // No free slots
    }

    /// Post an event to a process
    #[cfg(feature = "std")]
    pub fn post(pid: ProcessId, event: Event) -> Result<(), ()> {
        let mut state = PROCESS_STATE.lock().unwrap();

        if state.process_table[pid.0].state != ProcessState::Stopped {
            state.event_queue.push(pid, event)
        } else {
            Err(())
        }
    }

    /// Post an event synchronously (execute immediately)
    #[cfg(feature = "std")]
    pub fn post_synch(pid: ProcessId, event: Event) -> Result<(), ()> {
        // Check if process is stopped
        {
            let state = PROCESS_STATE.lock().unwrap();
            if state.process_table[pid.0].state == ProcessState::Stopped {
                return Err(());
            }
        }

        // Execute immediately
        let old_current = current_process();
        set_current_process(Some(pid));
        set_current_event(Some(event));

        // Set state to Executing and poll without holding lock
        let poll_result = {
            let mut state = PROCESS_STATE.lock().unwrap();
            state.process_table[pid.0].state = ProcessState::Executing;

            if let Some(future) = state.process_table[pid.0].future.take() {
                drop(state);

                let waker = process_waker(pid);
                let mut context = Context::from_waker(&waker);

                let mut future = future;
                let result = future.as_mut().poll(&mut context);

                // Put future back
                let mut state = PROCESS_STATE.lock().unwrap();
                state.process_table[pid.0].future = Some(future);
                Some(result)
            } else {
                None
            }
        };

        if let Some(result) = poll_result {
            let mut state = PROCESS_STATE.lock().unwrap();
            match result {
                Poll::Ready(_) => {
                    state.process_table[pid.0].state = ProcessState::Stopped;
                    state.process_table[pid.0].future = None;
                }
                Poll::Pending => {
                    state.process_table[pid.0].state = ProcessState::Running;
                }
            }
        }

        set_current_event(None);
        set_current_process(old_current);
        Ok(())
    }

    /// Request polling of a process
    #[cfg(feature = "std")]
    pub fn poll(pid: ProcessId) {
        eprintln!("[ProcessManager::poll] Called for {:?}", pid);
        let mut state = PROCESS_STATE.lock().unwrap();
        // Allow polling for Running or Executing processes
        if state.process_table[pid.0].state == ProcessState::Running
            || state.process_table[pid.0].state == ProcessState::Executing
        {
            eprintln!("[ProcessManager::poll] Setting needs_poll=true for {:?}", pid);
            state.process_table[pid.0].needs_poll = true;
        } else {
            eprintln!("[ProcessManager::poll] Process {:?} not in valid state, state={:?}",
                      pid, state.process_table[pid.0].state);
        }
    }

    /// Run one iteration of the event loop
    /// Returns number of pending events
    #[cfg(feature = "std")]
    pub fn run() -> usize {
        // First, handle poll requests
        let poll_pids: Vec<ProcessId> = {
            let mut state = PROCESS_STATE.lock().unwrap();
            state
                .process_table
                .iter_mut()
                .enumerate()
                .filter_map(|(i, pcb)| {
                    if pcb.needs_poll && pcb.state == ProcessState::Running {
                        eprintln!("[ProcessManager] Process {:?} needs poll", ProcessId(i));
                        pcb.needs_poll = false;
                        Some(ProcessId(i))
                    } else {
                        None
                    }
                })
                .collect()
        };

        if !poll_pids.is_empty() {
            eprintln!("[ProcessManager] Processing {} poll requests", poll_pids.len());
        }

        for pid in poll_pids {
            eprintln!("[ProcessManager] Executing poll for {:?}", pid);
            Self::execute_process(pid, Event::new(EVENT_POLL, 0));
        }

        // Process one event from queue if available
        let entry = {
            let mut state = PROCESS_STATE.lock().unwrap();
            state.event_queue.pop()
        };

        if let Some(entry) = entry {
            Self::execute_process(entry.pid, entry.event);
        }

        let state = PROCESS_STATE.lock().unwrap();
        state.event_queue.len()
    }

    /// Execute a process with an event
    #[cfg(feature = "std")]
    fn execute_process(pid: ProcessId, event: Event) {
        // Check if process is running and set state to Executing
        {
            let mut state = PROCESS_STATE.lock().unwrap();
            if state.process_table[pid.0].state != ProcessState::Running {
                return;
            }
            state.process_table[pid.0].state = ProcessState::Executing;
        }

        // Set current process and event in thread-local storage
        set_current_process(Some(pid));
        set_current_event(Some(event));

        let waker = process_waker(pid);
        let mut context = Context::from_waker(&waker);

        // Poll the future WITHOUT holding the lock
        // We need to temporarily take ownership of the future, poll it, and put it back
        let poll_result = {
            let mut state = PROCESS_STATE.lock().unwrap();
            let pcb = &mut state.process_table[pid.0];
            if let Some(future) = pcb.future.take() {
                // Drop the lock before polling
                drop(state);

                // Poll the future
                let mut future = future;
                let result = future.as_mut().poll(&mut context);

                // Re-acquire lock and put future back
                let mut state = PROCESS_STATE.lock().unwrap();
                state.process_table[pid.0].future = Some(future);
                result
            } else {
                return;
            }
        };

        match poll_result {
            Poll::Ready(_) => {
                let mut state = PROCESS_STATE.lock().unwrap();
                state.process_table[pid.0].state = ProcessState::Stopped;
                state.process_table[pid.0].future = None;

                // Post EXITED event to all processes
                drop(state);
                for i in 0..MAX_PROCESSES {
                    let _ = Self::post(ProcessId(i), Event::new(EVENT_EXITED, pid.0));
                }
                set_current_event(None);
                set_current_process(None);
                return;
            }
            Poll::Pending => {
                let mut state = PROCESS_STATE.lock().unwrap();
                state.process_table[pid.0].state = ProcessState::Running;
            }
        }

        set_current_event(None);
        set_current_process(None);
    }

    /// Get current process ID
    #[cfg(feature = "std")]
    pub fn current() -> Option<ProcessId> {
        current_process()
    }

    /// Check if process is running
    #[cfg(feature = "std")]
    pub fn is_running(pid: ProcessId) -> bool {
        let state = PROCESS_STATE.lock().unwrap();
        state.process_table[pid.0].state != ProcessState::Stopped
    }

    /// Get number of pending events
    #[cfg(feature = "std")]
    pub fn num_events() -> usize {
        let state = PROCESS_STATE.lock().unwrap();
        state.event_queue.len()
    }

    /// Exit current process
    #[cfg(feature = "std")]
    pub fn exit() {
        if let Some(pid) = Self::current() {
            let mut state = PROCESS_STATE.lock().unwrap();
            state.process_table[pid.0].state = ProcessState::Stopped;
            state.process_table[pid.0].future = None;
        }
    }
}

/// Create a waker for a process
fn process_waker(pid: ProcessId) -> Waker {
    unsafe fn clone(data: *const ()) -> RawWaker {
        RawWaker::new(data, &VTABLE)
    }

    unsafe fn wake(data: *const ()) {
        let pid = ProcessId(data as usize);
        ProcessManager::poll(pid);
    }

    unsafe fn wake_by_ref(data: *const ()) {
        let pid = ProcessId(data as usize);
        ProcessManager::poll(pid);
    }

    unsafe fn drop(_data: *const ()) {}

    const VTABLE: RawWakerVTable = RawWakerVTable::new(clone, wake, wake_by_ref, drop);

    let raw_waker = RawWaker::new(pid.0 as *const (), &VTABLE);
    unsafe { Waker::from_raw(raw_waker) }
}

/// Future for waiting on any event
pub struct WaitEvent {
    expected: Option<EventType>,
}

impl Future for WaitEvent {
    type Output = Event;

    fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<Event> {
        if let Some(event) = current_event() {
            if let Some(expected) = self.expected {
                if event.event_type == expected {
                    return Poll::Ready(event);
                }
            } else {
                return Poll::Ready(event);
            }
        }
        Poll::Pending
    }
}

/// Wait for any event
pub fn wait_event() -> WaitEvent {
    WaitEvent { expected: None }
}

/// Wait for specific event type
pub fn wait_event_type(event_type: EventType) -> WaitEvent {
    WaitEvent {
        expected: Some(event_type),
    }
}

/// Future for waiting until condition is true
pub struct WaitUntil<F>
where
    F: FnMut() -> bool,
{
    condition: F,
}

impl<F> Future for WaitUntil<F>
where
    F: FnMut() -> bool + Unpin,
{
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
        if (self.condition)() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

/// Wait until condition is true
pub fn wait_until<F>(condition: F) -> WaitUntil<F>
where
    F: FnMut() -> bool,
{
    WaitUntil { condition }
}

/// Yield execution
pub struct Yield {
    yielded: bool,
}

impl Future for Yield {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
        if self.yielded {
            Poll::Ready(())
        } else {
            self.yielded = true;
            // Request poll so we get called again
            if let Some(pid) = ProcessManager::current() {
                ProcessManager::poll(pid);
            }
            Poll::Pending
        }
    }
}

/// Yield to other processes
pub fn yield_now() -> Yield {
    Yield { yielded: false }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_process_basic() {
        ProcessManager::init();

        let pid = ProcessManager::start("test", async {
            let event = wait_event().await;
            assert_eq!(event.event_type, EVENT_INIT);
        })
        .unwrap();

        // Process one event (INIT)
        ProcessManager::run();

        // Process should be done
        std::thread::sleep(std::time::Duration::from_millis(10));
    }

    #[test]
    fn test_event_queue() {
        ProcessManager::init();

        let pid = ProcessManager::start("receiver", async {
            let event = wait_event().await;
            assert_eq!(event.event_type, EVENT_INIT);

            let event = wait_event_type(100).await;
            assert_eq!(event.data, 42);
        })
        .unwrap();

        // Process INIT
        ProcessManager::run();

        // Post custom event
        ProcessManager::post(pid, Event::new(100, 42)).unwrap();

        // Process custom event
        ProcessManager::run();
    }

    #[test]
    fn test_yield() {
        use std::sync::atomic::{AtomicUsize, Ordering};

        ProcessManager::init();

        static COUNTER: AtomicUsize = AtomicUsize::new(0);

        let _pid = ProcessManager::start("yielder", async {
            let _event = wait_event().await; // INIT

            for _ in 0..5 {
                COUNTER.fetch_add(1, Ordering::SeqCst);
                yield_now().await;
            }
        })
        .unwrap();

        // Run until queue is empty
        while ProcessManager::run() > 0 {
            std::thread::sleep(std::time::Duration::from_millis(1));
        }

        assert_eq!(COUNTER.load(Ordering::SeqCst), 5);
    }
}
