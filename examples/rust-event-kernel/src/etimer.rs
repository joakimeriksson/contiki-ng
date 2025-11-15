//! Event timer implementation
//!
//! Timers that automatically post events to processes when they expire.

use crate::clock::{Clock, ClockTime};
use crate::process::{Event, ProcessId, ProcessManager, EVENT_POLL, EVENT_TIMER};
use crate::timer::Timer;
use core::cell::RefCell;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

#[cfg(feature = "std")]
use std::sync::Mutex;

/// Event timer structure
pub struct EventTimer {
    timer: Timer,
    owner: Option<ProcessId>,
    id: usize,
}

impl EventTimer {
    /// Create a new event timer
    pub fn new() -> Self {
        static NEXT_ID: core::sync::atomic::AtomicUsize =
            core::sync::atomic::AtomicUsize::new(0);
        Self {
            timer: Timer::new(),
            owner: None,
            id: NEXT_ID.fetch_add(1, core::sync::atomic::Ordering::Relaxed),
        }
    }

    /// Set timer for current process
    pub fn set<C: Clock>(&mut self, interval: ClockTime) {
        #[cfg(feature = "std")]
        eprintln!("[EventTimer::set] Setting timer {} for {:?} ticks", self.id, interval);

        self.timer.set::<C>(interval);
        self.owner = ProcessManager::current();

        #[cfg(feature = "std")]
        eprintln!("[EventTimer::set] Owner: {:?}", self.owner);

        ETIMER_MANAGER.with_lock(|manager| {
            manager.add_timer(self.id, self.timer);
        });

        #[cfg(feature = "std")]
        eprintln!("[EventTimer::set] Calling ETimerProcess::request_poll()");

        ETimerProcess::request_poll();
    }

    /// Check if timer expired
    pub fn expired(&self) -> bool {
        self.owner.is_none()
    }

    /// Reset timer with same interval (stable periodic)
    pub fn reset(&mut self) {
        if !self.expired() {
            self.timer.reset();
            self.owner = ProcessManager::current();

            ETIMER_MANAGER.with_lock(|manager| {
                manager.update_timer(self.id, self.timer);
            });

            ETimerProcess::request_poll();
        }
    }

    /// Restart timer from now
    pub fn restart<C: Clock>(&mut self) {
        self.timer.restart::<C>();
        self.owner = ProcessManager::current();

        ETIMER_MANAGER.with_lock(|manager| {
            manager.update_timer(self.id, self.timer);
        });

        ETimerProcess::request_poll();
    }

    /// Stop timer
    pub fn stop(&mut self) {
        self.owner = None;

        ETIMER_MANAGER.with_lock(|manager| {
            manager.remove_timer(self.id);
        });
    }

    /// Get expiration time
    pub fn expiration_time(&self) -> ClockTime {
        self.timer.expiration_time()
    }

    /// Get start time
    pub fn start_time(&self) -> ClockTime {
        self.timer.start_time()
    }

    /// Mark timer as expired (called by etimer process)
    fn mark_expired(&mut self) {
        self.owner = None;
    }
}

impl Default for EventTimer {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for EventTimer {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Timer entry in global list
#[derive(Copy, Clone)]
struct TimerEntry {
    id: usize,
    timer: Timer,
    owner: ProcessId,
}

/// Event timer manager (global state)
struct ETimerManager {
    timers: Vec<TimerEntry>,
}

impl ETimerManager {
    fn new() -> Self {
        Self {
            timers: Vec::new(),
        }
    }

    fn add_timer(&mut self, id: usize, timer: Timer) {
        if let Some(owner) = ProcessManager::current() {
            #[cfg(feature = "std")]
            eprintln!("[ETimerManager::add_timer] Adding timer {} for process {:?}, total timers: {}",
                      id, owner, self.timers.len() + 1);
            self.timers.push(TimerEntry { id, timer, owner });
        } else {
            #[cfg(feature = "std")]
            eprintln!("[ETimerManager::add_timer] ERROR: No current process!");
        }
    }

    fn update_timer(&mut self, id: usize, timer: Timer) {
        if let Some(entry) = self.timers.iter_mut().find(|e| e.id == id) {
            entry.timer = timer;
            if let Some(owner) = ProcessManager::current() {
                entry.owner = owner;
            }
        }
    }

    fn remove_timer(&mut self, id: usize) {
        self.timers.retain(|e| e.id != id);
    }

    fn check_expired<C: Clock>(&mut self) {
        #[cfg(feature = "std")]
        eprintln!("[etimer] Checking {} active timers", self.timers.len());

        let mut expired = Vec::new();

        for entry in &self.timers {
            if entry.timer.expired::<C>() {
                expired.push(*entry);
            }
        }

        #[cfg(feature = "std")]
        if !expired.is_empty() {
            eprintln!("[etimer] Found {} expired timers", expired.len());
        }

        // Post timer events and remove expired timers
        for entry in expired {
            #[cfg(feature = "std")]
            eprintln!("[etimer] Posting EVENT_TIMER to process {:?} for timer {}", entry.owner, entry.id);

            let _ = ProcessManager::post(
                entry.owner,
                Event::new(EVENT_TIMER, entry.id),
            );
            self.timers.retain(|e| e.id != entry.id);
        }
    }

    fn next_expiration<C: Clock>(&self) -> Option<ClockTime> {
        self.timers
            .iter()
            .map(|e| e.timer.expiration_time())
            .min()
    }

    fn has_active_timers(&self) -> bool {
        !self.timers.is_empty()
    }
}

#[cfg(feature = "std")]
static ETIMER_MANAGER: std::sync::LazyLock<ETimerManagerWrapper> =
    std::sync::LazyLock::new(|| {
        ETimerManagerWrapper(Mutex::new(RefCell::new(ETimerManager::new())))
    });

#[cfg(feature = "std")]
struct ETimerManagerWrapper(Mutex<RefCell<ETimerManager>>);

#[cfg(feature = "std")]
impl ETimerManagerWrapper {
    fn with_lock<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut ETimerManager) -> R,
    {
        let lock = self.0.lock().unwrap();
        let mut manager = lock.borrow_mut();
        f(&mut manager)
    }
}

/// Event timer background process
pub struct ETimerProcess {
    _phantom: core::marker::PhantomData<()>,
}

#[cfg(feature = "std")]
pub(crate) static ETIMER_PROCESS_ID: Mutex<Option<ProcessId>> = Mutex::new(None);

impl ETimerProcess {
    /// Run the etimer process (async)
    pub async fn run<C: Clock>() {
        use crate::process::wait_event_type;

        #[cfg(feature = "std")]
        eprintln!("[etimer] Etimer process started, waiting for events...");

        loop {
            // Wait for poll event
            #[cfg(feature = "std")]
            eprintln!("[etimer] Waiting for POLL event...");

            wait_event_type(EVENT_POLL).await;

            #[cfg(feature = "std")]
            eprintln!("[etimer] Got POLL event, checking timers...");

            // Check all timers
            ETIMER_MANAGER.with_lock(|manager| {
                manager.check_expired::<C>();
            });

            // Don't request another poll - we'll be polled again by the main loop
            // or when a new timer is set
        }
    }

    /// Initialize etimer system
    #[cfg(feature = "std")]
    pub fn init<C: Clock + 'static>() {
        eprintln!("[etimer] Initializing etimer system...");
        let pid = ProcessManager::start("etimer", Self::run::<C>()).ok();
        eprintln!("[etimer] Started etimer process with PID: {:?}", pid);
        *ETIMER_PROCESS_ID.lock().unwrap() = pid;
    }

    /// Request poll of etimer process
    #[cfg(feature = "std")]
    pub fn request_poll() {
        eprintln!("[etimer::request_poll] Called!");
        if let Some(pid) = *ETIMER_PROCESS_ID.lock().unwrap() {
            eprintln!("[etimer::request_poll] Calling ProcessManager::poll({:?})", pid);
            ProcessManager::poll(pid);
        } else {
            eprintln!("[etimer::request_poll] ERROR: No etimer process ID!");
        }
    }

    /// Check if any timers are pending
    #[cfg(feature = "std")]
    pub fn pending() -> bool {
        ETIMER_MANAGER.with_lock(|manager| manager.has_active_timers())
    }

    /// Get next expiration time
    #[cfg(feature = "std")]
    pub fn next_expiration<C: Clock>() -> Option<ClockTime> {
        ETIMER_MANAGER.with_lock(|manager| manager.next_expiration::<C>())
    }
}

/// Future implementation for EventTimer (allows .await syntax)
impl Future for EventTimer {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
        if self.expired() {
            Poll::Ready(())
        } else {
            // Check if we received a timer event for this timer
            if let Some(event) = crate::process::current_event() {
                if event.event_type == EVENT_TIMER && event.data == self.id {
                    self.mark_expired();
                    return Poll::Ready(());
                }
            }
            Poll::Pending
        }
    }
}

/// Async delay helper
pub async fn delay<C: Clock>(duration: ClockTime) {
    let mut timer = EventTimer::new();
    timer.set::<C>(duration);
    timer.await;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::SystemClock;
    use crate::process::wait_event;

    #[test]
    fn test_etimer_basic() {
        ProcessManager::init();
        SystemClock::init();
        ETimerProcess::init::<SystemClock>();

        let done = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let done_clone = done.clone();

        let _pid = ProcessManager::start("timer_test", async move {
            let _init = wait_event().await;

            let mut timer = EventTimer::new();
            timer.set::<SystemClock>(ClockTime::from_ticks(50));

            timer.await;

            done_clone.store(true, std::sync::atomic::Ordering::Release);
        })
        .unwrap();

        // Run event loop
        for _ in 0..100 {
            ProcessManager::run();
            std::thread::sleep(std::time::Duration::from_millis(10));

            if done.load(std::sync::atomic::Ordering::Acquire) {
                break;
            }
        }

        assert!(done.load(std::sync::atomic::Ordering::Acquire));
    }

    #[test]
    fn test_delay_helper() {
        ProcessManager::init();
        SystemClock::init();
        ETimerProcess::init::<SystemClock>();

        let done = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let done_clone = done.clone();

        let _pid = ProcessManager::start("delay_test", async move {
            let _init = wait_event().await;

            delay::<SystemClock>(ClockTime::from_ticks(30)).await;

            done_clone.store(true, std::sync::atomic::Ordering::Release);
        })
        .unwrap();

        // Run event loop
        for _ in 0..100 {
            ProcessManager::run();
            std::thread::sleep(std::time::Duration::from_millis(10));

            if done.load(std::sync::atomic::Ordering::Acquire) {
                break;
            }
        }

        assert!(done.load(std::sync::atomic::Ordering::Acquire));
    }
}
