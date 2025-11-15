//! Contiki-NG Event Kernel in Rust
//!
//! A modern, type-safe implementation of Contiki-NG's event-driven kernel
//! using Rust's async/await.
//!
//! # Features
//!
//! - **Type-safe processes**: Async/await instead of macro-based protothreads
//! - **Memory safety**: No undefined behavior, no buffer overflows
//! - **Zero-cost abstractions**: Compiles to efficient state machines
//! - **Platform-independent**: Abstract clock interface
//! - **Event-driven**: Cooperative multitasking with event queue
//!
//! # Example
//!
//! ```rust,no_run
//! use contiki_ng_rs::prelude::*;
//! use contiki_ng_rs::platform::SystemClock;
//!
//! async fn blink_process() {
//!     let _init = wait_event().await;
//!
//!     loop {
//!         // Toggle LED
//!         delay::<SystemClock>(ClockTime::from_ticks(500)).await;
//!     }
//! }
//!
//! fn main() {
//!     SystemClock::init();
//!     ProcessManager::init();
//!     ETimerProcess::init::<SystemClock>();
//!
//!     ProcessManager::start("blink", blink_process()).unwrap();
//!
//!     loop {
//!         ProcessManager::run();
//!     }
//! }
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

pub mod clock;
pub mod etimer;
pub mod platform;
pub mod process;
pub mod sync;
pub mod timer;

/// Convenience re-exports
pub mod prelude {
    pub use crate::clock::{Clock, ClockTime};
    pub use crate::etimer::{delay, EventTimer, ETimerProcess};
    pub use crate::process::{
        wait_event, wait_event_type, wait_until, yield_now, Event, ProcessId, ProcessManager,
        EVENT_CONTINUE, EVENT_EXITED, EVENT_EXIT, EVENT_INIT, EVENT_MSG, EVENT_NONE, EVENT_POLL,
        EVENT_TIMER,
    };
    pub use crate::sync::{BinarySemaphore, Semaphore};
    pub use crate::timer::Timer;

    #[cfg(feature = "platform-native")]
    pub use crate::platform::native::NativeClock;
}

#[cfg(test)]
mod tests {
    use super::prelude::*;

    #[test]
    fn test_basic_integration() {
        use crate::platform::SystemClock;

        ProcessManager::init();
        SystemClock::init();

        let _pid = ProcessManager::start("test", async {
            let event = wait_event().await;
            assert_eq!(event.event_type, EVENT_INIT);
        })
        .unwrap();

        ProcessManager::run();
    }
}
