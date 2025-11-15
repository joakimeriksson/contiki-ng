//! Native/POSIX platform clock implementation
//!
//! Uses system monotonic clock for high-precision timing.

#[cfg(feature = "std")]
use std::time::{Duration, Instant};

use crate::clock::{Clock, ClockTime};

#[cfg(feature = "std")]
use std::sync::OnceLock;

#[cfg(feature = "std")]
static START_TIME: OnceLock<Instant> = OnceLock::new();

/// Native platform clock (uses system monotonic clock)
pub struct NativeClock;

#[cfg(feature = "std")]
impl Clock for NativeClock {
    /// 1000 ticks per second (1ms resolution)
    const TICKS_PER_SECOND: u32 = 1000;

    fn init() {
        START_TIME.get_or_init(Instant::now);
    }

    fn now() -> ClockTime {
        let start = START_TIME.get_or_init(Instant::now);
        let elapsed = start.elapsed();
        let ticks = elapsed.as_millis() as u32;
        ClockTime::from_ticks(ticks)
    }

    fn delay_usec(us: u16) {
        std::thread::sleep(Duration::from_micros(us as u64));
    }

    fn wait(ticks: ClockTime) {
        std::thread::sleep(Duration::from_millis(ticks.as_ticks() as u64));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_native_clock_monotonic() {
        NativeClock::init();

        let t1 = NativeClock::now();
        std::thread::sleep(Duration::from_millis(10));
        let t2 = NativeClock::now();

        assert!(t2.has_passed(t1));
        let diff = t2.wrapping_sub(t1);
        // Should be approximately 10ms (allow some variance)
        assert!(diff.as_ticks() >= 9 && diff.as_ticks() <= 20);
    }

    #[test]
    fn test_native_clock_delay() {
        NativeClock::init();

        let t1 = NativeClock::now();
        NativeClock::delay_usec(5000); // 5ms
        let t2 = NativeClock::now();

        let diff = t2.wrapping_sub(t1);
        assert!(diff.as_ticks() >= 4 && diff.as_ticks() <= 10);
    }
}
