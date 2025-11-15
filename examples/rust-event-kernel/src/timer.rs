//! Simple passive timer implementation
//!
//! Timers do not post events automatically; applications must poll them.

use crate::clock::{Clock, ClockTime};

/// Simple timer (passive - no event posting)
#[derive(Copy, Clone, Debug)]
pub struct Timer {
    start: ClockTime,
    interval: ClockTime,
}

impl Timer {
    /// Create a new uninitialized timer
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

    /// Set the timer with explicit start time
    pub fn set_at(&mut self, start: ClockTime, interval: ClockTime) {
        self.start = start;
        self.interval = interval;
    }

    /// Check if timer has expired
    pub fn expired<C: Clock>(&self) -> bool {
        let expiration = self.start.wrapping_add(self.interval);
        C::now().has_passed(expiration)
    }

    /// Reset timer to same interval (preserves start time - prevents drift)
    /// Use this for stable periodic timers
    pub fn reset(&mut self) {
        self.start = self.start.wrapping_add(self.interval);
    }

    /// Restart timer from current time (may drift)
    /// Use this if you want to restart from "now"
    pub fn restart<C: Clock>(&mut self) {
        self.start = C::now();
    }

    /// Get time remaining until expiration (0 if expired)
    pub fn remaining<C: Clock>(&self) -> ClockTime {
        let expiration = self.start.wrapping_add(self.interval);
        let now = C::now();

        if now.has_passed(expiration) {
            ClockTime::from_ticks(0)
        } else {
            expiration.wrapping_sub(now)
        }
    }

    /// Get the start time
    pub fn start_time(&self) -> ClockTime {
        self.start
    }

    /// Get the interval
    pub fn interval(&self) -> ClockTime {
        self.interval
    }

    /// Get expiration time
    pub fn expiration_time(&self) -> ClockTime {
        self.start.wrapping_add(self.interval)
    }
}

impl Default for Timer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::SystemClock;

    #[test]
    fn test_timer_basic() {
        SystemClock::init();

        let mut timer = Timer::new();
        timer.set::<SystemClock>(ClockTime::from_ticks(100));

        // Should not be expired immediately
        assert!(!timer.expired::<SystemClock>());

        // Wait for expiration
        SystemClock::wait(ClockTime::from_ticks(110));
        assert!(timer.expired::<SystemClock>());
    }

    #[test]
    fn test_timer_remaining() {
        SystemClock::init();

        let mut timer = Timer::new();
        timer.set::<SystemClock>(ClockTime::from_ticks(100));

        let remaining = timer.remaining::<SystemClock>();
        assert!(remaining.as_ticks() > 0 && remaining.as_ticks() <= 100);

        SystemClock::wait(ClockTime::from_ticks(110));
        assert_eq!(timer.remaining::<SystemClock>().as_ticks(), 0);
    }

    #[test]
    fn test_timer_reset() {
        SystemClock::init();

        let mut timer = Timer::new();
        timer.set::<SystemClock>(ClockTime::from_ticks(50));

        let start1 = timer.start_time();

        // Reset (should preserve interval and shift start time)
        timer.reset();
        let start2 = timer.start_time();

        assert_eq!(start2.wrapping_sub(start1).as_ticks(), 50);
        assert_eq!(timer.interval().as_ticks(), 50);
    }

    #[test]
    fn test_timer_restart() {
        SystemClock::init();

        let mut timer = Timer::new();
        let t1 = SystemClock::now();
        timer.set::<SystemClock>(ClockTime::from_ticks(50));

        SystemClock::wait(ClockTime::from_ticks(10));

        // Restart from now
        timer.restart::<SystemClock>();
        let t2 = SystemClock::now();

        // Start time should be approximately current time
        let diff = timer.start_time().wrapping_sub(t2);
        assert!(diff.as_ticks() <= 5); // Allow some tolerance
    }
}
