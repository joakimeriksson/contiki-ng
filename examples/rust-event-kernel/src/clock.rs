//! Clock abstraction layer
//!
//! Provides platform-independent time representation and clock interface.

use core::ops::{Add, Sub};

/// Clock time representation (handles wraparound)
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
#[repr(transparent)]
pub struct ClockTime(pub u32);

impl ClockTime {
    /// Create from raw ticks
    pub const fn from_ticks(ticks: u32) -> Self {
        Self(ticks)
    }

    /// Create from seconds (requires clock frequency)
    pub const fn from_secs(secs: u32, ticks_per_sec: u32) -> Self {
        Self(secs.saturating_mul(ticks_per_sec))
    }

    /// Create from milliseconds (requires clock frequency)
    pub const fn from_millis(millis: u32, ticks_per_sec: u32) -> Self {
        Self((millis as u64 * ticks_per_sec as u64 / 1000) as u32)
    }

    /// Get raw ticks
    pub const fn as_ticks(self) -> u32 {
        self.0
    }

    /// Convert to seconds (approximate)
    pub const fn as_secs(self, ticks_per_sec: u32) -> u32 {
        self.0 / ticks_per_sec
    }

    /// Convert to milliseconds (approximate)
    pub const fn as_millis(self, ticks_per_sec: u32) -> u32 {
        (self.0 as u64 * 1000 / ticks_per_sec as u64) as u32
    }

    /// Wrapping addition (handles overflow)
    pub const fn wrapping_add(self, other: Self) -> Self {
        Self(self.0.wrapping_add(other.0))
    }

    /// Wrapping subtraction (handles underflow)
    pub const fn wrapping_sub(self, other: Self) -> Self {
        Self(self.0.wrapping_sub(other.0))
    }

    /// Check if this time has passed relative to reference time
    /// Handles wraparound correctly
    pub fn has_passed(self, reference: Self) -> bool {
        // Signed subtraction to handle wraparound
        // If self - reference >= 0, then self has passed reference
        (self.0.wrapping_sub(reference.0) as i32) >= 0
    }

    /// Calculate time until target (saturating to 0 if passed)
    pub fn time_until(self, target: Self) -> Self {
        if target.has_passed(self) {
            target.wrapping_sub(self)
        } else {
            Self::from_ticks(0)
        }
    }
}

impl Add for ClockTime {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        self.wrapping_add(other)
    }
}

impl Sub for ClockTime {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        self.wrapping_sub(other)
    }
}

/// Platform-specific clock implementation trait
pub trait Clock {
    /// Number of ticks per second
    const TICKS_PER_SECOND: u32;

    /// Initialize the clock hardware
    fn init();

    /// Get current time in ticks
    fn now() -> ClockTime;

    /// Busy-wait for specified ticks
    fn wait(ticks: ClockTime) {
        let start = Self::now();
        let target = start.wrapping_add(ticks);
        while !Self::now().has_passed(target) {
            // Busy wait
            core::hint::spin_loop();
        }
    }

    /// Microsecond-precision delay (platform-specific)
    fn delay_usec(us: u16);

    /// Get seconds since boot/epoch (may overflow)
    fn seconds() -> u32 {
        Self::now().as_secs(Self::TICKS_PER_SECOND)
    }

    /// Helper: create ClockTime from seconds
    fn from_seconds(secs: u32) -> ClockTime {
        ClockTime::from_secs(secs, Self::TICKS_PER_SECOND)
    }

    /// Helper: create ClockTime from milliseconds
    fn from_millis(millis: u32) -> ClockTime {
        ClockTime::from_millis(millis, Self::TICKS_PER_SECOND)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clock_time_basic() {
        let t1 = ClockTime::from_ticks(100);
        let t2 = ClockTime::from_ticks(50);

        assert_eq!(t1.wrapping_add(t2).as_ticks(), 150);
        assert_eq!(t1.wrapping_sub(t2).as_ticks(), 50);
    }

    #[test]
    fn test_clock_time_wraparound() {
        let t1 = ClockTime::from_ticks(u32::MAX - 10);
        let t2 = ClockTime::from_ticks(20);

        let sum = t1.wrapping_add(t2);
        assert_eq!(sum.as_ticks(), 9);
    }

    #[test]
    fn test_has_passed() {
        let t1 = ClockTime::from_ticks(100);
        let t2 = ClockTime::from_ticks(150);

        assert!(t2.has_passed(t1));
        assert!(!t1.has_passed(t2));
    }

    #[test]
    fn test_has_passed_wraparound() {
        let t1 = ClockTime::from_ticks(u32::MAX - 10);
        let t2 = ClockTime::from_ticks(10); // Wrapped around

        assert!(t2.has_passed(t1));
        assert!(!t1.has_passed(t2));
    }

    #[test]
    fn test_time_conversion() {
        const TICKS_PER_SEC: u32 = 1000;

        let t = ClockTime::from_secs(5, TICKS_PER_SEC);
        assert_eq!(t.as_ticks(), 5000);
        assert_eq!(t.as_secs(TICKS_PER_SEC), 5);

        let t2 = ClockTime::from_millis(1500, TICKS_PER_SEC);
        assert_eq!(t2.as_ticks(), 1500);
        assert_eq!(t2.as_millis(TICKS_PER_SEC), 1500);
    }
}
