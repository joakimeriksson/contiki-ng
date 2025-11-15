//! Platform-specific implementations

#[cfg(feature = "platform-native")]
pub mod native;

#[cfg(feature = "platform-native")]
pub use native::NativeClock as SystemClock;
