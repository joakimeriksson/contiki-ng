//! Synchronization primitives for process communication
//!
//! Semaphores and other coordination mechanisms.

use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicUsize, Ordering};
use core::task::{Context, Poll};

/// Counting semaphore
pub struct Semaphore {
    count: AtomicUsize,
}

impl Semaphore {
    /// Create a new semaphore with initial count
    pub const fn new(initial: usize) -> Self {
        Self {
            count: AtomicUsize::new(initial),
        }
    }

    /// Signal (increment) the semaphore
    pub fn signal(&self) {
        self.count.fetch_add(1, Ordering::Release);
    }

    /// Try to wait (decrement) without blocking
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

    /// Get current count
    pub fn count(&self) -> usize {
        self.count.load(Ordering::Acquire)
    }

    /// Wait (async)
    pub fn wait(&self) -> SemaphoreWait<'_> {
        SemaphoreWait { sem: self }
    }
}

/// Future for waiting on semaphore
pub struct SemaphoreWait<'a> {
    sem: &'a Semaphore,
}

impl<'a> Future for SemaphoreWait<'a> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
        if self.sem.try_wait() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

/// Binary semaphore (mutex-like)
pub struct BinarySemaphore {
    locked: AtomicUsize,
}

impl BinarySemaphore {
    /// Create a new binary semaphore (unlocked)
    pub const fn new() -> Self {
        Self {
            locked: AtomicUsize::new(0),
        }
    }

    /// Try to acquire the lock
    pub fn try_lock(&self) -> bool {
        self.locked
            .compare_exchange(0, 1, Ordering::Acquire, Ordering::Relaxed)
            .is_ok()
    }

    /// Release the lock
    pub fn unlock(&self) {
        self.locked.store(0, Ordering::Release);
    }

    /// Lock (async)
    pub fn lock(&self) -> BinarySemaphoreWait<'_> {
        BinarySemaphoreWait { sem: self }
    }
}

impl Default for BinarySemaphore {
    fn default() -> Self {
        Self::new()
    }
}

/// Future for locking binary semaphore
pub struct BinarySemaphoreWait<'a> {
    sem: &'a BinarySemaphore,
}

impl<'a> Future for BinarySemaphoreWait<'a> {
    type Output = BinarySemaphoreGuard<'a>;

    fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<Self::Output> {
        if self.sem.try_lock() {
            Poll::Ready(BinarySemaphoreGuard { sem: self.sem })
        } else {
            Poll::Pending
        }
    }
}

/// RAII guard for binary semaphore
pub struct BinarySemaphoreGuard<'a> {
    sem: &'a BinarySemaphore,
}

impl<'a> Drop for BinarySemaphoreGuard<'a> {
    fn drop(&mut self) {
        self.sem.unlock();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_semaphore_basic() {
        let sem = Semaphore::new(2);

        assert_eq!(sem.count(), 2);
        assert!(sem.try_wait());
        assert_eq!(sem.count(), 1);
        assert!(sem.try_wait());
        assert_eq!(sem.count(), 0);
        assert!(!sem.try_wait());

        sem.signal();
        assert_eq!(sem.count(), 1);
        assert!(sem.try_wait());
    }

    #[test]
    fn test_binary_semaphore() {
        let sem = BinarySemaphore::new();

        assert!(sem.try_lock());
        assert!(!sem.try_lock());

        sem.unlock();
        assert!(sem.try_lock());
    }
}
