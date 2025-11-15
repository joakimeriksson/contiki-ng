//! Periodic timer example
//!
//! Demonstrates stable periodic timers that don't drift.

use contiki_ng_rs::prelude::*;
use contiki_ng_rs::platform::SystemClock;

async fn periodic_process() {
    println!("[periodic] Process started");

    // Wait for INIT event
    let _init = wait_event().await;
    println!("[periodic] Initialized");

    let mut timer = EventTimer::new();
    let interval = SystemClock::from_millis(250);

    // Set initial timer
    timer.set::<SystemClock>(interval);

    let start_time = SystemClock::now();
    let mut count = 0;

    loop {
        // Wait for timer
        timer.await;

        count += 1;
        let elapsed = SystemClock::now().wrapping_sub(start_time);

        println!(
            "[periodic] Tick #{} at {}ms (expected: {}ms)",
            count,
            elapsed.as_millis(SystemClock::TICKS_PER_SECOND),
            count * 250
        );

        if count >= 10 {
            break;
        }

        // Reset timer (preserves interval, prevents drift)
        timer.reset();
    }

    println!("[periodic] Process complete");
}

fn main() {
    println!("=== Contiki-NG Rust Event Kernel: Periodic Timer Example ===\n");

    // Initialize system
    SystemClock::init();
    ProcessManager::init();
    ETimerProcess::init::<SystemClock>();

    // Start periodic process
    ProcessManager::start("periodic", periodic_process()).unwrap();

    println!("Starting event loop\n");

    // Run event loop until queue is empty
    loop {
        let pending = ProcessManager::run();
        if pending == 0 && !ETimerProcess::pending() {
            std::thread::sleep(std::time::Duration::from_millis(10));
            if ProcessManager::run() == 0 && !ETimerProcess::pending() {
                break;
            }
        }
        std::thread::sleep(std::time::Duration::from_millis(1));
    }

    println!("\n=== Example complete ===");
}
