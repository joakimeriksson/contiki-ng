//! Blink example
//!
//! Simulates blinking an LED using periodic timers.

use contiki_ng_rs::prelude::*;
use contiki_ng_rs::platform::SystemClock;

async fn blink_process() {
    println!("[blink] Process started");

    // Wait for INIT event
    let _init = wait_event().await;
    println!("[blink] Initialized");

    let mut led_state = false;
    let mut timer = Timer::new();

    loop {
        // Toggle LED state
        led_state = !led_state;
        println!("[blink] LED: {}", if led_state { "ON" } else { "OFF" });

        // Set a timer and busy-wait (with yielding to other processes)
        timer.set::<SystemClock>(SystemClock::from_millis(500));
        while !timer.expired::<SystemClock>() {
            yield_now().await;  // Yield to other processes
        }
    }
}

fn main() {
    println!("=== Contiki-NG Rust Event Kernel: Blink Example ===\n");

    // Initialize system
    SystemClock::init();
    ProcessManager::init();

    // Start blink process
    ProcessManager::start("blink", blink_process()).unwrap();

    println!("Starting event loop (press Ctrl+C to exit)\n");

    // Run event loop for 5 seconds
    let start = SystemClock::now();
    let duration = SystemClock::from_millis(5000);

    while !SystemClock::now().has_passed(start + duration) {
        ProcessManager::run();
        std::thread::sleep(std::time::Duration::from_micros(100));
    }

    println!("\n=== Example complete ===");
}
