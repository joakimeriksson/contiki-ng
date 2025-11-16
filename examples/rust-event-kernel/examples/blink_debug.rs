//! Debug blink example with logging

use contiki_ng_rs::prelude::*;
use contiki_ng_rs::platform::SystemClock;

async fn blink_process() {
    println!("[blink] Process started");

    let _init = wait_event().await;
    println!("[blink] Got INIT event");

    let mut led_state = false;

    for i in 0..3 {
        led_state = !led_state;
        println!("[blink] Iteration {}, LED: {}", i, if led_state { "ON" } else { "OFF" });

        println!("[blink] Setting timer for 100ms...");
        delay::<SystemClock>(SystemClock::from_millis(100)).await;
        println!("[blink] Timer expired!");
    }

    println!("[blink] Complete");
}

fn main() {
    println!("=== Debug Blink Example ===\n");

    SystemClock::init();
    ProcessManager::init();
    ETimerProcess::init::<SystemClock>();

    ProcessManager::start("blink", blink_process()).unwrap();

    println!("Running event loop...\n");

    let start = SystemClock::now();
    let duration = SystemClock::from_millis(2000);

    let mut iterations = 0;
    while !SystemClock::now().has_passed(start + duration) {
        let pending = ProcessManager::run();
        iterations += 1;

        // Periodically poll etimer to check for expirations
        // Poll every 250 iterations (≈125ms with 500μs sleep)
        if iterations % 250 == 0 && ETimerProcess::pending() {
            ProcessManager::poll(ProcessId(0)); // Poll etimer process (PID 0)
        }

        if iterations % 100 == 0 {
            println!("[main] Iteration {}, pending events: {}", iterations, pending);
        }

        std::thread::sleep(std::time::Duration::from_micros(500));
    }

    println!("\n=== Done after {} iterations ===", iterations);
}
