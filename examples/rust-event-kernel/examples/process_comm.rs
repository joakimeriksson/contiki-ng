//! Process communication example
//!
//! Demonstrates inter-process communication using events.

use contiki_ng_rs::prelude::*;
use contiki_ng_rs::platform::SystemClock;

// Custom event types
const EVENT_PING: u8 = 0x10;
const EVENT_PONG: u8 = 0x11;

async fn ping_process(pong_pid: ProcessId) {
    println!("[ping] Process started");

    // Wait for INIT event
    let _init = wait_event().await;
    println!("[ping] Initialized, sending pings to pong process");

    for i in 0..5 {
        println!("[ping] Sending PING #{}", i + 1);
        ProcessManager::post(pong_pid, Event::new(EVENT_PING, i)).unwrap();

        // Wait for PONG response
        let event = wait_event_type(EVENT_PONG).await;
        println!("[ping] Received PONG #{}", event.data);

        // Wait a bit before next ping
        delay::<SystemClock>(SystemClock::from_millis(200)).await;
    }

    println!("[ping] Process complete");
}

async fn pong_process(ping_pid: ProcessId) {
    println!("[pong] Process started");

    // Wait for INIT event
    let _init = wait_event().await;
    println!("[pong] Initialized, waiting for pings");

    for _ in 0..5 {
        // Wait for PING
        let event = wait_event_type(EVENT_PING).await;
        println!("[pong] Received PING #{}", event.data);

        // Respond with PONG
        println!("[pong] Sending PONG #{}", event.data);
        ProcessManager::post(ping_pid, Event::new(EVENT_PONG, event.data)).unwrap();
    }

    println!("[pong] Process complete");
}

fn main() {
    println!("=== Contiki-NG Rust Event Kernel: Process Communication Example ===\n");

    // Initialize system
    SystemClock::init();
    ProcessManager::init();
    ETimerProcess::init::<SystemClock>();

    // Start pong process first (so we can get its PID)
    let pong_pid = ProcessManager::start("pong", async {
        // Placeholder - will be replaced
    })
    .unwrap();

    // Start ping process with pong's PID
    let ping_pid = ProcessManager::start("ping", ping_process(pong_pid)).unwrap();

    // Now start the actual pong process
    ProcessManager::start("pong", pong_process(ping_pid)).unwrap();

    println!("Starting event loop\n");

    // Run event loop
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
