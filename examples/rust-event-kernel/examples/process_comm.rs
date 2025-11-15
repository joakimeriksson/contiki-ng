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

// Version without delay for testing
async fn ping_process_no_timer(pong_pid: ProcessId) {
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

// Version without timer for testing
async fn pong_process_no_timer(ping_pid: ProcessId) {
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

    // Use static PIDs - we know ping will be process 0, pong will be process 1
    let ping_pid = ProcessId(0);
    let pong_pid = ProcessId(1);

    // Start both processes
    ProcessManager::start("ping", ping_process_no_timer(pong_pid)).unwrap();
    ProcessManager::start("pong", pong_process_no_timer(ping_pid)).unwrap();

    println!("Starting event loop\n");

    // Run event loop
    loop {
        let pending = ProcessManager::run();
        if pending == 0 {
            std::thread::sleep(std::time::Duration::from_millis(10));
            if ProcessManager::run() == 0 {
                break;
            }
        }
        std::thread::sleep(std::time::Duration::from_micros(100));
    }

    println!("\n=== Example complete ===");
}
