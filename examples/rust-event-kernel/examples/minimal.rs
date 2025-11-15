//! Minimal working example
//! Tests basic process system without timers or complex event matching

use contiki_ng_rs::prelude::*;
use contiki_ng_rs::platform::SystemClock;

async fn process_a() {
    println!("[A] Started");
    let event = wait_event().await;  // Wait for INIT
    println!("[A] Got INIT event: {:?}", event.event_type);

    for i in 0..3 {
        println!("[A] Iteration {}", i);
        yield_now().await;
    }

    println!("[A] Complete");
}

async fn process_b() {
    println!("[B] Started");
    let event = wait_event().await;  // Wait for INIT
    println!("[B] Got INIT event: {:?}", event.event_type);

    for i in 0..3 {
        println!("[B] Iteration {}", i);
        yield_now().await;
    }

    println!("[B] Complete");
}

fn main() {
    println!("=== Minimal Test ===\n");

    SystemClock::init();
    ProcessManager::init();

    println!("Starting processes...");
    ProcessManager::start("process_a", process_a()).unwrap();
    ProcessManager::start("process_b", process_b()).unwrap();

    println!("Running event loop...\n");

    // Run for a bit
    for _ in 0..20 {
        let pending = ProcessManager::run();
        println!("  [loop] pending events: {}", pending);
        std::thread::sleep(std::time::Duration::from_millis(50));
    }

    println!("\n=== Done ===");
}
