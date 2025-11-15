//! Benchmark example
//!
//! Measures performance metrics of the event kernel.

use contiki_ng_rs::prelude::*;
use contiki_ng_rs::platform::SystemClock;
use std::time::Instant;

// Custom event for benchmarking
const EVENT_BENCH: u8 = 0x20;

/// Benchmark event posting and delivery
fn bench_event_posting() {
    println!("\n=== Benchmark: Event Posting & Delivery ===");

    ProcessManager::init();
    SystemClock::init();

    let iterations = 10000;
    let mut received_count = 0;

    let _pid = ProcessManager::start("receiver", async move {
        let _init = wait_event().await;

        for _ in 0..iterations {
            wait_event_type(EVENT_BENCH).await;
        }
    })
    .unwrap();

    // Process INIT
    ProcessManager::run();

    // Benchmark event posting
    let start = Instant::now();

    for i in 0..iterations {
        ProcessManager::post(ProcessId(0), Event::new(EVENT_BENCH, i)).unwrap();
    }

    let post_duration = start.elapsed();

    // Benchmark event processing
    let start = Instant::now();

    while ProcessManager::run() > 0 {
        received_count += 1;
    }

    let process_duration = start.elapsed();

    println!("  Events posted:    {}", iterations);
    println!("  Events processed: {}", received_count);
    println!("  Post time:        {:?}", post_duration);
    println!("  Process time:     {:?}", process_duration);
    println!(
        "  Post rate:        {:.0} events/sec",
        iterations as f64 / post_duration.as_secs_f64()
    );
    println!(
        "  Process rate:     {:.0} events/sec",
        received_count as f64 / process_duration.as_secs_f64()
    );
    println!(
        "  Avg latency:      {:.2} µs/event",
        process_duration.as_micros() as f64 / received_count as f64
    );
}

/// Benchmark timer accuracy
fn bench_timer_accuracy() {
    println!("\n=== Benchmark: Timer Accuracy ===");

    ProcessManager::init();
    SystemClock::init();
    ETimerProcess::init::<SystemClock>();

    let test_cases = vec![
        ("10ms", SystemClock::from_millis(10)),
        ("50ms", SystemClock::from_millis(50)),
        ("100ms", SystemClock::from_millis(100)),
        ("500ms", SystemClock::from_millis(500)),
    ];

    for (name, interval) in test_cases {
        let interval_copy = interval;

        let _pid = ProcessManager::start("timer_test", async move {
            let _init = wait_event().await;

            let start = SystemClock::now();
            delay::<SystemClock>(interval_copy).await;
            let elapsed = SystemClock::now().wrapping_sub(start);

            let expected_ms = interval_copy.as_millis(SystemClock::TICKS_PER_SECOND);
            let actual_ms = elapsed.as_millis(SystemClock::TICKS_PER_SECOND);
            let error = (actual_ms as i32 - expected_ms as i32).abs();

            println!(
                "  {} - Expected: {}ms, Actual: {}ms, Error: {}ms",
                name, expected_ms, actual_ms, error
            );
        })
        .unwrap();

        // Run until complete
        loop {
            let pending = ProcessManager::run();
            if pending == 0 && !ETimerProcess::pending() {
                std::thread::sleep(std::time::Duration::from_millis(1));
                if ProcessManager::run() == 0 {
                    break;
                }
            }
            std::thread::sleep(std::time::Duration::from_micros(100));
        }
    }
}

/// Benchmark process switching overhead
fn bench_process_switching() {
    println!("\n=== Benchmark: Process Switching ===");

    ProcessManager::init();
    SystemClock::init();

    let iterations = 1000;

    for i in 0..3 {
        let _pid = ProcessManager::start(&format!("process_{}", i), async move {
            let _init = wait_event().await;

            for _ in 0..iterations {
                yield_now().await;
            }
        })
        .unwrap();
    }

    // Process all INITs
    for _ in 0..3 {
        ProcessManager::run();
    }

    let start = Instant::now();

    while ProcessManager::run() > 0 {}

    let duration = start.elapsed();

    let total_switches = iterations * 3;
    println!("  Total yields:     {}", total_switches);
    println!("  Duration:         {:?}", duration);
    println!(
        "  Switch rate:      {:.0} switches/sec",
        total_switches as f64 / duration.as_secs_f64()
    );
    println!(
        "  Avg switch time:  {:.2} µs",
        duration.as_micros() as f64 / total_switches as f64
    );
}

/// Benchmark memory usage
fn bench_memory_usage() {
    println!("\n=== Benchmark: Memory Usage ===");

    println!("  ClockTime:        {} bytes", std::mem::size_of::<ClockTime>());
    println!("  Timer:            {} bytes", std::mem::size_of::<Timer>());
    println!("  EventTimer:       {} bytes", std::mem::size_of::<EventTimer>());
    println!("  Event:            {} bytes", std::mem::size_of::<Event>());
    println!("  ProcessId:        {} bytes", std::mem::size_of::<ProcessId>());
    println!("  Semaphore:        {} bytes", std::mem::size_of::<Semaphore>());
    println!(
        "  BinarySemaphore:  {} bytes",
        std::mem::size_of::<BinarySemaphore>()
    );
}

fn main() {
    println!("=== Contiki-NG Rust Event Kernel: Benchmark ===");

    bench_memory_usage();
    bench_event_posting();
    bench_timer_accuracy();
    bench_process_switching();

    println!("\n=== Benchmarks complete ===");
}
