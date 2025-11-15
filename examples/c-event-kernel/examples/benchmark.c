/**
 * C Event Kernel Benchmark
 * For comparison with Rust implementation
 */

#define _POSIX_C_SOURCE 199309L

#include "../src/process.h"
#include "../src/clock.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define BENCH_EVENT 0x20

/* Benchmark: Event posting and delivery */
static volatile unsigned int received_count = 0;

PROCESS(receiver_process, "receiver");

PROCESS_THREAD(receiver_process, ev, data) {
    PROCESS_BEGIN();

    static unsigned int iterations = 10000;

    while(received_count < iterations) {
        PROCESS_WAIT_EVENT();
        if(ev == BENCH_EVENT) {
            received_count++;
        }
    }

    PROCESS_END();
}

void bench_event_posting(void) {
    printf("\n=== Benchmark: Event Posting & Delivery (C) ===\n");

    process_init();
    clock_init();

    unsigned int iterations = 10000;
    received_count = 0;

    process_start(&receiver_process, NULL);

    /* Process INIT event */
    process_run();

    /* Benchmark event posting */
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    for(unsigned int i = 0; i < iterations; i++) {
        process_post(&receiver_process, BENCH_EVENT, (void*)(uintptr_t)i);
    }

    clock_gettime(CLOCK_MONOTONIC, &end);

    double post_time = (end.tv_sec - start.tv_sec) +
                       (end.tv_nsec - start.tv_nsec) / 1000000000.0;

    /* Benchmark event processing */
    clock_gettime(CLOCK_MONOTONIC, &start);

    while(process_run() > 0) {
        /* Process events */
    }

    clock_gettime(CLOCK_MONOTONIC, &end);

    double process_time = (end.tv_sec - start.tv_sec) +
                          (end.tv_nsec - start.tv_nsec) / 1000000000.0;

    printf("  Events posted:    %u\n", iterations);
    printf("  Events processed: %u\n", received_count);
    printf("  Post time:        %.6f s\n", post_time);
    printf("  Process time:     %.6f s\n", process_time);
    printf("  Post rate:        %.0f events/sec\n", iterations / post_time);
    printf("  Process rate:     %.0f events/sec\n", received_count / process_time);
    printf("  Avg latency:      %.2f µs/event\n",
           (process_time * 1000000) / received_count);
}

/* Benchmark: Process switching */
static volatile unsigned int yield_count1 = 0;
static volatile unsigned int yield_count2 = 0;
static volatile unsigned int yield_count3 = 0;

PROCESS(yield_process1, "yield1");
PROCESS(yield_process2, "yield2");
PROCESS(yield_process3, "yield3");

PROCESS_THREAD(yield_process1, ev, data) {
    PROCESS_BEGIN();

    static unsigned int iterations = 1000;

    for(unsigned int i = 0; i < iterations; i++) {
        yield_count1++;
        PROCESS_YIELD();
    }

    PROCESS_END();
}

PROCESS_THREAD(yield_process2, ev, data) {
    PROCESS_BEGIN();

    static unsigned int iterations = 1000;

    for(unsigned int i = 0; i < iterations; i++) {
        yield_count2++;
        PROCESS_YIELD();
    }

    PROCESS_END();
}

PROCESS_THREAD(yield_process3, ev, data) {
    PROCESS_BEGIN();

    static unsigned int iterations = 1000;

    for(unsigned int i = 0; i < iterations; i++) {
        yield_count3++;
        PROCESS_YIELD();
    }

    PROCESS_END();
}

void bench_process_switching(void) {
    printf("\n=== Benchmark: Process Switching (C) ===\n");

    process_init();
    clock_init();

    yield_count1 = yield_count2 = yield_count3 = 0;

    process_start(&yield_process1, NULL);
    process_start(&yield_process2, NULL);
    process_start(&yield_process3, NULL);

    /* Process INITs */
    while(process_run() > 0 && (yield_count1 < 1 || yield_count2 < 1 || yield_count3 < 1)) {}

    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while(process_run() > 0) {
        /* Process yields */
    }

    clock_gettime(CLOCK_MONOTONIC, &end);

    double duration = (end.tv_sec - start.tv_sec) +
                      (end.tv_nsec - start.tv_nsec) / 1000000000.0;

    unsigned int total_yields = yield_count1 + yield_count2 + yield_count3;

    printf("  Total yields:     %u\n", total_yields);
    printf("  Duration:         %.6f s\n", duration);
    printf("  Switch rate:      %.0f switches/sec\n", total_yields / duration);
    printf("  Avg switch time:  %.2f µs\n", (duration * 1000000) / total_yields);
}

/* Benchmark: Memory usage */
void bench_memory_usage(void) {
    printf("\n=== Benchmark: Memory Usage (C) ===\n");
    printf("  clock_time_t:     %zu bytes\n", sizeof(clock_time_t));
    printf("  struct pt:        %zu bytes\n", sizeof(struct pt));
    printf("  struct process:   %zu bytes\n", sizeof(struct process));
    printf("  process_event_t:  %zu bytes\n", sizeof(process_event_t));
}

int main(void) {
    printf("=== Contiki-NG C Event Kernel: Benchmark ===\n");

    bench_memory_usage();
    bench_event_posting();
    bench_process_switching();

    printf("\n=== Benchmarks complete ===\n");

    return 0;
}
