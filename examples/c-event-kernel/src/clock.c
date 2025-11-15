/**
 * POSIX clock implementation for benchmarking
 */

#define _POSIX_C_SOURCE 199309L

#include "clock.h"
#include <time.h>
#include <unistd.h>

static struct timespec start_time;

void clock_init(void) {
    clock_gettime(CLOCK_MONOTONIC, &start_time);
}

clock_time_t clock_time(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    time_t sec = now.tv_sec - start_time.tv_sec;
    long nsec = now.tv_nsec - start_time.tv_nsec;

    return (clock_time_t)(sec * 1000 + nsec / 1000000);
}

void clock_delay_usec(uint16_t us) {
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = us * 1000L;
    nanosleep(&ts, NULL);
}
