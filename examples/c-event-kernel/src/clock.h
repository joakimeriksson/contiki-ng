/**
 * Simplified clock abstraction for benchmarking
 */

#ifndef CLOCK_H
#define CLOCK_H

#include <stdint.h>

typedef uint32_t clock_time_t;

#define CLOCK_SECOND 1000  /* 1ms ticks */

void clock_init(void);
clock_time_t clock_time(void);
void clock_delay_usec(uint16_t us);

#endif /* CLOCK_H */
