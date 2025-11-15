/**
 * Simplified Contiki-NG-style Process System (C Implementation)
 * For benchmarking comparison with Rust implementation
 */

#ifndef PROCESS_H
#define PROCESS_H

#include <stdint.h>
#include <stdbool.h>

/* Process event types */
typedef uint8_t process_event_t;
typedef void *process_data_t;

#define PROCESS_EVENT_NONE            0x80
#define PROCESS_EVENT_INIT            0x81
#define PROCESS_EVENT_POLL            0x82
#define PROCESS_EVENT_EXIT            0x83
#define PROCESS_EVENT_CONTINUE        0x85
#define PROCESS_EVENT_MSG             0x86
#define PROCESS_EVENT_EXITED          0x87
#define PROCESS_EVENT_TIMER           0x88

/* Process states */
#define PROCESS_STATE_NONE      0
#define PROCESS_STATE_RUNNING   1
#define PROCESS_STATE_CALLED    2

/* Protothread definitions */
#define PT_WAITING 0
#define PT_YIELDED 1
#define PT_EXITED  2
#define PT_ENDED   3

struct pt {
    unsigned short lc;
};

#define PT_INIT(pt)   (pt)->lc = 0

#define PT_BEGIN(pt) { unsigned short _pt_line = (pt)->lc; switch(_pt_line) { case 0:

#define PT_END(pt) } _pt_line = 0; (pt)->lc = 0; return PT_ENDED; }

#define PT_WAIT_UNTIL(pt, condition) \
    do { \
        (pt)->lc = __LINE__; case __LINE__: \
        if(!(condition)) return PT_WAITING; \
    } while(0)

#define PT_YIELD(pt) \
    do { \
        (pt)->lc = __LINE__; case __LINE__: \
        return PT_YIELDED; \
    } while(0)

#define PT_EXIT(pt) \
    do { \
        (pt)->lc = 0; \
        return PT_EXITED; \
    } while(0)

/* Process structure */
struct process {
    struct process *next;
    const char *name;
    char (*thread)(struct pt *, process_event_t, process_data_t);
    struct pt pt;
    unsigned char state;
    bool needs_poll;
};

/* Process macros */
#define PROCESS(name, strname) \
    static char process_thread_##name(struct pt *, process_event_t, process_data_t); \
    struct process name = { NULL, strname, process_thread_##name, {0}, 0, false }

#define PROCESS_THREAD(name, ev, data) \
    static char process_thread_##name(struct pt *process_pt, \
                                      process_event_t ev, \
                                      process_data_t data)

#define PROCESS_BEGIN() PT_BEGIN(process_pt)
#define PROCESS_END() PT_END(process_pt)
#define PROCESS_WAIT_EVENT() PT_YIELD(process_pt)
#define PROCESS_WAIT_EVENT_UNTIL(c) PT_WAIT_UNTIL(process_pt, ev == PROCESS_EVENT_POLL || (c))
#define PROCESS_YIELD() PT_YIELD(process_pt)
#define PROCESS_EXIT() PT_EXIT(process_pt)

/* Event queue configuration */
#ifndef PROCESS_CONF_NUMEVENTS
#define PROCESS_CONF_NUMEVENTS 32
#endif

/* Maximum number of processes */
#ifndef MAX_PROCESSES
#define MAX_PROCESSES 16
#endif

/* Process API */
void process_init(void);
void process_start(struct process *p, process_data_t data);
int process_post(struct process *p, process_event_t ev, process_data_t data);
void process_poll(struct process *p);
unsigned char process_run(void);
unsigned char process_nevents(void);
bool process_is_running(struct process *p);

/* Global current process */
extern struct process *process_current;

#endif /* PROCESS_H */
