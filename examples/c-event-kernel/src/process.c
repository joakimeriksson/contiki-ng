/**
 * Simplified Contiki-NG-style Process System Implementation
 */

#include "process.h"
#include <string.h>

/* Event data structure */
struct event_data {
    process_data_t data;
    struct process *p;
    process_event_t ev;
};

/* Event queue */
static struct event_data events[PROCESS_CONF_NUMEVENTS];
static unsigned char nevents, fevent;

/* Process list */
static struct process *process_list = NULL;

/* Current process */
struct process *process_current = NULL;

/* Initialize the process system */
void process_init(void) {
    nevents = fevent = 0;
    process_list = NULL;
    process_current = NULL;
    memset(events, 0, sizeof(events));
}

/* Start a process */
void process_start(struct process *p, process_data_t data) {
    if(p == NULL) return;

    /* Initialize protothread */
    PT_INIT(&p->pt);

    /* Set state */
    p->state = PROCESS_STATE_RUNNING;
    p->needs_poll = false;

    /* Add to process list */
    p->next = process_list;
    process_list = p;

    /* Post INIT event */
    process_post(p, PROCESS_EVENT_INIT, data);
}

/* Post an event to a process */
int process_post(struct process *p, process_event_t ev, process_data_t data) {
    unsigned char snum;

    if(p == NULL || p->state == PROCESS_STATE_NONE) {
        return -1;
    }

    /* Check if queue is full */
    snum = (fevent + nevents) % PROCESS_CONF_NUMEVENTS;
    if(nevents == PROCESS_CONF_NUMEVENTS) {
        return -1; /* Queue full */
    }

    /* Add event to queue */
    events[snum].ev = ev;
    events[snum].data = data;
    events[snum].p = p;

    nevents++;

    return 0;
}

/* Request polling of a process */
void process_poll(struct process *p) {
    if(p != NULL && p->state == PROCESS_STATE_RUNNING) {
        p->needs_poll = true;
    }
}

/* Execute a process */
static void do_event(void) {
    struct event_data *ev;
    struct process *p;

    /* Get next event from queue */
    if(nevents == 0) {
        return;
    }

    ev = &events[fevent];

    p = ev->p;

    if(p == NULL || p->state != PROCESS_STATE_RUNNING) {
        nevents--;
        fevent = (fevent + 1) % PROCESS_CONF_NUMEVENTS;
        return;
    }

    /* Execute process */
    process_current = p;
    p->state = PROCESS_STATE_CALLED;

    char ret = p->thread(&p->pt, ev->ev, ev->data);

    if(ret == PT_EXITED || ret == PT_ENDED) {
        p->state = PROCESS_STATE_NONE;

        /* Post EXITED event to all processes */
        struct process *q;
        for(q = process_list; q != NULL; q = q->next) {
            process_post(q, PROCESS_EVENT_EXITED, p);
        }
    } else {
        p->state = PROCESS_STATE_RUNNING;
    }

    process_current = NULL;

    /* Remove event from queue */
    nevents--;
    fevent = (fevent + 1) % PROCESS_CONF_NUMEVENTS;
}

/* Handle poll requests */
static void do_poll(void) {
    struct process *p;

    for(p = process_list; p != NULL; p = p->next) {
        if(p->needs_poll && p->state == PROCESS_STATE_RUNNING) {
            p->needs_poll = false;

            process_current = p;
            p->state = PROCESS_STATE_CALLED;

            char ret = p->thread(&p->pt, PROCESS_EVENT_POLL, NULL);

            if(ret == PT_EXITED || ret == PT_ENDED) {
                p->state = PROCESS_STATE_NONE;
            } else {
                p->state = PROCESS_STATE_RUNNING;
            }

            process_current = NULL;
        }
    }
}

/* Run one iteration of the event loop */
unsigned char process_run(void) {
    /* First handle poll requests */
    do_poll();

    /* Then process one event */
    do_event();

    return nevents;
}

/* Get number of pending events */
unsigned char process_nevents(void) {
    return nevents;
}

/* Check if process is running */
bool process_is_running(struct process *p) {
    return p != NULL && p->state != PROCESS_STATE_NONE;
}
