/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Initialiation file for the Contiki low-layer network stack (NETSTACK)
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "net/netstack.h"
#include "lib/list.h"
#include <stdio.h>

/* The list of IP processors that will process IP packets before uip or after */
LIST(ip_processor_list);

/* Note: localdest is only used for the output callback */
enum netstack_ip_action
netstack_process_ip_callback(uint8_t type, const linkaddr_t *localdest)
{
  enum netstack_ip_action action = NETSTACK_IP_PROCESS;
  struct netstack_ip_packet_processor *p;
  int processor_count = 0;

  printf("[NETSTACK] netstack_process_ip_callback() called, type=%d\n", type);
  fflush(stdout);

  for(p = list_head(ip_processor_list);
      p != NULL;
      p = list_item_next(p)) {
    processor_count++;
    printf("[NETSTACK] Processing IP processor #%d at %p\n", processor_count, (void*)p);
    fflush(stdout);

    if(type == NETSTACK_IP_OUTPUT) {
      if(p->process_output != NULL) {
        printf("[NETSTACK] Calling process_output() at %p\n", (void*)p->process_output);
        fflush(stdout);
        action = p->process_output(localdest);
        printf("[NETSTACK] process_output() returned action=%d\n", action);
        fflush(stdout);
      } else {
        printf("[NETSTACK] process_output is NULL\n");
        fflush(stdout);
      }
    } else if(type == NETSTACK_IP_INPUT) {
      if(p->process_input != NULL) {
        printf("[NETSTACK] Calling process_input() at %p\n", (void*)p->process_input);
        fflush(stdout);
        action = p->process_input();
        printf("[NETSTACK] process_input() returned action=%d\n", action);
        fflush(stdout);
      } else {
        printf("[NETSTACK] process_input is NULL\n");
        fflush(stdout);
      }
    }
    /* if not NETSTACK_IP_PROCESS - quit and return the desired action */
    if(action != NETSTACK_IP_PROCESS) {
      printf("[NETSTACK] Action is not PROCESS (%d), returning early\n", action);
      fflush(stdout);
      return action;
    }
  }

  printf("[NETSTACK] Finished processing %d IP processors, returning action=%d\n", processor_count, action);
  fflush(stdout);
  return action;
}
/*---------------------------------------------------------------------------*/
void
netstack_ip_packet_processor_add(struct netstack_ip_packet_processor *p)
{
  if(p != NULL) {
    list_add(ip_processor_list, p);
  }
}
/*---------------------------------------------------------------------------*/
void
uip_ds6_ip_packet_processor_rm(struct netstack_ip_packet_processor *p)
                               {
  list_remove(ip_processor_list, p);
}
/*---------------------------------------------------------------------------*/
