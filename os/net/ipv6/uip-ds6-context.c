/**
 * \addtogroup uip6
 * @{
 */

/**
 * \file   Prefix management of 6LoWPAN-ND
 *
 * \author Sebastien Defauw, Laurent Deru
 */
/*
 * Copyright (c) 2015, Cetic
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
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
 */
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include "lib/random.h"
#include "net/ipv6/uip-nd6.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-ds6-context.h"
#include "net/ipv6/uip-packetqueue.h"

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "IPv6 DS CTX"
#define LOG_LEVEL LOG_LEVEL_IPV6

uip_ds6_context_pref_t uip_ds6_context_pref_list[UIP_DS6_CONTEXT_PREF_NB];  /** \brief Prefix list */

/*---------------------------------------------------------------------------*/
void
uip_ds6_context_init(void)
{
  memset(uip_ds6_context_pref_list, 0, sizeof(uip_ds6_context_pref_list));
}

/*---------------------------------------------------------------------------*/
uip_ds6_context_pref_t *
uip_ds6_context_pref_add(uip_ipaddr_t *ipaddr, uint8_t length, uint8_t c_cid)
{
  uip_ds6_context_pref_t *loccontext;
  loccontext = &uip_ds6_context_pref_list[(c_cid & UIP_ND6_6CO_FLAG_CID)];
  if(loccontext->state == CONTEXT_PREF_ST_FREE) {
    loccontext->state = (c_cid & UIP_ND6_6CO_FLAG_C) ?
      CONTEXT_PREF_ST_COMPRESS : CONTEXT_PREF_ST_UNCOMPRESSONLY;
    uip_ipaddr_copy(&loccontext->ipaddr, ipaddr);
    loccontext->length = length;
    loccontext->cid = c_cid & UIP_ND6_6CO_FLAG_CID;
    LOG_INFO("Adding context prefix ");
    LOG_INFO_6ADDR(&loccontext->ipaddr);
    LOG_INFO_(" length %u, c %x, cid %x\n",
              length, c_cid & 0x10, c_cid & 0x0f);
    return loccontext;
  } else {
    LOG_WARN("No more space in Context Prefix list\n");
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
void
uip_ds6_context_pref_rm(uip_ds6_context_pref_t *prefix)
{
  if(prefix != NULL && prefix->state != CONTEXT_PREF_ST_FREE) {
    prefix->state = CONTEXT_PREF_ST_FREE;
  }
  return;
}
/*---------------------------------------------------------------------------*/
void
uip_ds6_context_pref_rm_all(void)
{
  uip_ds6_context_pref_t *loccontext;
  for(loccontext = uip_ds6_context_pref_list;
      loccontext < uip_ds6_context_pref_list + UIP_DS6_CONTEXT_PREF_NB;
      loccontext++) {
    uip_ds6_context_pref_rm(loccontext);
  }
}
/*---------------------------------------------------------------------------*/
uip_ds6_context_pref_t *
uip_ds6_context_pref_lookup(uip_ipaddr_t *ipaddr)
{
  uip_ds6_context_pref_t *loccontext;
  for(loccontext = uip_ds6_context_pref_list;
      loccontext < uip_ds6_context_pref_list + UIP_DS6_CONTEXT_PREF_NB;
      loccontext++) {
    if(loccontext->state != CONTEXT_PREF_ST_FREE &&
       uip_ipaddr_prefixcmp(ipaddr, &loccontext->ipaddr, loccontext->length)) {
      return loccontext;
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
uip_ds6_context_pref_t *
uip_ds6_context_pref_lookup_by_cid(uint8_t cid)
{
  uip_ds6_context_pref_t *loccontext;
  if(cid >= UIP_DS6_CONTEXT_PREF_NB) {
    return NULL;
  }
  loccontext = &uip_ds6_context_pref_list[cid];
  return loccontext->state == CONTEXT_PREF_ST_FREE ? NULL : loccontext;
}

/** @}*/
