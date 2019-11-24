/*
 * Copyright (c) 2015, SICS Swedish ICT.
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
 */

/**
 * \author Simon Duquennoy <simonduq@sics.se>
 */

#ifndef __PROJECT_CONF_H__
#define __PROJECT_CONF_H__

#undef ENERGEST_CONF_ON
#define ENERGEST_CONF_ON (0)

#undef RF_CORE_CONF_CHANNEL
#define RF_CORE_CONF_CHANNEL 13

#undef LLSEC802154_CONF_ENABLED
#define LLSEC802154_CONF_ENABLED (1)

#undef NETSTACK_CONF_LLSEC
#define NETSTACK_CONF_LLSEC noncoresec_driver

#define NONCORESEC_CONF_SEC_LVL (7)

#undef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER noncoresec_framer

#undef NONCORESEC_CONF_KEY
#define NONCORESEC_CONF_KEY { 0x9D , 0x1B , 0xFF , 0x00 , \
                            0xFE , 0xF0 , 0xF8 , 0xA5 , \
                            0x06 , 0xB1 , 0x9E , 0xF4 , \
                            0xA0 , 0xB0 , 0xBD , 0x4C }

#endif /* __PROJECT_CONF_H__ */
