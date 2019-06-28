/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#ifndef __CC_H__
#define __CC_H__

#include <stdlib.h>

//typedef unsigned   char    u8_t;
//typedef signed     char    s8_t;
//typedef unsigned   short   u16_t;
//typedef signed     short   s16_t;
//typedef unsigned   long    u32_t;
//typedef signed     long    s32_t;

#ifndef U8_F
#define U8_F "u"
#endif
#ifndef X8_F
#define X8_F "x"
#endif
#define U16_F "hu"
#define S16_F "hd"
#define X16_F "hx"
#define U32_F "lu"
#define S32_F "ld"
#define X32_F "lx"
#ifndef	SZT_F
#define	SZT_F "p"
#endif

#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_STRUCT
#define PACK_STRUCT_END
#define PACK_STRUCT_FIELD(x) x

#define	LWIP_NO_INTTYPES_H		1
#define	LWIP_NO_STDINT_H		0

#ifndef BYTE_ORDER
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define BYTE_ORDER LITTLE_ENDIAN
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
#define BYTE_ORDER BIG_ENDIAN
#else
/* Oops, there must be something wrong unless you've got a PDP-11. */
#error  predefined macro __BYTE_ORDER__ must be __ORDER_LITTLE_ENDIAN__ or __ORDER_BIG_ENDIAN__ \
        This software doesn't support PDP endian.
#endif


#endif

#define LWIP_RAND() ((u32_t)rand())

#if defined(SOLID_NDEBUG) || defined(NDEBUG)
#define	LWIP_PLATFORM_ASSERT(x)
#else /* !(defined(SOLID_NDEBUG) || defined(NDEBUG)) */
#define	LWIP_PLATFORM_ASSERT(x)
#if 0
#include "solid_cs_assert.h"
#define	LWIP_PLATFORM_ASSERT(x)		((x)?0:solid_cs_abort())
#endif
#endif /* (defined(SOLID_NDEBUG) || defined(NDEBUG)) */

#define	 sio_open(devnum)				(NULL)
#define	 sio_send(c, fd)
#define	 sio_recv(fd)
#define	 sio_read(fd, data, len)		(-1)
#define	 sio_tryread(fd, data, len)		(-1)
#define	 sio_write(fd, data, len)		(-1)
#define	 sio_read_abort(fd)

#include "solid_log.h"
#define	LWIP_PLATFORM_DIAG(msg)			do { SOLID_LOG_printf msg;} while(0)

#ifndef errno
extern int _net_sys_errno;
#define errno _net_sys_errno
#endif

#endif /* __CC_H__ */

