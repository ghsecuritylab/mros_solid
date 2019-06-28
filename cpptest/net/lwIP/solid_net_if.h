#ifndef _SOLID_NET_IF_H_
#define _SOLID_NET_IF_H_

#include "impl_net_stack.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
*	ネットワークの初期化
*/
void SOLID_NET_IF_Init(void);

/*
*	ネットワーク機能のテストコード
*/
void SOLID_NET_IF_Test(void);


#define	net_strerror(eno)			IMPL_NET_STACK_strerror((eno))

#ifndef	SOLID_NET_IF_SOFT_RX_FIFO_LENGTH
#define	SOLID_NET_IF_SOFT_RX_FIFO_LENGTH	0
#endif	/* SOLID_NET_IF_SOFT_RX_FIFO_LENGTH */

typedef struct {
    void    (*transmitPacket)(pktbuf pb);
    void    (*get_macaddr)(uint8_t macaddr[], size_t len);
} SOLID_NET_IF_ADAPTER;

SOLID_NET_IF_ADAPTER  *SOLID_NET_IF_GetAdapter(const char *name);

#ifdef __cplusplus
}
#endif
#endif	/*  _SOLID_NET_IF_H_ */
