#ifndef _USERAPP_H_
#define _USERAPP_H_


#include "kernel_cfg.h"
#include <kernel.h>

#include "Main.h"
#ifdef __cplusplus
extern "C" {
#endif

void user_task(intptr_t exinf);

#ifdef __cplusplus
}
#endif	/* cplusplus */

#endif	//_USERAPP_H_