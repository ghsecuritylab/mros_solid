#ifndef _MROS_H
#define _MROS_H

#include "kernel_cfg.h"
#include <kernel.h>

#include "Main.h"

#ifdef __cplusplus
extern "C" {
#endif
extern char mem[1024 * 32];		//とりあえず300KB
extern void xml_mas_task(intptr_t exinf);

#ifdef __cplusplus
}
#endif
#endif //_MROS_H_