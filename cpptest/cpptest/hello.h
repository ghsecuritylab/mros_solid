#ifndef HELLO_H
#define HELLO_H


#include "kernel_cfg.h"
#include <kernel.h>

#include "Main.h"



#ifdef __cplusplus
extern "C"{
#endif

extern void hello_task(intptr_t exinf);

#ifdef __cplusplus
}
#endif
#endif	//HELLO_H