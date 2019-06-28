#ifndef _SYS_ARCH_H_
#define	_SYS_ARCH_H_

#include "solid_mutex.h"

#define	SYS_ARCH_DECL_PROTECT(lev)	unsigned long lev

#define	SYS_ARCH_PROTECT(lev)		lev = SOLID_MUTEX_PushInt()

#define	SYS_ARCH_UNPROTECT(lev)		SOLID_MUTEX_PopInt(lev)	

#endif	/* _SYS_ARCH_H_ */
