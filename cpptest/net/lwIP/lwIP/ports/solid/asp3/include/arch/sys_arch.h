#ifndef _SYS_ARCH_H_
#define	_SYS_ARCH_H_

typedef	void* 	sys_sem_t;
typedef	void* 	sys_mutex_t;
typedef	void* 	sys_mbox_t;
typedef	void* 	sys_thread_t;
typedef	void* 	sys_prot_t;

#define	LWIP_NO_STDINT_H	0

#define	LWIP_SOLID_NO_NEW_TCPIP_THREAD

#endif	/* _SYS_ARCH_H_ */
