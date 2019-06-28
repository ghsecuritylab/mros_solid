#include	<stdint.h>
#include 	"lwip/sys.h"
#include 	"lwip/tcpip.h"

#include	"solid_cs_assert.h"

#include	<kernel.h>
#include 	<solid_log.h>
#include	<kernel/dataqueue.h>
#include	<syslog.h>

#include	"solid_timer.h"

#define		MSEC2USEC(msec)				((msec)*1000)

#define REFER_WEAK_VAR(type, name)\
        extern type __attribute__((weak)) __ ## name ## __;

REFER_WEAK_VAR(int, NET_SYS_EVFLG_ID_BASE);
REFER_WEAK_VAR(int, NET_SYS_MUTEX_ID_BASE);
REFER_WEAK_VAR(int, NET_SYS_SEM_ID_BASE);
REFER_WEAK_VAR(int, NET_SYS_DTQ_ID_BASE);
REFER_WEAK_VAR(int, NET_SYS_TSK_ID_BASE);

#define IS_AVAILABLE(weak_var)       (&weak_var)

#define DECLARE_VAR(type, name) \
        static type name;

static int net_sys_evflg_id_base = 1;
static int net_sys_mutex_id_base = 1;
static int net_sys_sem_id_base = 1;
static int net_sys_dtq_id_base = 1;
static int net_sys_tsk_id_base = 1;

#define		NET_SYS_MAX_EVFLG			5
#define		NET_SYS_EVFLG_ID_BASE		net_sys_evflg_id_base
#define		NET_SYS_MAX_MUTEX			5
#define		NET_SYS_MUTEX_ID_BASE		net_sys_mutex_id_base
#define		NET_SYS_MAX_SEM				5
#define		NET_SYS_SEM_ID_BASE			net_sys_sem_id_base
#define		NET_SYS_MAX_DTQ				5
#define		NET_SYS_DTQ_ID_BASE			net_sys_dtq_id_base
#define		NET_SYS_MAX_TSK				5
#define		NET_SYS_TSK_ID_BASE			net_sys_tsk_id_base

// aliases.
#define		NET_SYS_MAX_MBOX				NET_SYS_MAX_DTQ
#define		NET_SYS_MBOX_ID_BASE			NET_SYS_DTQ_ID_BASE

#define		NET_SYS_TSK_ID_MAX			(NET_SYS_TSK_ID_BASE + NET_SYS_MAX_TSK -1)
#define		NET_SYS_IS_VALID_TSKID(tid)	(((tid) >= NET_SYS_TSK_ID_BASE) && ((tid) <= NET_SYS_TSK_ID_MAX))

#if LWIP_COMPAT_MUTEX
typedef struct {
	ID	mutex_id;
	int	in_use;	/* true or false */
} arch_mutex_t;

#define	MTX_ID(am)				((am)?am->mutex_id:0)
#define	MTX_IS_IN_USE(am)		((am)&&(am->in_use != 0))

arch_mutex_t	sys_mutex_table[NET_SYS_MAX_MUTEX];
#endif /* LWIP_COMPAT_MUTEX */

static	ID		global_protection_mutex = -1;
static	ID		global_io_event_flag = -1;
static	ID		global_daemon_tskid = -1;

typedef struct {
	ID	semaphore_id;
	int	in_use;	/* true or false */
} arch_semaphore_t;

#define	SEM_ID(am)				((am)?am->semaphore_id:0)
#define	SEM_IS_IN_USE(am)		((am)&&(am->in_use != 0))

arch_semaphore_t	sys_semaphore_table[NET_SYS_MAX_SEM];

typedef struct {
	ID	dataqueue_id;
	int	in_use;	/* true or false */
} arch_mbox_t;

#define	DTQ_ID(am)				((am)?am->dataqueue_id:0)
#define	DTQ_IS_IN_USE(am)		((am)&&(am->in_use != 0))

arch_mbox_t	sys_mbox_table[NET_SYS_MAX_DTQ];

typedef struct {
	const char		*name;
	lwip_thread_fn	thread_func;
	void			*thread_arg;
} lwip_thread_args;

typedef struct {
	ID					tskid;
	lwip_thread_args	args;
} lwip_threads_entry;

static	lwip_threads_entry	threads_entry[NET_SYS_MAX_TSK];

void sys_init(void)
{
	/*
	*	タスクコンテキストで呼ばれるのでOSリソースの初期化はできない
	*/

}

void sys_setup_OSresource(void)
{
	/*
	*	タスクディスパッチ前に呼び出される
	*/

	int		i;

        /*
         * カーネル資源の定義情報から内部の初期化のみ行う
         */
        #define SYNC_VAR(weak_var, var)  if (IS_AVAILABLE(weak_var)) var = weak_var

        SYNC_VAR(__NET_SYS_EVFLG_ID_BASE__, net_sys_evflg_id_base);
        SYNC_VAR(__NET_SYS_MUTEX_ID_BASE__, net_sys_mutex_id_base);
        SYNC_VAR(__NET_SYS_SEM_ID_BASE__, net_sys_sem_id_base);
        SYNC_VAR(__NET_SYS_DTQ_ID_BASE__, net_sys_dtq_id_base);
        SYNC_VAR(__NET_SYS_TSK_ID_BASE__, net_sys_tsk_id_base);

#if LWIP_COMPAT_MUTEX
	global_protection_mutex = NET_SYS_SEM_ID_BASE;

	for ( i= 0; i < NET_SYS_MAX_SEM; i++ ) {
		sys_semaphore_table[i].semaphore_id = (ID)(i + NET_SYS_SEM_ID_BASE + 1);
		sys_semaphore_table[i].in_use = 0;
	}

#else /* !LWIP_COMPAT_MUTEX */
	global_protection_mutex = NET_SYS_MUTEX_ID_BASE;

	for ( i= 0; i < NET_SYS_MAX_MUTEX; i++ ) {
		sys_mutex_table[i].mutex_id = (ID)(i + NET_SYS_MUTEX_ID_BASE + 1);
		sys_mutex_table[i].in_use = 0;
	}

	for ( i= 0; i < NET_SYS_MAX_SEM; i++ ) {
		sys_semaphore_table[i].semaphore_id = (ID)(i + NET_SYS_SEM_ID_BASE);
		sys_semaphore_table[i].in_use = 0;
	}

#endif /* LWIP_COMPAT_MUTEX */

	for ( i= 0; i < NET_SYS_MAX_MBOX; i++ ) {
		sys_mbox_table[i].dataqueue_id = (ID)(i + NET_SYS_MBOX_ID_BASE);
		sys_mbox_table[i].in_use = 0;
	}

	for ( i = 0; i < NET_SYS_MAX_TSK; i++) {
		threads_entry[i].tskid = NET_SYS_TSK_ID_BASE + i;
		threads_entry[i].args.name = NULL;
		threads_entry[i].args.thread_func = NULL;
		threads_entry[i].args.thread_arg = NULL;
	}

	global_io_event_flag = NET_SYS_EVFLG_ID_BASE;
}

typedef struct {
	tcpip_init_done_fn	fn_done;
	void				*arg;
} _tcpip_init_args;

static _tcpip_init_args tcpip_init_args;

void lwip_daemon_thread(void *arg);

void sys_start_daemon(void (*fn)(void*), void* arg)
{
	tcpip_init_args.fn_done = fn;
	tcpip_init_args.arg = arg;
	sys_thread_new("TCP/IP daemon thread", lwip_daemon_thread, &tcpip_init_args, 0, 0);
}

#ifndef LWIP_COMPAT_MUTEX
err_t sys_mutex_new(sys_mutex_t *mutex)
{
	int		i;
	arch_mutex_t	*tb = sys_mutex_table;

	for ( i= 0; i < NET_SYS_MAX_MUTEX; i++,tb++ ) {
		if ( !MTX_IS_IN_USE(tb)) {
			tb->in_use = 1;
			*mutex = tb;
			break;
		}
	}

	if ( i < NET_SYS_MAX_MUTEX) {
		return	ERR_OK;
	} else {
		return	ERR_MEM;
	}
}

void sys_mutex_lock(sys_mutex_t *mutex)
{
	ER	rErr;
	arch_mutex_t	*tb = (arch_mutex_t*)*mutex;

	solid_cs_assert(MTX_IS_IN_USE(tb));
	rErr = loc_mtx(MTX_ID(tb));
	solid_cs_assert(rErr == E_OK);
}


void sys_mutex_unlock(sys_mutex_t *mutex)
{
	ER 	rErr;
	arch_mutex_t	*tb = (arch_mutex_t*)*mutex;

	solid_cs_assert(MTX_IS_IN_USE(tb));
	rErr = unl_mtx(MTX_ID(tb));
	solid_cs_assert(rErr == E_OK);
}

void sys_mutex_free(sys_mutex_t *mutex)
{
	arch_mutex_t	*tb = (arch_mutex_t*)*mutex;

	tb->in_use = 0;
}

int sys_mutex_valid(sys_mutex_t *mutex)
{
	arch_mutex_t	*tb = (arch_mutex_t*)*mutex;
	return ((MTX_ID(tb) !=0) && MTX_IS_IN_USE(tb))?1:0;
}

void sys_mutex_set_invalid(sys_mutex_t *mutex)
{
	*mutex = (sys_mutex_t)0;
}
#endif /* LWIP_COMPAT_MUTEX */

sys_prot_t sys_arch_protect(void)
{
	loc_cpu();
	return &global_protection_mutex;
}

void sys_arch_unprotect(sys_prot_t pval)
{
	unl_cpu();
}

err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
	int		i;
	arch_semaphore_t	*tb = sys_semaphore_table;

	solid_cs_assert( count <= 1 );

	for ( i= 0; i < NET_SYS_MAX_SEM; i++,tb++ ) {
		if ( !SEM_IS_IN_USE(tb)) {
			tb->in_use = 1;
			*sem = tb;
			break;
		}
	}

	if ( i < NET_SYS_MAX_SEM) {
		while (count-- > 0 ) {
			ER	rErr;
			rErr = sig_sem(SEM_ID(tb));
			solid_cs_assert( rErr == E_OK );
		}
		return	ERR_OK;
	} else {
		return	ERR_MEM;
	}
}

void sys_sem_signal(sys_sem_t *sem)
{
	ER	rErr;
	arch_semaphore_t	*tb = (arch_semaphore_t*)*sem;

	solid_cs_assert(SEM_IS_IN_USE(tb));
	rErr = sig_sem(SEM_ID(tb));
	solid_cs_assert( rErr == E_OK );
}

u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
	arch_semaphore_t	*tb = (arch_semaphore_t*)*sem;
	u32_t				ts,te;
	ER					rErr;
	solid_cs_assert(SEM_IS_IN_USE(tb));
	ts = sys_now();

	if (timeout) {
		rErr = twai_sem(SEM_ID(tb), MSEC2USEC(timeout));
	} else {
		rErr = wai_sem(SEM_ID(tb));
	}

	if (rErr == E_OK ) {
		u32_t			t_elapsed;
		te = sys_now();
		t_elapsed = (te > ts)? (te - ts ): (0xffffffff - ts + te);
		return t_elapsed;
	} else {
		return SYS_ARCH_TIMEOUT;
	}
}

void sys_sem_free(sys_sem_t *sem)
{
	arch_semaphore_t	*tb = (arch_semaphore_t*)*sem;

	tb->in_use = 0;
}

int sys_sem_valid(sys_sem_t *sem)
{
	arch_semaphore_t	*tb = (arch_semaphore_t*)*sem;
	return ((SEM_ID(tb) !=0) && SEM_IS_IN_USE(tb))?1:0;
}

void sys_sem_set_invalid(sys_sem_t *sem)
{
	*sem = (sys_sem_t*)0;
}

//#define	SYS_ARCH_MBOX_DEBUG

#ifdef	SYS_ARCH_MBOX_DEBUG

//#define	SYS_ARCH_MBOX_DEBUG_HALT_AT_FULL

#define	ID_FILTER(id)				((id)==1)

#define	MAX_DTQCB_LOG				64
#define INDEX_DTQ(dtqid)    		((uint_t)((dtqid) - TMIN_DTQID))
#define	get_dtqcb_byID(dtqid)		(&(dtqcb_table[INDEX_DTQ(dtqid)]))
#define	get_dtqcb(mboxid)			get_dtqcb_byID(DTQ_ID(mboxid))

typedef struct _dtqcb_logrec {
	ID				dtqid;
	const	char*	func;
	int				line;
	DTQCB			dtqcb;
	void			*msgadr;
	u32_t			timeout;
} dtqcb_logrec;

static dtqcb_logrec _dtqcbLog[MAX_DTQCB_LOG];
static int	_dtcqb_log_w_pos = 0;

static	__inline__ void logDTQCB(ID dtqid, const char *func, int line, const DTQCB* dtqcb, void *msgadr, u32_t timeout)
{
	if (!ID_FILTER(dtqid))		return;
	dtqcb_logrec*	rec = &_dtqcbLog[_dtcqb_log_w_pos++];

	rec->dtqid = dtqid;
	rec->func = func;
	rec->line = line;
	rec->dtqcb = *dtqcb;
	rec->msgadr = msgadr;
	rec->timeout = timeout;
#ifdef	SYS_ARCH_MBOX_DEBUG_HALT_AT_FULL
	solid_cs_assert( _dtcqb_log_w_pos < MAX_DTQCB_LOG );
#else	/* !SYS_ARCH_MBOX_DEBUG_HALT_AT_FULL */
	_dtcqb_log_w_pos %= MAX_DTQCB_LOG;
#endif	/* SYS_ARCH_MBOX_DEBUG_HALT_AT_FULL */
}

#define	LOG_DTQCB(mbox, msgadr, timeout)		logDTQCB(DTQ_ID(mbox), __func__, __LINE__, get_dtqcb(mbox),(msgadr),(timeout))
#else /* !SYS_ARCH_MBOX_DEBUG */
#define	LOG_DTQCB(mbox, msgadr, timeout)
#endif	/* SYS_ARCH_MBOX_DEBUG */

err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
	int		i;

	LWIP_UNUSED_ARG(size);

	arch_mbox_t	*tb = sys_mbox_table;

	for ( i= 0; i < NET_SYS_MAX_DTQ; i++,tb++ ) {
		if ( !DTQ_IS_IN_USE(tb)) {
			tb->in_use = 1;
			*mbox = tb;
			break;
		}
	}

	if ( i < NET_SYS_MAX_DTQ) {
		return	ERR_OK;
	} else {
		return	ERR_MEM;
	}
}

void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
	ER	rErr;
	arch_mbox_t	*tb = (arch_mbox_t*)*mbox;

	solid_cs_assert(DTQ_IS_IN_USE(tb));
	rErr = snd_dtq(DTQ_ID(tb), (intptr_t)msg);
	LOG_DTQCB(tb,msg, 0xffffffff);
	solid_cs_assert( rErr == E_OK );
}

err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
	ER		rErr;
	arch_mbox_t	*tb = (arch_mbox_t*)*mbox;

	solid_cs_assert(DTQ_IS_IN_USE(tb));
	rErr = psnd_dtq(DTQ_ID(tb), (intptr_t)msg);
	LOG_DTQCB(tb,msg,0);
	solid_cs_assert( rErr == E_OK );
	if (rErr == E_OK ) {
		return ERR_OK;
	} else {
		return ERR_MEM;
	}
}

u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
	ER		rErr;
	arch_mbox_t	*tb = (arch_mbox_t*)*mbox;
	u32_t				ts,te;

	solid_cs_assert(DTQ_IS_IN_USE(tb));
	ts = sys_now();
	if (timeout ) {
		rErr = trcv_dtq(DTQ_ID(tb), (intptr_t*)msg, MSEC2USEC(timeout));
		LOG_DTQCB(tb,*msg,timeout);
	} else {
		rErr = rcv_dtq(DTQ_ID(tb), (intptr_t*)msg);
		LOG_DTQCB(tb,*msg,timeout);
	}
	if ( rErr == E_OK ) {
		u32_t			t_elapsed;
		te = sys_now();
		t_elapsed = (te >= ts)? (te - ts ): (0xffffffff - ts + te);
		solid_cs_assert(t_elapsed != SYS_ARCH_TIMEOUT);
		return t_elapsed;
	} else {
		LOG_DTQCB(tb,(void*)rErr, timeout);
		return SYS_ARCH_TIMEOUT;
	}
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
	ER		rErr;
	arch_mbox_t	*tb = (arch_mbox_t*)*mbox;

	solid_cs_assert(DTQ_IS_IN_USE(tb));
	rErr = prcv_dtq(DTQ_ID(tb), (intptr_t*)msg);
	LOG_DTQCB(tb,*msg,0);
	if ( rErr == E_OK ) {
		return	0; /* msec */
	} else {
		return SYS_MBOX_EMPTY;
	}
}

void sys_mbox_free(sys_mbox_t *mbox)
{
	arch_mbox_t	*tb = (arch_mbox_t*)*mbox;

	tb->in_use = 0;
}

int sys_mbox_valid(sys_mbox_t *mbox)
{
	arch_mbox_t	*tb = (arch_mbox_t*)*mbox;
	return ((DTQ_ID(tb) !=0) && DTQ_IS_IN_USE(tb))?1:0;
}

void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
	*mbox = (sys_mbox_t*)0;
}

u32_t sys_now(void)
{
	/*
	*	TOPPERS(uITRON)のget_timはタスクコンテキストからしか呼び出せないため、
	*   Core ServiceのTimerを直接参照する
	*/
    return (u32_t)(0xffffffff & (( SOLID_TIMER_ToUsec(SOLID_TIMER_GetCurrentTick()) ) /1000 ));
}

sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread, void *arg, int stacksize, int prio)
{
	ID	tskid = NET_SYS_TSK_ID_BASE - 1;
	lwip_threads_entry	*ent = threads_entry;

	LWIP_UNUSED_ARG(stacksize);
	LWIP_UNUSED_ARG(prio);

	loc_cpu();{
		int		i;

		/* 空いているタスクを探す　*/
		for ( i = 0; i < NET_SYS_MAX_TSK; i++, ent++) {
			if ( ent->args.thread_func == NULL ) {
				tskid = ent->tskid;
				break;
			}
		}
		solid_cs_assert( i < NET_SYS_MAX_TSK);

		ent->args.name = name;
		ent->args.thread_func = thread;
		ent->args.thread_arg = arg;

	} unl_cpu();

	if (NET_SYS_IS_VALID_TSKID(tskid)) act_tsk(tskid);

	return (sys_thread_t)tskid;	/* don't look back. */
}

void lwip_thread_task(intptr_t exinf)
{
	lwip_thread_args	*args;
	ID					myID;
	ER					rErr;
	LWIP_UNUSED_ARG(exinf);	/* 引数は静的なパラメータのみなので使えない */

	rErr = get_tid(&myID);
	solid_cs_assert(rErr == E_OK);
	solid_cs_assert((myID >= NET_SYS_TSK_ID_BASE) && (myID < NET_SYS_TSK_ID_MAX));

	args = &threads_entry[myID - NET_SYS_TSK_ID_BASE].args;

	solid_cs_assert(args->thread_func);
	solid_cs_assert(args->name);

	syslog(LOG_INFO, "[Net:%s] started: Task ID:%d",args->name, myID);

	args->thread_func(args->thread_arg); /* LWIPのworker threadを実行 */

	solid_cs_assert(0);	/* こんなところに来てはいけない */
}

void lwip_daemon_thread(void *arg)
{
	ER	rErr;
	_tcpip_init_args		*pa = (_tcpip_init_args*)arg;

	rErr = get_tid(&global_daemon_tskid);
	solid_cs_assert(rErr == E_OK);
	solid_cs_assert((global_daemon_tskid >= NET_SYS_TSK_ID_BASE) && (global_daemon_tskid < NET_SYS_TSK_ID_MAX));

	/*
	*	lwIPの初期化関数を呼び出す。
	*	オリジナルのtcpip_initは内部でsys_thread_newを呼び出して
	*   別スレッドがメインループを実行するが、この実装では
	*   別スレッドを起こすことなく、直接関数内部でメインループを
    *   実行する様に改造されたtcpip_initの実装・動作を期待している。
	*/
#ifndef LWIP_SOLID_NO_NEW_TCPIP_THREAD
	#error	LWIP_SOLID_NO_NEW_TCPIP_THREAD must be defined here.
#endif /* LWIP_SOLID_NO_NEW_TCPIP_THREAD */
	tcpip_init(pa->fn_done, pa->arg);

	solid_cs_assert(0);	/* こんなところに来てはいけない */
	return;
}

/*
 *
 * 使用するカーネル資源の定義
 *
 */

#if 0
/*
 * ドキュメント用
 *
 */

_kernel_.* -> _kext_net_.*

#if USE_TASAN
#define NET_STACK_SIZE              8192
#else
#define NET_STACK_SIZE              2048
#endif

const TINIB _kext_net_tinib_table[] =
{
        {(TA_NULL),(intptr_t)101,(TASK)lwip_thread_task,INT_PRIORITY(HIGH_PRIORITY), ROUND_STK_T(NET_STACK_SIZE), NULL },
        {(TA_NULL),(intptr_t)102,(TASK)lwip_thread_task,INT_PRIORITY(HIGH_PRIORITY), ROUND_STK_T(NET_STACK_SIZE), NULL },
        {(TA_NULL),(intptr_t)103,(TASK)lwip_thread_task,INT_PRIORITY(HIGH_PRIORITY), ROUND_STK_T(NET_STACK_SIZE), NULL },
        {(TA_NULL),(intptr_t)104,(TASK)lwip_thread_task,INT_PRIORITY(HIGH_PRIORITY), ROUND_STK_T(NET_STACK_SIZE), NULL },
        {(TA_NULL),(intptr_t)105,(TASK)lwip_thread_task,INT_PRIORITY(HIGH_PRIORITY), ROUND_STK_T(NET_STACK_SIZE), NULL },
};

const uint32_t  _kext_net_tnum_tinib = NUM_OF(_kext_net_tinib_table, TINIB);

/*
 *  起動順位に注意！
 */
const ID _kext_net_torder_table[] = {
        TASK_RESERVED_NET1,
        TASK_RESERVED_NET2,
        TASK_RESERVED_NET3,
        TASK_RESERVED_NET4,
        TASK_RESERVED_NET5,
};

const uint32_t  _kext_net_tnum_torder = NUM_OF(_kext_net_torder_table, ID);


const SEMINIB _kext_net_seminib_table[] = {
        {TA_NULL, 0, 1},
        {TA_NULL, 0, 1},
        {TA_NULL, 0, 1},
        {TA_NULL, 0, 1},
        {TA_NULL, 0, 1},
};

/* flgは未使用 */

#define     MAX_NET_QUEUE       32  /* workaround */

const DTQINIB _kext_net_dtqinib_table[] = {
        {TA_NULL, MAX_NET_QUEUE, NULL},
        {TA_NULL, MAX_NET_QUEUE, NULL},
        {TA_NULL, MAX_NET_QUEUE, NULL},
        {TA_NULL, MAX_NET_QUEUE, NULL},
        {TA_NULL, MAX_NET_QUEUE, NULL},
 };

const MTXINIB _kext_net_mtxinib_table[] = {
        {TA_NULL, INT_PRIORITY(0) },
        {TA_NULL, INT_PRIORITY(0) },
        {TA_NULL, INT_PRIORITY(0) },
        {TA_NULL, INT_PRIORITY(0) },
        {TA_NULL, INT_PRIORITY(0) },
};
#endif
