/* kernel_cfg.c */
#include "kernel/kernel_int.h"
#include "kernel_cfg.h"

#include "Main.h"  // アプリケーション独自の定義

#if !(TKERNEL_PRID == 0x0007U && (TKERNEL_PRVER & 0xf000U) == 0x3000U)
#error The kernel does not match this configuration file.
#endif

/*
 *  Include Directives
 */

/*
*	動的に生成するカーネル資源の数を定義
*	(静的なテーブルで定義される資源に追加して動的に
* 	生成できる資源数を定義
*/
#define	TNUM_AID_TSK	0 		// タスク
#define	TNUM_AID_SEM	0 		// セマフォ
#define	TNUM_AID_FLG	0 		// イベントフラグ
#define	TNUM_AID_DTQ	0 		// データキュー
#define	TNUM_AID_PDQ	0 		// 優先度データキュー
#define	TNUM_AID_MTX	0 		// ミューテックス
#define	TNUM_AID_MPF	0 		// 固定長メモリプール
#define	TNUM_AID_CYC	0 		// 周期通知
#define	TNUM_AID_ALM	0 		// アラーム通知
#define	TNUM_AID_ISR	0 		// ISR（割込みサービスルーチン

/*
 *  Task Management Functions
 */

/*
 *  タスク初期化ブロック
 *  (kernel/task.h)
 *
 * typedef struct task_initialization_block {
 *    ATR         tskatr;         // タスク属性
 *    intptr_t    exinf;          // タスクの拡張情報
 *    TASK        task;           // タスクの起動番地
 *    uint_t      ipriority;      // タスクの起動時優先度（内部表現）
 *    size_t      stksz;          // スタック領域のサイズ（丸めた値）
 *    void        *stk;           // スタック領域の先頭番地
 * } TINIB;
 */

const TINIB _kernel_tinib_table[] = {
		{ TA_ACT, 0, root_task, INT_PRIORITY(MID_PRIORITY), ROUND_STK_T(STACK_SIZE), NULL},
		__SOLID_RESERVED_TASKS__
};

/*
 * タスク初期化ブロックの設定例
 *
 *
 * #define STACK_SIZE			2048
 *
 * extern void task1(void *);
 * extern void task2(void *);
 *
 * static STK_T _stack_TASK1[COUNT_STK_T(STACK_SIZE)];
 *
 * const TINIB _kernel_tinib_table[] = {
 *
 *     // 生成時に起動(TA_ACT), スタック領域のアドレスを指定 （スタックフェンス機能 有効)
 *     { TA_ACT, 0, task1, INT_PRIORITY(MID_PRIORITY), ROUND_STK_T(STACK_SIZE), _stack_TASK1 },
 *
 *     // 生成時には起動しない, スタックをカーネル内で確保（スタックフェンス機能 有効)
 *     { TA_NULL, 0, task2, INT_PRIORITY(HIGH_PRIORITY), ROUND_STK_T(STACK_SIZE), NULL},
 *
 * };
 *
 */

const ID _kernel_torder_table[] = {
	ROOT_TASK,
	__SOLID_RESERVED_TASK_ORDER__
};

/*
 *  Semaphore Functions
 */

/*
 *  セマフォ初期化ブロック
 *  (kernel/semaphore.h)
 *
 * typedef struct semaphore_initialization_block {
 *    ATR         sematr;         // セマフォ属性
 *    uint_t      isemcnt;        // セマフォの資源数の初期値
 *    uint_t      maxsem;         // セマフォの最大資源数
 * } SEMINIB;
 */

const SEMINIB _kernel_seminib_table[] = {
	__SOLID_RESERVED_SEMS__
};

/*
 *  セマフォ初期化ブロックの設定例
 *
 * const SEMINIB _kernel_seminib_table[] = {
 *    { TA_NULL, 1, 1 },	// 待ち行列=FIFO順, 最大数=1,初期値=1でセマフォを生成
 *    { TA_TPRI, 0, 1 },  // 待ち行列=タスク優先度順, 最大数=1,初期値=0でセマフォを生成
 *};
 *
 */

/*
 *  Eventflag Functions
 */

/*
 *  イベントフラグ初期化ブロック
 *  (kernel/eventflag.h)
 *
 * typedef struct eventflag_initialization_block {
 *    ATR         flgatr;         // イベントフラグ属性
 *    FLGPTN      iflgptn;        // イベントフラグのビットパターンの初期値
 * } FLGINIB;
 *
 */

const FLGINIB _kernel_flginib_table[] = {

};

/*
 *  イベントフラグ初期化ブロックの設定例
 *
 * const FLGINIB _kernel_flginib_table[] = {
 *		{ TA_NULL, 0x00 },			// 複数タスク待ち不可, 待ち行列=FIFO順, フラグ初期値=0
 *		{ TA_CLR, 0x01 }, 			// 複数タスク待ち不可, 待ち行列=FIFO順, 待ち解除時にフラグクリア,フラグ初期値=1
 *		{ TA_WMUL|TA_CLR, 0x00 }, 	// 複数タスク待ち許可, 待ち行列=FIFO順, 待ち解除時にフラグクリア,フラグ初期値=0
 *		{ TA_WMUL|TA_TPRI, 0x00 }, 	// 複数タスク待ち許可, 待ち行列=タスク優先度順, フラグ初期値=0
 * };
 *
 */

/*
 *  Dataqueue Functions
 */

/*
 *  データキュー初期化ブロック
 *  (kernel/dataqueue.h)
 *
 * typedef struct dataqueue_initialization_block {
 *    ATR         dtqatr;         // データキュー属性
 *    uint_t      dtqcnt;         // データキューの容量
 *    DTQMB       *p_dtqmb;       // データキュー管理領域の先頭番地
 * } DTQINIB;
 */

const DTQINIB _kernel_dtqinib_table[] = {
	__SOLID_RESERVED_DTQS__
};

/*
 *  データキュー初期化ブロックの設定例
 *
 * static MB_T    _kernel_dtqmb_DTQ3[TCNT_DTQMB(2)];
 *
 * const DTQINIB _kernel_dtqinib_table[] = {
 *     { TA_NULL, 2, NULL },              // 待ち行列=FIFO順, 容量=2
 *     { TA_NULL, 0, NULL },              // 待ち行列=FIFO順, 容量=0
 *     { TA_TPRI, 2, _kernel_dtqmb_DTQ3 },// 待ち行列=タスク優先度順, 容量=2, 管理領域を指定
 * };
 *
 */


/*
 *  Priority Dataqueue Functions
 */

/*
 *  優先度データキュー初期化ブロック
 *  (kernel/pridataq.h)
 *
 * typedef struct pridataq_initialization_block {
 *     ATR         pdqatr;         // 優先度データキュー属性
 *     uint_t      pdqcnt;         // 優先度データキューの容量
 *     PRI         maxdpri;        // データ優先度の最大値
 *     PDQMB       *p_pdqmb;       // 優先度データキュー管理領域の先頭番地
 * } PDQINIB;
 */

const PDQINIB _kernel_pdqinib_table[] = {

};

/*
 *  優先度データキュー初期化ブロックの設定例
 *
 *
 * static MB_T    _kernel_pdqmb_PDQ3[TCNT_PDQMB(2)];
 *
 * const PDQINIB _kernel_pdqinib_table[] = {
 *     { TA_NULL, 2, TMAX_DPRI, NULL },              // 送信待ち行列=FIFO順, データ優先度=最大(最低)
 *     { TA_NULL, 0, 8, NULL },                      // 送信待ち行列=FIFO順、容量=0
 *     { TA_TPRI, 2, TMAX_DPRI, _kernel_pdqmb_PDQ3 },// 送信待ち行列=タスク優先度順,  管理領域を指定
 * };
 *
 */


/*
 *  Mutex Functions
 */

/*
 *  ミューテックス初期化ブロック
 *  (kernel/mutex.h)
 *
 * typedef struct mutex_initialization_block {
 *     ATR         mtxatr;         // ミューテックス属性
 *     uint_t      ceilpri;        // ミューテックスの上限優先度（内部表現）
 * } MTXINIB;
 */

const MTXINIB _kernel_mtxinib_table[] = {

};

/*
 *  ミューテックス初期化ブロックの設定例
 *
 * const MTXINIB _kernel_mtxinib_table[] = {
 *     { TA_NULL, INT_PRIORITY(0) },    //待ち行列=FIFO順 (優先度は無効)
 *     { TA_CEILING, INT_PRIORITY(9) }, //優先度上限ミューテックス, 上限優先度有効
 *     { TA_TPRI, INT_PRIORITY(0) },    //待ち行列=タスク優先度順、優先度上限なし
 * };
 *
 */

#ifdef TOPPERS_SUPPORT_MESSAGEBUF
/*
 *  メッセージバッファ初期化ブロック(optional)
 *  (kernel/messagebuf.h)
 *
 *  typedef struct messagebuf_initialization_block {
 *     ATR         mbfatr;			   // メッセージバッファ属性
 *     uint_t      maxmsz;			   // メッセージの最大長(バイト数)
 *     size_t      mbfsz;			     // メッセージバッファ管理領域のサイズ(バイト数)
 *     void        *mbfmb;         // メッセージバッファ管理領域の先頭番地
 *  } MBFINIB;
 */
const MBFINIB _kernel_mbfinib_table[] = {

};

/*
 *  メッセージバッファ初期化ブロックの設定例
 *
 * #define  MBF3_MSGSZ        50
 * #define  MBF3_MSGCNT       100
 *
 * static MB_T    _kernel_mbfmb_MBF3[TCNT_MBFMB(MBF3_MSGCNT, MBF3_MSGSZ)];
 *
 * const MBFINIB _kernel_mbfinib_table[] = {
 *    // 最大メッセージサイズ=16, 管理領域 =32バイト
 *     { TA_NULL, 16, 32, NULL },
 *    // 最大メッセージサイズ=8, 管理領域=maxmsz * 16個分
 *     { TA_NULL, 8, TSZ_MBFMB(16,8), NULL },
 *    // 最大メッセージサイズ=50, 管理領域=maxmsz * 100個分
 *     { TA_NULL, MBF3_MSGSZ, TSZ_MBFMB(MBF3_MSGCNT,MBF3_MSGSZ), _kernel_mbfmb_MBF3 },
 * };
 */
#endif /* TOPPERS_SUPPORT_MESSAGEBUF */

/*
 *  Fixed-sized Memorypool Functions
 */

/*
 *  固定長メモリプール初期化ブロック
 *  (kernel/mempfix.h)
 *
 * typedef struct fixed_memorypool_initialization_block {
 *     ATR         mpfatr;         // 固定長メモリプール属性
 *     uint_t      blkcnt;         // メモリブロック数
 *     uint_t      blksz;          // メモリブロックのサイズ（丸めた値）
 *     void        *mpf;           // 固定長メモリプール領域の先頭番地
 *     MPFMB       *p_mpfmb;       // 固定長メモリプール管理領域の先頭番地
 * } MPFINIB;
 */

const MPFINIB _kernel_mpfinib_table[] = {

};

/*
 *  固定長メモリプール初期化ブロックの設定例
 *
 *
 * #define MPF_BLOCK_SIZE      64
 *
 * #define NUM_BLOCKS_MPF1     1
 * #define NUM_BLOCKS_MPF2     2
 * #define NUM_BLOCKS_MPF3     1
 * #define NUM_BLOCKS_MPF4     1
 *
 * static  MPF_T   _pool1[ NUM_BLOCKS_MPF1 * COUNT_MPF_T(MPF_BLOCK_SIZE) ];
 * static  MPF_T   _pool4[ NUM_BLOCKS_MPF2 * COUNT_MPF_T(MPF_BLOCK_SIZE) ];
 *
 * static  MB_T    _mpfmb2[TCNT_MPFMB(NUM_BLOCKS_MPF2)];
 * static  MB_T    _mpfmb4[TCNT_MPFMB(NUM_BLOCKS_MPF4)];
 *
 * const MPFINIB _kernel_mpfinib_table[] = {
 *     { TA_NULL, NUM_BLOCKS_MPF1, ROUND_MPF_T(MPF_BLOCK_SIZE),_pool1, NULL },           // データ領域を静的変数で確保
 *     { TA_NULL, NUM_BLOCKS_MPF2, ROUND_MPF_T(MPF_BLOCK_SIZE),NULL, (MPFMB*)_mpfmb2},   // 管理領域を静的変数で確保
 *     { TA_TPRI, NUM_BLOCKS_MPF3, ROUND_MPF_T(1),NULL,NULL },                           // 待ち行列=タスク優先度順
 *     { TA_NULL, NUM_BLOCKS_MPF4, ROUND_MPF_T(MPF_BLOCK_SIZE),_pool4, (MPFMB*)_mpfmb4 },// すべての領域を静的変数で確保
 * };
 *
 */



/*
 *  Cyclic Notification Functions
 */

/*
 *  周期通知初期化ブロック
 *  (kernel/cyclic.h)
 *
 * typedef struct cyclic_handler_initialization_block {
 *     ATR         cycatr;         // 周期通知属性
 *     intptr_t    exinf;          // 通知ハンドラの拡張情報
 *     NFYHDR      nfyhdr;         // 通知ハンドラの起動番地
 *     RELTIM      cyctim;         // 周期通知の起動周期
 *     RELTIM      cycphs;         // 周期通知の起動位相
 * } CYCINIB;
 */

const CYCINIB _kernel_cycinib_table[] = {

};

/*
 *  周期通知初期化ブロックの設定例
 *
 * #define CYC1_CYCLE      100U       // 100usec
 * extern void cyc_func(intptr_t exinf);
 *
 * const CYCINIB _kernel_cycinib_table[] = {
 *     { TA_STA, 0, cyc_func, CYC1_CYCLE, CYC1_CYCLE }, // 生成と同時に100usec間隔でcyc_funcを呼び出す
 * };
 *
 */

/*
 *  Alarm Notification Functions
 */

/*
 *  アラーム通知初期化ブロック
 *  (kernel/alarm.h)
 * typedef struct alarm_handler_initialization_block {
 *	 ATR		 almatr;		 // アラーム通知属性
 *	 T_NFYINFO   nfyinfo;	   //通知パラメータ構造体
 * } ALMINIB;
 *
 *	  通知パラメータ構造体の定義:
 *
 *		  typedef struct {
 *		  MODE	nfymode;			 // 通知処理モード
 *			  union {					 // タイムイベントの通知に関する付随情報
 *				  T_NFY_COMMON	initializer;
 *				  T_NFY_HDR   handler;
 *				  T_NFY_VAR   setvar;
 *				  T_NFY_IVAR  incvar;
 *				  T_NFY_TSK   acttsk;
 *				  T_NFY_TSK   wuptsk;
 *				  T_NFY_SEM   sigsem;
 *				  T_NFY_FLG   setflg;
 *				  T_NFY_DTQ   snddtq;
 *			  } nfy;
 *			  union {					 // エラーの通知に関する付随情報
 *				  T_NFY_COMMON	initializer;
 *				  T_ENFY_VAR  setvar;
 *				  T_NFY_IVAR  incvar;
 *				  T_NFY_TSK   acttsk;
 *				  T_NFY_TSK   wuptsk;
 *				  T_NFY_SEM   sigsem;
 *				  T_NFY_FLG   setflg;
 *				  T_ENFY_DTQ  snddtq;
 *			  } enfy;
 *		  } T_NFYINFO;
 *
 */

const ALMINIB _kernel_alminib_table[] = {

};

/*
 * アラーム通知初期化ブロックの設定例
 *
 * #define TASK1 1
 * #define TASK2 2
 * #define SEM1 1
 * #define FLG1 1
 * #define DTQ1 1
 *
 * extern bool_t event_variable;
 * extern int_t count_variable;
 * extern ER error_variable;
 *
 * const ALMINIB _kernel_alminib_table[] = {
 * 	// 通知時に変数 event_variableにtrueを設定、エラー通知なし
 * 	{TA_NULL, {TNFY_SETVAR, {NFY_PARAM_SETVAR(&event_variable, true)}}},
 *
 * 	// 通知時にタスク(ID=TASK2)を起動、エラー通知なし
 * 	{TA_NULL, {TNFY_ACTTSK, {NFY_PARAM_ACT_TSK(TASK2)}}},
 *
 * 	// 通知時にタスク(ID=TASK2)を起床、エラー通知発生時には変数error_varibaleにエラーを格納する
 * 	{TA_NULL, {TNFY_WUPTSK | TENFY_SETVAR, {NFY_PARAM_WUP_TSK(TASK2)}, {ERRNFY_PARAM_SETVAR(&error_variable)}}},
 *
 * 	// 通知時にセマフォに返却(sig_sem)、エラー通知発生時にはタスク(ID=TASK2)を起床
 * 	{TA_NULL, {TNFY_SIGSEM | TENFY_ACTTSK, {NFY_PARAM_SIG_SEM(SEM1)}, {ERRNFY_PARAM_ACT_TSK(TASK2)}}},
 *
 * 	// 通知時にイベントフラグをセット(ID FLG1に対し 0x01)
 * 	{TA_NULL, {TNFY_SETFLG, {NFY_PARAM_SET_FLG(FLG1, 0x01)}}},
 *
 * 	// 通知時にデータキューに対しデータ(0x01)を送信、エラーが発生した場合はTASK2を起床させる
 * 	{TA_NULL, {TNFY_SNDDTQ | TENFY_WUPTSK, {NFY_PARAM_SND_DTQ(DTQ1, 0x01)}, {ERRNFY_PARAM_WUP_TSK(TASK2)}}},
 *
 * 	// 通知時にタスク(ID=TASK2)を起動、 エラー発生時にはSEM1に対しsig_semを実行する
 * 	{TA_NULL, {TNFY_ACTTSK | TENFY_SIGSEM, {NFY_PARAM_ACT_TSK(TASK2)}, {ERRNFY_PARAM_SIG_SEM(SEM1)}}},
 *
 * 	// 通知時にタスク(ID=TASK2)を起動、 エラー発生時にはイベントフラグ FLG1に対し0x02をセット
 * 	{TA_NULL, {TNFY_ACTTSK | TENFY_SETFLG, {NFY_PARAM_ACT_TSK(TASK2)}, {ERRNFY_PARAM_SET_FLG(FLG1, 0x02)}}},
 *
 * 	// 通知時にタスク(ID=TASK2)を起動、 エラー発生時にはデータキューに対しエラーコードを送信
 * 	{TA_NULL, {TNFY_ACTTSK | TENFY_SNDDTQ, {NFY_PARAM_ACT_TSK(TASK2)}, {ERRNFY_PARAM_SND_DTQ(DTQ1)}}},
 *
 * 	// 通知時に変数 count_varibaleをインクリメント、エラー通知なし
 * 	{TA_NULL, {TNFY_INCVAR, {NFY_PARAM_INCVAR(&count_variable)}}},
 *
 * 	// 通知時にタスク(ID=TASK2)を起動、 エラー発生時には通知時に変数 count_varibaleをインクリメント
 * 	{TA_NULL, {TNFY_ACTTSK | TENFY_INCVAR, {NFY_PARAM_ACT_TSK(TASK2)}, {ERRNFY_PARAM_INCVAR(&count_variable)}}},
 * };
 *
*/


/*
 *  Interrupt Management Functions
 */

/*
 *  割込み初期化設定ブロック
 *  (kernel/interrupt.h)
 *
 * typedef struct interrupt_handler_initialization_block {
 *     INTNO       intno;     // 割込み番号
 *     ATR         inhatr;    // 割込みハンドラ属性
 *     FP          inthdr;    // 割込みハンドラの出入口処理の番地 (ISR使用時はINTINI_USE_ISRを指定)
 *                               ->関数型:void (foo)(void);
 *     ATR         intatr;    // 割込み属性 (TA_ENAINTのみサポート）
 *     PRI         intpri;    // 割込み優先度 [-1:最低優先度 ... -32:最高優先度]
 * } INTINIB;
 */

const INTINIB _kernel_intinib_table[] = {

};

/*
 * 割込み初期化設定ブロックの設定例
 *
 *
 * #define     INTNO_SGI1                1    // 割込み番号1
 * #define     INTNO_SGI2                2    // 割込み番号2
 *
 * extern void int_handler_sgi2(void);
 *
 * //
 * // 割込み初期化ブロック
 * //
 * const INTINIB _kernel_intinib_table[] = {
 *     {INTNO_SGI1,TA_NULL, INTINI_USE_ISR, TA_NULL,(-1)},     // IRQ1 <- 最低優先度、ISRを使用
 *     {INTNO_SGI2,TA_NULL, int_handler_sgi2, TA_NULL,(-32)},  // IRQ2 <- 最高優先度、直接割込みハンドラ関数を登録
 * };
 */
/*
*
* 割込みサービスルーチン(ISR)初期化ブロック
* (kernel.h)
*
* typedef struct t_cisr {
*   ATR         isratr;     // 割込みサービスルーチン属性
*   intptr_t    exinf;      // 割込みサービスルーチンの拡張情報
*   INTNO       intno;      // 割込みサービスルーチンを登録する割込み番号
*   ISR         isr;        // 割込みサービスルーチンの先頭番地
*                               ->関数型:void (foo)(intptr_t);
*   PRI         isrpri;     // 割込みサービスルーチン優先度 [1:最高優先度 ... 16:最低優先度]
*} T_CISR;
*/

const T_CISR  _kernel_isrini_cfg_table[] = {

};


/*
 * 割込みサービスルーチン(ISR)初期化ブロックの設定例
 *
 *
 * #define     INTNO_SGI1                1    // 割込み番号1
 *
 * #define ISRPRI_HIGH_PRIORITY   1       // 高優先度 //
 * #define ISRPRI_MID_PRIORITY    10      // 中優先度 //
 * #define ISRPRI_LOW_PRIORITY    16      // 低優先度 // //
 *
 * extern void isr_high_handler(intptr_t exinf); //
 * extern void isr_mid_handler(intptr_t exinf); //
 * extern void isr_low_handler(intptr_t exinf); //
 *
 * //
 * //  割込みサービルルーチン初期化ブロック
 * //
 * const T_CISR  _kernel_isrini_cfg_table[] = {
 *     {TA_NULL, (intptr_t)1, INTNO_SGI1, isr_mid_handler, ISRPRI_MID_PRIORITY},  // 中優先度のISRを登録
 *     {TA_NULL, (intptr_t)2, INTNO_SGI1, isr_low_handler, ISRPRI_LOW_PRIORITY},  // 低優先度のISRを登録
 *     {TA_NULL, (intptr_t)3, INTNO_SGI1, isr_high_handler, ISRPRI_TOP_PRIORITY}, // 高優先度のISRを登録
 * };
 *
 */

/*
* 割込みサービスルーチン(ISR)初期化ブロックに登録したISRに
* 対応するIDを指定してください。
*/
const ID _kernel_isrorder_table[] = {

};

/*
 *  Initialization Routine
 */

/*
 * ユーザ初期化関数呼出
 *
 * カーネル起動時にタスクの起動、ディスパッチを開始する前に呼び出されます。
 * タスクが動作する前に行っておく処理がある場合に必要な処理があれば記述してください。
 * この関数から復帰するまでタスクは開始されません。
 */

void
_kernel_call_inirtn(void)
{
		__SOLID_SYS_INTRTN();
}

/*
 *  Termination Routine
 */

/*
 * ユーザ終了処理関数呼出
 *
 * カーネルが終了したタイミングで呼び出されます。
 * 必要な処理があれば記述してください。
 * この関数が呼び出された時点ではすべてのタスク及びカーネルの動作は
 * 終了しているため、終了処理の中でサービスコールなどのカーネルの機能を使うことはできません。
 */

void
_kernel_call_terrtn(void)
{
		__SOLID_SYS_TERRTN();
}

/***** DO NOT REMOVE *****/
#include <kernel_cb_store.c>
/***** DO NOT REMOVE *****/
