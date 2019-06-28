#include "impl_timer.h"
#include "impl_globaltick.h"

#include "solid_iores.h"
#include "easy_arm.h"

#define TIMER_CLK (32000000UL)
#define TIMER_MAXCOUNT (0xFFFFFFFF)

static SOLID_IORES_INFO g_TimerInfo;
static SOLID_IORES_INFO g_TickInfo;

#define OSTM0CMP	(*((volatile unsigned long*)(g_TimerInfo.addr+0x00)))
#define OSTM0CNT	(*((volatile unsigned long*)(g_TimerInfo.addr+0x04)))
#define OSTM0TE		(*((volatile unsigned char*)(g_TimerInfo.addr+0x10)))
#define OSTM0TS		(*((volatile unsigned char*)(g_TimerInfo.addr+0x14)))
#define OSTM0TT		(*((volatile unsigned char*)(g_TimerInfo.addr+0x18)))
#define OSTM0CTL	(*((volatile unsigned char*)(g_TimerInfo.addr+0x20)))

#define OSTM1CMP	(*((volatile unsigned long*)(g_TickInfo.addr+0x00)))
#define OSTM1CNT	(*((volatile unsigned long*)(g_TickInfo.addr+0x04)))
#define OSTM1TE		(*((volatile unsigned char*)(g_TickInfo.addr+0x10)))
#define OSTM1TS		(*((volatile unsigned char*)(g_TickInfo.addr+0x14)))
#define OSTM1TT		(*((volatile unsigned char*)(g_TickInfo.addr+0x18)))
#define OSTM1CTL	(*((volatile unsigned char*)(g_TickInfo.addr+0x20)))

/* タイマの初期化: ハードウェアを初期化し、pInfoにタイマの情報をセットしてください。 */
void IMPL_TIMER_Init(IMPL_TIMER_INFO* pInfo)
{
    /* OSタイマの情報取得 */
    SOLID_IORES_GetInfo("OS_TIMER", &g_TimerInfo);

    pInfo->tickspersec = TIMER_CLK;
    pInfo->maxticks    = TIMER_MAXCOUNT;
    pInfo->intno       = g_TimerInfo.extra;
    pInfo->config      = 0x02; /* edge trigger */

    OSTM0TT  = 1;	/* カウンタ停止 */
    OSTM0CTL = 2;	/* フリーランニングコンペアモード */
    __DMB();
}

/* タイマの現在の値の取得: 直前のSetTimerからの経過時間をticksで返してください。 */
unsigned long IMPL_TIMER_GetTimerTick()
{
    unsigned long cr = OSTM0CNT;
    return cr;
}

/* タイマの設定: ticksカウント後にタイマ割り込みが発生するように再設定してください。 */
void IMPL_TIMER_SetTimer(unsigned long ticks)
{
    OSTM0TT  = 1;	/* カウンタ停止 */
    __DMB();
    OSTM0CMP = ticks - 1;	/* タイマカウンタ設定*/
    __DMB();
    OSTM0TS  = 1;	/* タイマ開始/再開 */
    __DMB();
}

/* 割込み発生時にタイマ内部の割込みクリア・終了処理が必要な場合は後処理を実装してください */
void  IMPL_TIMER_ClearInterrupt(void)
{
	/* NOP, RZ/A1HのOSタイマはカウント再開で割込み要因がクリアされるため後処理は不要 */
}

/* タイマの一時停止: 省電力等によりタイマを一時停止する時に呼ばれます。 */
void  IMPL_TIMER_Suspend(void)
{
    OSTM0TT  = 1;	/* カウンタ停止 */
    __DMB();
}

/* タイマの再開: 省電力等によりタイマを再開する時に呼ばれます。 */
void  IMPL_TIMER_Resume(void)
{
    OSTM0TS  = 1;	/* タイマ開始/再開 */
    __DMB();
}

/* タイマの初期化: ハードウェアをフリーランモードで初期化し、pInfoのtickspersecに情報をセットしてください。 */
void IMPL_GLOBAL_TICK_Init(IMPL_GLOBAL_TICK_INFO* pInfo)
{
    /* OSタイマの情報取得 */
    SOLID_IORES_GetInfo("OS_TIMER1", &g_TickInfo);

    pInfo->maxticks = 0xFFFFFFFF;
    pInfo->tickspersec = TIMER_CLK;

    OSTM1TT  = 1;	/* カウンタ停止 */
    OSTM1CTL = 2;	/* フリーランニングコンペアモード */
    __DMB();
    OSTM1CMP = 0;	/* タイマカウンタ設定*/
    __DMB();
    OSTM1TS  = 1;	/* タイマ開始/再開 */
    __DMB();
}

/* タイマの現在の値の取得: 呼出時のタイマハードウェアのカウンタ値を64bit値で返してください */
unsigned long long IMPL_GLOBAL_TICK_GetCurrent(void)
{
    return (unsigned long long)(OSTM1CNT);
}

/* タイマの一時停止: 省電力等によりタイマを一時停止する時に呼ばれます。 */
void IMPL_GLOBAL_TICK_Suspend(void)
{
    OSTM1TT  = 1;	/* カウンタ停止 */
    __DMB();
}

/* タイマの再開: 省電力等によりタイマを再開する時に呼ばれます。 */
void IMPL_GLOBAL_TICK_Resume(void)
{
    OSTM1TS  = 1;	/* タイマ開始/再開 */
    __DMB();
}
