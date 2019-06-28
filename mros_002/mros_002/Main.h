/*
 *
 * ASP3カーネルのタスク起動動作確認用
 *
 */

#include <kernel.h>

/*
 *  ターゲット依存の定義
 */

/*
 *  各タスクの優先度の定義
 */

#define EXC_PRIORITY 1  /* 例外処理タスクの優先度 */
#define MAIN_PRIORITY 5 /* メインタスクの優先度 */
						/* HIGH_PRIORITYより高くすること */

#define HIGH_PRIORITY 9 /* 並行実行されるタスクの優先度 */
#define MID_PRIORITY 10
#define LOW_PRIORITY 11

/*
 *  ターゲットに依存する可能性のある定数の定義
 */

#ifndef STACK_SIZE
#define STACK_SIZE 4096 /* タスクのスタックサイズ */
#endif					/* STACK_SIZE */

/*
 *  関数のプロトタイプ宣言
 */
#ifndef TOPPERS_MACRO_ONLY

void root_task(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */
