
#include <syslog.h>
#include "kernel_cfg.h"
#include "Main.h"

/*
 *  メインタスク
 */

void root_task(intptr_t exinf)
{
	syslog_msk_log(LOG_UPTO(LOG_INFO), LOG_UPTO(LOG_INFO));
	syslog(LOG_INFO, "root task started\n");

	act_tsk(HELLO_TASK);
	act_tsk(XML_MAS_TASK);
	
}
