
#include <syslog.h>
#include "goodbye.h"

void goodbye_task(intptr_t exinf)
{
	syslog_msk_log(LOG_UPTO(LOG_INFO), LOG_UPTO(LOG_INFO));
	syslog(LOG_INFO, "goodbye,world\n");
}