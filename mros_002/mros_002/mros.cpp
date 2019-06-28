#include "mros.h"
#include <kernel.h>
#include "kernel_cfg.h"
#include <stdlib.h>
#include <syslog.h>
void xml_mas_task(){
	intptr_t *dq;
	unsigned char *xdq;
	dq = (intptr_t*)malloc(sizeof(char)*4);
	rcv_dtq(XML_DTQ,dq);
	xdq = (unsigned char*)dq;	
	
	
	for (int i = 0; i < 4; i++)
	{
		syslog(LOG_INFO, "received dtq:%d", xdq[i]);
		syslog(LOG_INFO, "received dq:%d", dq[i]);	
		/*
		received dtq : 80 received dq : -131055792 received dtq : 63 received dq : -507305808 received dtq : 48 received dq : -486068204 received dtq : -8 received dq : 523145
		*/
	}
}