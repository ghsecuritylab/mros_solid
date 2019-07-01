#include "mros.h"
#include <syslog.h>

void xml_mas_task(intptr_t exinf){
	#ifndef _XML_MASTER_
	#define _XML_MASTER_
	syslog(LOG_INFO,"start xml_mas_task");
	intptr_t dq;
	unsigned char *xdq;
	
	//dq = (intptr_t *)new char[4];		//mallocと何が違うか．特にmallocなくていけた.ssp/sampleを参考

	rcv_dtq(XML_DTQ,(intptr_t*)(&dq));
	xdq = (unsigned char*)dq;
	#endif
	
	int size = xdq[1];
	size += xdq[2]*256;
	size += xdq[3]*65536;
	
	syslog(LOG_INFO,"received size:%d",size);
	
	slp_tsk();
}