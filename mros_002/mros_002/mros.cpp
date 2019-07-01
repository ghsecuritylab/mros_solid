#include "mros.h"
#include <syslog.h>
#include <cstring>
#include <string>
#include <iostream>
char mem[1024 * 32];
void xml_mas_task(intptr_t exinf){
	#ifndef _XML_MASTER_
	#define _XML_MASTER_
	syslog(LOG_INFO,"start xml_mas_task");
	intptr_t dq;
	unsigned char *xdq;
	char rbuf[512];
	//dq = (intptr_t *)new char[4];		//mallocと何が違うか．特にmallocなくていけた.ssp/sampleを参考

	rcv_dtq(XML_DTQ,(intptr_t*)(&dq));
	xdq = (unsigned char*)dq;
	#endif
	
	int size = xdq[1];
	size += xdq[2]*256;
	size += xdq[3]*65536;
	
	syslog(LOG_INFO,"received size:%d",size);
	memcpy(&rbuf,&mem,size);			//rbufに共有メモリmemの内容をコピー
	//copy xml data to rbuf is ok

	std::string str,meth;
	str = rbuf;		//string* = char*
	int mh,mt;
	mh = (int)str.find("<methodCall>");
	mt = (int)str.find("</methodCall>");
	syslog(LOG_INFO, "<methodCall> size:%d", sizeof("<methodCall>"));		//charのsizeofだと文字数＋１になるので次の行のスタートでー1している
	for(int i = mh + sizeof("<methodCall>")-1;i<mt;i++){
		meth = meth + str[i];
	}
	std::cout << "meth:" << meth << std::endl;
	slp_tsk();
}