
#include <iostream>
//iostreamをインクルードすると，solid_endがe0000->f0000に増えた
#include <stdlib.h>
//標準ライブラリの予約語alignがt_stddef.hで定義されてしまったから，ダメだった．
//標準ライブラリを先にインクルードすればおｋ
#include <syslog.h>

#include "hello.h"
#include "ros.h"

void hello_task(intptr_t exinf){
	
	std::string pstr;
	pstr = "<methodCall>registerPublisher</methodCall>\n";

	//syslog(LOG_INFO,"vi_size:%d",vi.size());
	syslog_msk_log(LOG_UPTO(LOG_INFO), LOG_UPTO(LOG_INFO));
	syslog(LOG_INFO, "Hello,world\n");
	ID id;
	get_tid(&id);
	syslog(LOG_INFO,"task id [%d]",id);
	//問題のpush_back
	std::vector<std::string> v;
	std::string str = "test";
	v.push_back(str);
	std::cout << v[0] << std::endl;

	int argc = 0;
	char *argv = NULL;
	//ros::init(argc,argv,str);
	/*
	std::string str("test");
	std::vector<int> test(3);
	test.push_back(1);
	*/
	
	intptr_t *dq;
	intptr_t *dqq;
	dq =(intptr_t *)malloc(sizeof(char) * 10);	//コード分析で怒られはするけれどビルドは通る
	dqq = (intptr_t *)malloc(sizeof(char) * 10);
	syslog(LOG_INFO,"start receive data queue\n");
	
	rcv_dtq(DTQ_ID, dq);
	rcv_dtq(DTQ_IDD,dqq);
	syslog(LOG_INFO, "received data queue\n");
	int *xdq;
	unsigned char *ydq;
	
	xdq = (int *)dqq;
	ydq = (unsigned char *)dq;
	for(int i=0;i<1;i++){
		syslog(LOG_INFO, "xdq[i]:%d", xdq[i]);
	}
	int size = ydq[1];
	size += ydq[2]*256;
	size += ydq[3]*65536;
	syslog(LOG_INFO,"size:%d",size);
	
	syslog(LOG_INFO, "ydq[0]:%d", ydq[0]);
	syslog(LOG_INFO, "ydq[1]:%d", ydq[1]);		//208 -> 31
	syslog(LOG_INFO, "ydq[2]:%d", ydq[2]);		//7 -> 48
	syslog(LOG_INFO, "ydq[3]:%d", ydq[3]);		//0 -> -8
	
	free(dq);
	free(dqq);
	
}
