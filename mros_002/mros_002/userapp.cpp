#include <iostream>
//iostreamをインクルードすると，solid_endがe0000->f0000に増えた
#include <stdlib.h>
//標準ライブラリの予約語alignがt_stddef.hで定義されてしまったから，ダメだった．
//標準ライブラリを先にインクルードすればおｋ
#include <syslog.h>

#include "userapp.h"
#include "ros.h"
//hello_taskが今回の
void hello_task(intptr_t exinf)
{
	//ros::init ok
	std::string topic1 = "mros_test_topic";
	ros::init(topic1);

	//NodeHandleのオブジェクトつくる	:ok
	ros::NodeHandle n;
	//advertise		
	ros::Publisher chatter_pub = n.advertise(topic1,1);

	
	
	intptr_t *dq;
	intptr_t *dqq;
	free(dq);
	free(dqq);
	/*
	dq = (intptr_t *)malloc(sizeof(char) * 10); //コード分析で怒られはするけれどビルドは通る
	dqq = (intptr_t *)malloc(sizeof(char) * 10);
	*/
	/*
	rcv_dtq(DTQ_ID, dq);   //origin mrosの方式
	rcv_dtq(DTQ_IDD, dqq); //int型が送られてくるはず
	*/
	/*
	unsigned char *mros_dq;
	unsigned int *idq;
	mros_dq = (unsigned char *)dq;
	idq = (unsigned int *)dqq;

	int size = mros_dq[1];
	size += mros_dq[2] * 256;
	size += mros_dq[3] * 65536;
	*/
	
	
}
