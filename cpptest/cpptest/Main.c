
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
	
	unsigned int size = 2000;
	unsigned char buf[4];				//char -> unsigned charに修正
	buf[0] = 'a';
	buf[1] = size;
	buf[2] = size/256;
	buf[3] = size/65536;
	intptr_t *sdq =(intptr_t) &buf;		//とりあえず無視してok,&はついてもつかなくてもok
	int a[3] = {1,2,3};

	for (int i = 0; i < 4; i++)
	{
		syslog(LOG_INFO, "buf[%d]:%d",i,buf[i]);
	}
	//intptr_t int_data = (intptr_t)&a;
	
	syslog(LOG_INFO,"start send data queue\n");
	snd_dtq(DTQ_ID,*sdq);
	snd_dtq(DTQ_IDD, (intptr_t)a[0]);
	snd_dtq(DTQ_IDD, (intptr_t)a[1]);
	snd_dtq(DTQ_IDD, (intptr_t)a[2]);
	
	//snd_dtqでデータキューにデータが入ってないみたい,サイズは確保されてる(count=0)
	//
	//act_tsk(GOODBYE_TASK);
	
	
	act_tsk(HELLO_TASK);
	syslog(LOG_INFO, "sended data queue\n");
	slp_tsk();
}
