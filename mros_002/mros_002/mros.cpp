#include "mros.h"
#include <syslog.h>
#include <cstring>
#include <string>
#include <iostream>
#include "TCPSocketConnection.h"


char mem[1024 * 32];					//shared memory
const char *m_ip = "192.168.11.4";		//ros master IP
const int m_port = 11311;				//ros master xmlrpc port 

void xml_mas_task(intptr_t exinf){
	#ifndef _XML_MASTER_
	#define _XML_MASTER_
	syslog(LOG_INFO,"start xml_mas_task");
	intptr_t dq;
	unsigned char *xdq;	//rcv_dtq
	char rbuf[512];		//共有メモリに書いたものをコピー
	#endif /* _XML_MASTER_ */
	//dq = (intptr_t *)new char[4];		//mallocと何が違うか．特にmallocなくていけた.ssp/sampleを参考

	//ソケットを作ってデータキューに入ってたら受信して
	//methによって分岐して，
	while(1){
		syslog(LOG_INFO,"xml_mas_task enter loop");
		TCPSocketConnection xml_mas_sock;
		xml_mas_sock.set_blocking(true,1500);
		
		//データキューに何もなかったらここで止まるわけやんな
		rcv_dtq(XML_DTQ,(intptr_t*)(&dq));
		xdq = (unsigned char*)dq;
		int size = xdq[1];
		size += xdq[2] * 256;
		size += xdq[3] * 65536;
		syslog(LOG_INFO, "received size:%d", size);
		memcpy(&rbuf, &mem, size); //rbufに共有メモリmemの内容をコピー
		//copy xml data to rbuf is ok

		std::string str, meth;
		str = rbuf; //string* = char*
		int mh, mt;
		mh = (int)str.find("<methodCall>");
		mt = (int)str.find("</methodCall>");
		syslog(LOG_INFO, "<methodCall> size:%d", sizeof("<methodCall>")); //charのsizeofだと文字数＋１になるので次の行のスタートでー1している
		for (int i = mh + sizeof("<methodCall>") - 1; i < mt; i++){
			meth = meth + str[i];
		}
		std::cout << "meth:" << meth << std::endl;
		
		//==========registerPublisher=================
		if (meth == "registerPublisher"){
			syslog(LOG_NOTICE, "meth:registerPublisher");
			std::string xml;
			xml_mas_sock.single_connect(m_ip, m_port);
		}
		//xml_mas_sock.close();
	}

}