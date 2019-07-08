#include "mros.h"
#include <syslog.h>
#include <cstring>
#include <string>
#include <iostream>
#include "TCPSocketConnection.h"

#include "impl_net_dev_getconfig.c"		//ネットワーク設定ファイルをインクルード

char mem[1024 * 32];					//shared memory
const char *m_ip = "192.168.11.4";		//ros master IP
const int m_port = 11311;				//ros master xmlrpc port 

//set node information
void get_node(node *node,std::string *xml,bool type){
	//ノードの情報が入ってるxml(呼び出し時はstr)からget_ttypeやget_tnameなどでほしい情報を切り出して構造体にセットする
	//切り出しの関数はxmlparser.cpp/hに定義
	std::string c;
	node->set_node_type(type);
	node->set_topic_type(get_ttype(*xml));
	node->set_topic_name(get_tname(*xml));
	node->set_callerid(get_cid(*xml));
	node->set_message_definition(get_msgdef(*xml));
	c += "http://";
	c += _network_config.ipaddr;		//network_config構造体のipaddrメンバ
	c += ":11411";
	node->set_uri(c);
	if(type){	//false->pub true->sub
		node->set_fptr(get_fptr(*xml));
	}

}

void xml_mas_task(intptr_t exinf){
	#ifndef _XML_MASTER_
	#define _XML_MASTER_
	syslog(LOG_INFO,"start xml_mas_task");
	intptr_t dq;
	unsigned char *xdq;	//rcv_dtqしたものをunsigned char*にキャスト
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

		/* xdq[1] = size,xdq[2] = size/256,xdq[3] = size/65536 */
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
			//xml_mas_sock.single_connect(m_ip, m_port);
			node pub;
			//std::string xml;
			syslog(LOG_INFO, "XML_MAS_TASK: register Publisher ID:[%d]", xdq[0]);
			pub.ID = xdq[0];
			get_node(&pub,&str,false);				//pubなのでfalse
			syslog(LOG_INFO,"pubID:%d",pub.ID);
			syslog(LOG_INFO, "pub_topic_name:%s", pub.topic_name.c_str());		//syslogの出力とstd::coutの出力	
			std::cout << "pub_topic_type:" << pub.topic_type <<std::endl;	
			std::cout << "pub_uri:" << pub.uri << std::endl;
		}
		//xml_mas_sock.close();
	}

}