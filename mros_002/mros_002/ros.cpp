#include "ros.h"
#include "kernel_cfg.h"
#include <syslog.h>
#include <kernel.h>
#include <iostream>
#include <cstring>
char mem[1024*32];
std::vector<ID> IDv;
std::vector<std::string> node_nv;

int ID_find(std::vector<ID> IDv,ID id){
	for(unsigned int i=0;i<IDv.size();i++){
		if(id == IDv[i]){
			return i;
		}
	}
	return -1;
}

void ros::init(std::string node_name){

	ID id;
	get_tid(&id);
	IDv.push_back(id);
	//適当にpush_back
	for (int i = 3; i < 6; i++)
	{
		IDv.push_back(i);
		syslog(LOG_INFO,"IDv[%d]:%d",i,IDv[5-i]);
	}
	std::cout << node_name << std::endl;

	syslog(LOG_INFO, "task ID:%d", id);
	
	node_nv.push_back(node_name);
	syslog(LOG_INFO, "node_nv[0]:%s", node_nv[0].c_str());
}

ros::Publisher ros::NodeHandle::advertise(std::string topic,int queue_size){
	ID id;
	get_tid(&id);
	Publisher pub;
	pub.node = node_nv[ID_find(IDv,id)].c_str();
	pub.topic = topic.c_str();

	//ここまでok(node_nvが自動変数で見えてないのは気になる...node_nv[0]で取得はできてるけど)
	
	//string pstrにxmlを格納して共有メモリにコピーして，サイズとpub.IDをデータキューで送信
	std::string pstr;
	pstr = "<methodCall>registerPublisher</methodCall>\n";
	pstr += "<topic_name>/";
	pstr += topic;
	pstr += "</topic_name>\n";
	pstr += "<topic_type>std_msgs/String</topic_type>\n";
	pstr += "<caller_id>/";
	pstr += pub.node;
	pstr += "</caller_id>\n";
	pstr += "<message_definition>string data</message_definition>\n";
	pstr += "<fptr>12345671</fptr>\n";
	
	std::memcpy(mem,pstr.c_str(),pstr.size());
	
	//memcpy ok
	
	int size = strlen(pstr.c_str());
	unsigned char pbuf[4];
	intptr_t pdq;
	pbuf[0] = pub.ID;
	pbuf[1] = size;
	pbuf[2] = size/256;
	pbuf[3] = size/65536;

	pdq = (intptr_t) &pbuf;
	snd_dtq(DTQ_ID,pdq);
	slp_tsk();
	return pub;
}