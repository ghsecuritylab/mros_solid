#include "ros.h"
#include <syslog.h>
#include <kernel.h>
#include "kernel_cfg.h"


std::vector<ID> IDv;
std::vector<std::string> node_nv;

int ID_find(std::vector<ID> IDv,ID id){
	for(unsigned int i=0;i<IDv.size();i++){
		if(IDv[i] == id){
			return i;
		}
	}
	return -1;
}

void ros::init(int argc, char *argv, std::string node_name)
{

	//syslog(LOG_INFO, "node_name:%s", node_name.c_str());
	ID id;
	get_tid(&id); //User taskで表示させたのと同じID
	syslog(LOG_INFO, "task ID :[%d]", id);
	
	IDv.push_back(id);

	//適当にIDvにID入れとこうとしても，abort.cで怒られる
	//IDv.push_back((ID)4);
	//IDv.push_back((ID)0);
	/* 問題の箇所 とりあえずスルーするしかない*/
	//node_nv.push_back(node_name);
}

ros::Publisher ros::NodeHandle::advertise(std::string topic,int queue_size){
	//セマフォの確認
	//node_nv.push_back("hoge");
	//stateの変更
	//syslog(LOG_INFO, "Change state [%d]", state);
	//ID id;
	//get_tid(&id);
	Publisher pub;
	pub.ID = 2;
	//pub.ID = count	//ほんとはこっち
	//pub.node = node_nv[ID_find(IDv,id)].c_str();
	//pub.topic = topic.c_str();
	/*
	std::string pstr;
	pstr = "<methodCall>registerPublisher</methodCall>\n";	//これもだめやけどcpptestの方ではいけてる
	pstr += "<topic_name>/";
	pstr += topic;
	pstr += "</topic_name>\n";
	pstr += "<topic_type>";
	
	pstr += "</topic_type>\n";
	*/
	
	unsigned char pbuf[4];
	//int size = pstr.length();	//strlenとは何が違うか
	unsigned int size = 2500;
	pbuf[0] = pub.ID;
	pbuf[1] = size;
	pbuf[2] = size/10;
	pbuf[3] = size/100;
	intptr_t *pdq = (intptr_t) &pbuf;
	syslog(LOG_INFO,"XML_DTQ:%d",XML_DTQ);
	for(int i=0;i<4;i++){
		syslog(LOG_INFO, "pbuf:%d", pbuf[i]);
	}
	
	snd_dtq(XML_DTQ, *pdq);
	slp_tsk();
	return pub;
}


