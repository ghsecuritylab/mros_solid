#ifndef _ROS_H_
#define _ROS_H_

#include <string>
#include <vector>
#include "mros.h"

//XML-MASタスク用のノード情報構造体
typedef struct node{
	std::string node_name;				//ノードの名前
	bool node_type;						//true->subscriber,false->publisher
	bool pub_stat;						//パブリッシュの状態
	char ID;							//mROS ID > 1
	std::string topic_name;				//ROS
	std::string topic_type;				//ROS
	std::string callerid;				//ROS
	std::string message_definition;		//ROS
	std::string uri;					//自ノードのURI
	int port;							//for sub 通信相手のノードのXML-RPC受付ポート
	std::string fptr;					//for sub コールバック関数のポインタ
	std::string ip;						//for sub 通信相手となるノードのIPアドレス
public:
	node() { this->pub_stat = false; };
	void set_node_type(bool type) { this->node_type = type; };
	void set_pub() { this->pub_stat = true; };
	void set_ID(char c) { this->ID = c; };
	void set_topic_name(std::string t) { this->topic_name = t; };
	void set_topic_type(std::string t) { this->topic_type = t; };
	void set_callerid(std::string t) { this->callerid = t; };
	void set_message_definition(std::string t) { this->message_definition = t; };
	void set_uri(std::string t) { this->uri = t; };
	void set_port(int t) { this->port = t; };
	void set_fptr(std::string t) { this->fptr = t; };
	void set_ip(std::string t) { this->ip = t; };
}node;

namespace ros{
	class Publisher{
		public:
			const char* topic;
			const char* node;
			char ID;
			template <class T> void publish(T& data);
	};
	class NodeHandle{
		public:
			Publisher advertise(std::string topic,int queue_size);
	};
	void init(std::string node_name);
}
#endif /* _ROS_H_ */