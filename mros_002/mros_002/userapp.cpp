#include <string>
#include <iostream>
#include <vector>
#include <syslog.h>
#include "ros.h"
#include "userapp.h"
void user_task(intptr_t exinf){
	//char*->std::stringの方法が紹介されてたけどこれもだめ
	/*
	std::string hoy;
	const char *cstr = "<methodCall>registerPublisher</methodCall>\n";
	hoy = std::string(cstr);
	std::cout << hoy << std::endl;
	*/
	int argc = 0;
	char *argv = NULL;
	std::string topic1 = "mros_test_node";
	ros::init(argc,argv,topic1);
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise((std::string)"mros_chat",10);
	syslog(LOG_INFO,"success advertise");
	
	slp_tsk();
}