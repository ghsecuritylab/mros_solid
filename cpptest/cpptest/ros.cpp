#include "ros.h"

#include <syslog.h>
#include <kernel.h>

#include <iostream>

std::vector<ID> IDv;
std::vector<std::string> node_nv;

void ros::init(int argc,char *argv,std::string node_name){
	std::vector<std::string> vec;
	std::string test = "inner_function_test";

	ID id;
	get_tid(&id);
	IDv.push_back(id);
	for(int i=0;i<5;i++){
		IDv.push_back(i);
	}
	for(const auto& e:IDv){	//autoは型推論
		//const auto& eを書くことでコンテナ内の要素の変更を禁止，要素のコピーも行わない
		//auto& eだとコンテナ内の要素を変更できる
		//auto eだと各要素がコピーされてからfor文に渡される．要素は変更できない
	}
	std::cout << node_name << std::endl;
	syslog(LOG_INFO, "%s", node_name.c_str());
	
	syslog(LOG_INFO,"task ID:%d",id);
	syslog(LOG_INFO, "before push_back");
	vec.push_back(test);
	node_nv.push_back(node_name); 
	syslog(LOG_INFO,"node_nv[0]:",node_nv[1].c_str());
	syslog(LOG_INFO, "after push_back");
	slp_tsk();
}