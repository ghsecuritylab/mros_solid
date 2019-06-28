#include "ros.h"

#include <syslog.h>
#include <kernel.h>

#include <iostream>

std::vector<ID> IDv;
std::vector<std::string> node_nv;

void ros::init(std::string node_name)
{

	ID id;
	get_tid(&id);
	IDv.push_back(id);
	//適当にpush_back
	for (int i = 3; i < 6; i++)
	{
		IDv.push_back(i);
		syslog(LOG_INFO,"IDv[%d]:%d",i,IDv[i]);
	}
	
	 //autoは型推論
	//const auto& eを書くことでコンテナ内の要素の変更を禁止，要素のコピーも行わない
	//auto& eだとコンテナ内の要素を変更できる
	//auto eだと各要素がコピーされてからfor文に渡される．要素は変更できない

	std::cout << node_name << std::endl;

	syslog(LOG_INFO, "task ID:%d", id);
	syslog(LOG_INFO, "before push_back");
	
	node_nv.push_back(node_name);
	syslog(LOG_INFO, "node_nv[0]:%s", node_nv[0].c_str());
}