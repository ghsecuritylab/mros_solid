/* ROSノードとしての初期化とPub/Subのクラス宣言 */

#ifndef _ROS_H_
#define _ROS_H_

#include <string>
#include <vector>
#include <kernel.h>
#include "kernel_cfg.h"
namespace ros{
	class Publisher{
		public:
			const char* topic;
			const char* node;
			char ID;
			template<class T> void publish(T& data);
			
	};

	extern "C" void init(int argc,char *argv,std::string node_name);
	class NodeHandle{
		public:
			Publisher advertise(std::string topic,int queue_size);
	};
}


#endif