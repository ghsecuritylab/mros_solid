#ifndef _ROS_H_
#define _ROS_H_

#include <string>
#include <vector>

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