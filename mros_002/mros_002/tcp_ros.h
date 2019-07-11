#ifndef _TCP_ROS_H_
#define _TCP_ROS_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <sstream>


#include "ros.h"

void add_len(char *buf, int len);
int pub_gen_header(char *buf, std::string id, std::string msg_def, std::string topic, std::string type, std::string md5);
int sub_gen_header(char *buf, std::string id, std::string nodelay, std::string topic, std::string type, std::string md5);
int pub_gen_msg(char *buf, char *msg);
int pub_gen_img_msg(char *buf, char *msgint, int size);

#endif
