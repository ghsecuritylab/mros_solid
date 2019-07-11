#ifndef _XMLCALL_H_
#define _XMLCALL_H_

#include <string>
#include <sstream>

#include <stdint.h>
#include <stdio.h>
#include <vector>
#include <stdlib.h>

//master API
/*
@param
    id => caller_id(caller name)
    srv => service name
    s_uri => service URI
    c_uri => caller URI
    topic => topic name
    type => topic type
*/
std::string registerPublisher(std::string id, std::string topic, std::string type, std::string c_uri);

//local function
std::string addHttpPost(std::string xml);
std::string makexmlcall(std::string name, std::vector<std::string> params, int pnum);

#endif /*_XMLCALL_H_*/