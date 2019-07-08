#ifndef _XMLPARSER_H_
#define _XMLPARSER_H_

#include <string>
#include <iostream>
#include <vector>



/*うまいこと実装して
typedef struct xmlNode{
    string  methodName;
    vector<string> params;    
    int e_num;  
    bool fault;
    xmlNode(){
        e_num=0;
        fault = true;
    }
}xmlNode;


string getbool(string param);
string getint(string param);
string getdouble(string param);
string getstring(string param);
string getarray(string param);
string getdate(string param);
string getbinary(string param);
string getstruct(string param);
string gettagless(string param);

void paramparser(xmlNode *node,string params,int num);
void faultperser(xmlNode *node,string body);

void callparser(xmlNode *node,string xml);
void resparser(xmlNode *node,string xml);

bool parser(xmlNode *node,string xml);
int ParseReceiveMessage(string http,xmlNode *node);
*/

//mROS用の切り出し関数
//もっとうまく作って
int get_port(std::string http);
std::string get_port2(std::string http);
std::string get_ip(std::string ip);

//node情報切り出し
std::string get_ttype(std::string xml);
std::string get_tname(std::string xml);
std::string get_cid(std::string xml);
std::string get_msgdef(std::string xml);
std::string get_fptr(std::string xml);
std::string req_topic_name(std::string xml);

#endif /* _XMLPARSER_H_ */
