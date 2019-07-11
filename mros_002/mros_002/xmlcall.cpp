#include "mros.h"
#include "xmlcall.h"

/*
template <typename T>
std::string to_string(const T &n){
	std::ostringstream stm;
	stm << n;
	return stm.str();
}
*/

std::string addHttpPost(std::string xml)
{
	/*
	std::stringstream ss;
	ss << xml.size();
	*/

	std::string size_str;

	size_str = "100";				//とりあえずビルドを通すためにごまかす
	std::string xml2;
	xml2 += "POST /RPC2 HTTP/1.1\n";
	xml2 += "Host: \n";
	xml2 += "Accept-Encoding: \n";
	xml2 += "User-Agent: \n";
	xml2 += "Content-Type: \n";
	xml2 += "Content-Length: ";
	xml2 += size_str;
	xml2 += "\n\n";
	xml2 += xml;

	std::cout << xml2 << std::endl;
	return xml2;
}

std::string makexmlcall(std::string name, std::vector<std::string> params, int pnum)
{
	std::string m;
	m += "<?xml version='1.0'?>\n";
	m += "<methodCall>\n";
	m += "<methodName>";
	m += name;
	m += "</methodName>\n";
	m += "<params>\n";
	for (int i = 0; i < pnum; i++)
	{
		m += "<param>\n<value>";
		m += params[i];
		m += "</value>\n</param>\n";
	}
	m += "</params>";
	m += "</methodCall>\n\0";
	return m;
}

std::string registerPublisher(std::string id, std::string topic, std::string type, std::string c_uri)
{
	std::string xml;
	std::vector<std::string> params;
	params.push_back(id);
	params.push_back(topic);
	params.push_back(type);
	params.push_back(c_uri);
	xml = makexmlcall("registerPublisher", params, 4);
	xml = addHttpPost(xml);
	return xml;
}