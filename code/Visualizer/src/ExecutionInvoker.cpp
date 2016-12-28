/*
 * ExecutionInvoker.cpp
 *
 *  Created on: 27.12.2016
 *      Author: JochenAlt
 */

#include "Poco/Net/HTTPClientSession.h"
#include "Poco/Net/HTTPRequest.h"
#include "Poco/Net/HTTPResponse.h"
#include <Poco/Net/HTTPCredentials.h>
#include "Poco/StreamCopier.h"
#include "Poco/NullStream.h"
#include "Poco/Path.h"
#include "Poco/URI.h"
#include "Poco/Exception.h"

#include <iostream>
#include <sstream> // for ostringstream
#include <string>
#include "ExecutionInvoker.h"

ExecutionInvoker executionInvoker;

using namespace Poco;
using namespace Net;
using namespace std;

bool httpGET(string uristr, string &responsestr) {
	URI uri(uristr);
    std::string path(uri.getPathAndQuery());
    if (path.empty())
    	path = "/";

    HTTPClientSession session(uri.getHost(), uri.getPort());
    HTTPRequest request(HTTPRequest::HTTP_GET, path, HTTPMessage::HTTP_1_1);
    HTTPResponse response;

    session.sendRequest(request);
    std::istream& rs = session.receiveResponse(response);
    cout << response.getStatus() << " " << response.getReason() << endl;

    string line;
    responsestr = "";
    while(std::getline(rs, line))
    	responsestr += line + "\r\n";

    std::cout << response.getStatus() << " " << response.getReason() << std::endl;
	return (response.getStatus() == HTTPResponse::HTTP_OK);
}

ExecutionInvoker::ExecutionInvoker() {
}

ExecutionInvoker& ExecutionInvoker::getInstance() {
	return executionInvoker;
}

bool ExecutionInvoker::startupBot() {
	Poco::Net::initializeNetwork();
	string response;
	// bool ok = httpGET("http://172.29.12.22:8000/exec/startupbot", response);
	std::ostringstream address;
	address << "http://" << host << ":" << port;
	bool ok = httpGET(address.str(), response);

	return ok;
}

bool ExecutionInvoker::teardownBot() {
	string response;
	bool ok = httpGET("http://172.29.12.22:8000/exec/teardownbot", response);
    return ok;
}
