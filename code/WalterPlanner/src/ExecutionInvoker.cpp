/*
 * ExecutionInvoker.cpp
 *
 * Author: JochenAlt
 */

#include "core.h"

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
#include "spatial.h"
#include "Trajectory.h"
#include "logger.h"

ExecutionInvoker executionInvoker;

using namespace Poco;
using namespace Net;
using namespace std;


bool ExecutionInvoker::httpGET(string path, string &responsestr, int timeout_ms) {
	std::ostringstream address;
	address << "http://" << host << ":" << port;
	if (path.find("/") != 0)
		address << "/";
	address << path;

	LOG(DEBUG) << "calling " << address.str();

	URI uri(address.str());
    std::string pathandquery(uri.getPathAndQuery());
    if (pathandquery.empty())
    	pathandquery = "/";

    HTTPClientSession session(uri.getHost(), uri.getPort());
    HTTPRequest request(HTTPRequest::HTTP_GET, pathandquery, HTTPMessage::HTTP_1_1);
    HTTPResponse response;

    session.setTimeout(Timespan(timeout_ms/1000,timeout_ms%1000)); // = 3s 0ms
    try {
    	session.sendRequest(request);
    	std::istream& rs = session.receiveResponse(response);
        string line;
        responsestr = "";
        while(std::getline(rs, line))
        	responsestr += line + "\r\n";

    	return (response.getStatus() == HTTPResponse::HTTP_OK);
    }
    catch (Poco::TimeoutException ex) {
    	setError(WEBSERVER_TIMEOUT);
    	LOG(DEBUG) << "request timeout " << ex.name();
    	std::ostringstream s;
    	s << "NOK(" << WEBSERVER_TIMEOUT << ") " << getLastErrorMessage();
    	responsestr = s.str();
    	return false;
    }
}


bool ExecutionInvoker::httpPOST(string path, string body, string &responsestr, int timeout_ms) {
	std::ostringstream address;
	address << "http://" << host << ":" << port;
	if (path.find("/") != 0)
		address << "/";
	address << path;

	URI uri(address.str());
    std::string pathandquery(uri.getPathAndQuery());
    if (pathandquery.empty())
    	pathandquery = "/";

    HTTPClientSession session(uri.getHost(), uri.getPort());
    HTTPRequest request(HTTPRequest::HTTP_POST, pathandquery, HTTPMessage::HTTP_1_1);
    request.setContentType("application/x-www-form-urlencoded");
    request.setKeepAlive(true); // notice setKeepAlive is also called on session (above)
    request.add("Content-Length", int_to_string((int)body.size()));

    HTTPResponse response;

    session.setTimeout(Timespan(timeout_ms/1000,timeout_ms%1000)); // = 3s 0ms
    try {
    	std::ostream& bodyOStream = session.sendRequest(request);
    	bodyOStream << body;  // sends the body
   	    std::istream& rs = session.receiveResponse(response);

    	string line;
    	responsestr = "";
    	while(std::getline(rs, line))
    		responsestr += line + "\r\n";

    	return (response.getStatus() == HTTPResponse::HTTP_OK);
    }
    catch (Poco::TimeoutException ex) {
      	setError(WEBSERVER_TIMEOUT);
      	LOG(DEBUG) << "request timeout " << ex.name();
      	std::ostringstream s;
      	s << "NOK(" << WEBSERVER_TIMEOUT << ") " << getLastErrorMessage();
      	responsestr = s.str();
      	return false;
     }
}


void ExecutionInvoker::setHost(string pHost, int pPort) {
	host = pHost;
	port = pPort;
}

ExecutionInvoker::ExecutionInvoker() {
	Poco::Net::initializeNetwork();
}

ExecutionInvoker& ExecutionInvoker::getInstance() {
	return executionInvoker;
}

bool ExecutionInvoker::startupBot() {
	string response;
	bool ok = httpGET("/executor/startupbot", response,10000);
	// no response expected
	return ok;
}

bool ExecutionInvoker::teardownBot() {
	string response;
	bool ok = httpGET("/executor/teardownbot", response,50000);
	// no response expected
	return ok;
}

bool ExecutionInvoker::isBotUpAndRunning() {
	string response;
	bool ok = httpGET("/executor/isupandrunning", response,500);
	return (ok && (response.find("true") == 0));
}

bool ExecutionInvoker::setAngles(JointAngles angles) {
	string response;
	int indent = 0;
	string anglesAsString = angles.toString(indent);
	std::ostringstream request;
	request << "/executor/setangles?param=" << urlEncode(anglesAsString);
	bool ok = httpGET(request.str(), response,200);
	return (ok && (response.find("OK") == 0));
}

TrajectoryNode ExecutionInvoker::getAngles() {
	TrajectoryNode node;
	string response;
	bool okHttp = httpGET("/executor/getangles", response,200);
	if (okHttp) {
		int idx = 0;
		bool ok = node.fromString(response, idx);
		if (!ok)
			node.null();
	}
	return node;
}

bool ExecutionInvoker::runTrajectory(Trajectory traj) {
	int indent = 0;
	string trajectoryStr = traj.toString(indent);
	string response;
	string bodyMessage = urlEncode(trajectoryStr);
	bool ok = httpPOST("/executor/settrajectory", bodyMessage, response, 5000);
	return (ok && (response.find("OK") == 0));
}

bool ExecutionInvoker::stopTrajectory() {
	string response;
	bool ok = httpGET("/executor/stoptrajectory", response, 1000);
	return (ok && (response.find("OK") == 0));
}

string ExecutionInvoker::directAccess(string directCommand,string &response) {
	std::ostringstream request;
	request << "/direct/cmd?param=" << urlEncode(directCommand);
	httpGET(request.str(), response, 3000);
	return response;
}



