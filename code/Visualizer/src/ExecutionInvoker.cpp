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
#include "spatial.h"
#include "Trajectory.h"

ExecutionInvoker executionInvoker;

using namespace Poco;
using namespace Net;
using namespace std;


bool ExecutionInvoker::httpGET(string path, string &responsestr) {
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
    HTTPRequest request(HTTPRequest::HTTP_GET, pathandquery, HTTPMessage::HTTP_1_1);
    HTTPResponse response;

    session.sendRequest(request);
    std::istream& rs = session.receiveResponse(response);

    string line;
    responsestr = "";
    while(std::getline(rs, line))
    	responsestr += line + "\r\n";

    std::cout << response.getStatus() << " " << response.getReason() << std::endl;
	return (response.getStatus() == HTTPResponse::HTTP_OK);
}


bool ExecutionInvoker::httpPOST(string path, string body, string &responsestr) {
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
    HTTPResponse response;

    std::ostream& bodyOStream = session.sendRequest(request);
    bodyOStream << body;  // sends the body

    std::istream& rs = session.receiveResponse(response);

    string line;
    responsestr = "";
    while(std::getline(rs, line))
    	responsestr += line + "\r\n";

    std::cout << response.getStatus() << " " << response.getReason() << std::endl;
	return (response.getStatus() == HTTPResponse::HTTP_OK);
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
	bool ok = httpGET("/executor/startupbot", response);
	// no response expected
	return ok;
}

bool ExecutionInvoker::teardownBot() {
	string response;
	bool ok = httpGET("/executor/teardownbot", response);
	// no response expected
	return ok;
}

bool ExecutionInvoker::isBotUpAndRunning() {
	string response;
	bool ok = httpGET("/executor/isupandrunning", response);
	return (ok && (response.compare("true") == 0));
}

bool ExecutionInvoker::setAngles(JointAngles angles) {
	string response;
	string anglesAsString = angles.toString();
	std::ostringstream request;
	request << "/executor/setangles?param=" << urlEncode(anglesAsString);
	bool ok = httpGET(request.str(), response);
	return (ok && (response.compare("OK") == 0));
}

TrajectoryNode ExecutionInvoker::getAngles() {
	TrajectoryNode node;
	string response;
	bool okHttp = httpGET("/executor/getangles", response);
	if (okHttp) {
		int idx = 0;
		bool ok = node.fromString(response, idx);
		if (!ok)
			node.null();
	}
	return node;
}

bool ExecutionInvoker::runTrajectory(Trajectory traj) {
	string trajectoryStr = traj.toString();
	string response;
	string bodyMessage = urlEncode(trajectoryStr);
	bool ok = httpPOST("/executor/settrajectory", bodyMessage, response);
	return (ok && (response.compare("OK") == 0));
}

bool ExecutionInvoker::stopTrajectory() {
	string response;
	bool ok = httpGET("/executor/stoptrajectory", response);
	return (ok && (response.compare("OK") == 0));
}

string ExecutionInvoker::directAccess(string directCommand,string reponse) {
	TrajectoryNode node;
	string response;
	std::ostringstream request;
	request << "/direct/cmd?param=" << urlEncode(directCommand);
	httpGET(request.str(), response);
	return response;
}



