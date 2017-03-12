/*
 * main.cpp
 *
 * Webserver (Mongoose)
 *
 * Author: JochenAlt
 */

#include "mongoose.h" // mongoose MUST be first include, do not know why

#include "core.h"
#include "CmdDispatcher.h"
#include "Util.h"
#include "setup.h"

#include "logger.h"
INITIALIZE_EASYLOGGINGPP

static struct mg_serve_http_opts s_http_server_opts;

#include <stdlib.h>
#include <ctype.h>


// called when ^C is pressed
void signalHandler(int s){
	cout << "Signal " << s << ". Exiting";
	cout.flush();
	exit(1);
}

void setupLogger() {
	// setup logger
	el::Configurations defaultConf;
    defaultConf.setToDefault();
    defaultConf.set(el::Level::Error,el::ConfigurationType::Format, "%datetime %level [%func] [%loc] %msg");
    defaultConf.set(el::Level::Error, el::ConfigurationType::Filename, "logs/walter.log");

    defaultConf.set(el::Level::Info,el::ConfigurationType::Format, "%datetime %level %msg");
    defaultConf.set(el::Level::Info, el::ConfigurationType::Filename, "logs/walter.log");

    defaultConf.set(el::Level::Debug, el::ConfigurationType::ToStandardOutput,std::string("false"));
    // defaultConf.set(el::Level::Debug, el::ConfigurationType::Enabled,std::string("false"));

    defaultConf.set(el::Level::Debug, el::ConfigurationType::Format, std::string("%datetime %level [%func] [%loc] %msg"));
    defaultConf.set(el::Level::Debug, el::ConfigurationType::Filename, "logs/walter.log");

    // logging from uC is on level Trace
    defaultConf.set(el::Level::Trace, el::ConfigurationType::ToStandardOutput,std::string("false"));
    defaultConf.set(el::Level::Trace, el::ConfigurationType::Format, std::string("%datetime %level [uC] %msg"));
    defaultConf.set(el::Level::Trace, el::ConfigurationType::Filename, "logs/walter.log");

    el::Loggers::reconfigureLogger("default", defaultConf);

	LOG(INFO) << "Walter Website Setup";
}


// Define an event handler function
static void ev_handler(struct mg_connection *nc, int ev, void *ev_data)
{
    switch (ev)
    {
    	case MG_EV_HTTP_REQUEST: {
    			struct http_message *hm = (struct http_message *) ev_data;
    			string uri(hm->uri.p, hm->uri.len);
    			string query(hm->query_string.p, hm->query_string.len);
    	        string body(hm->body.p, hm->body.len);

    	        bool ok;
    			string response;
    			// if our dispatcher knows the command, it creates a response and returns true.
    			// Otherwise assume that we deliver static content.
    			bool processed = CommandDispatcher::getInstance().dispatch(uri, query, body, response, ok);
    			if (processed) {
    				if (ok) {
    					mg_printf(nc, "HTTP/1.1 200 OK\r\n"
    						"Content-Type: text/plain\r\n"
    						"Content-Length: %d\r\n\r\n%s",
    							(int) response.length(), response.c_str());
    				} else {
    					mg_printf(nc, "HTTP/1.1 500 Server Error\r\n"
    							"Content-Length: %d\r\n\r\n%s",
    							(int) response.length(), response.c_str());
    				}
    			} else {
    				// no API call, serve static content
    				mg_serve_http(nc, (http_message*) ev_data, s_http_server_opts);
    			}
    		break;
    	}
    default:
        break;
    }
}


int main(void) {
	struct mg_mgr mgr;
	struct mg_connection *nc = NULL;
	cs_stat_t st;

	mg_mgr_init(&mgr, NULL);


	// if we run on jochens notebook, use a different port than on Linux
	// (where the machine is occupied completely and we can use 8000)
#ifdef _WIN32
	#define SERVER_PORT WIN_WEB_SERVER_PORT
#else
	#define SERVER_PORT LINUX_WEB_SERVER_PORT
#endif


	string serverport_s = int_to_string(SERVER_PORT);
	nc = mg_bind(&mgr, serverport_s.c_str(), ev_handler);
	if (nc == NULL) {
		LOG(ERROR) << "Cannot bind to " << SERVER_PORT;
		exit(1);
	}

	// Set up HTTP server parameters
	mg_set_protocol_http_websocket(nc);
	s_http_server_opts.document_root = "web_root"; // Set up web root directory
	if (mg_stat(s_http_server_opts.document_root, &st) != 0) {
		LOG(ERROR) << "Cannot find web_root directory, exiting.";
		exit(1);
	}

	// catch SIGINT (ctrl-C)
    signal (SIGINT,signalHandler);

	// log into logs/walter.log
	setupLogger();

	// initialize kinematics and trajectory compilation
	Kinematics::getInstance().setup();

	// initialize communication to cortex
	bool cortexOk = false;

	LOG(INFO) << "Walter's webserver running on port " << SERVER_PORT;

	// ToDo change this loop to two threads running on different cores
	int lastTimeCortexSetup = millis();
	while (true) {
		if (cortexOk)
			TrajectoryExecution::getInstance().loop();
		else {
			if (millis()> lastTimeCortexSetup+1000 ) {
				cortexOk = TrajectoryExecution::getInstance().setup(CortexSampleRate);
				lastTimeCortexSetup = millis();
				if (cortexOk) {
					LOG(INFO) << "Cortex initialized successfully";
				} else {
					string error = getLastErrorMessage();
					LOG(ERROR) << "Communication with cortex failed (" << error.c_str();
					CommandDispatcher::getInstance().addAlert("communication with Walters cortex failed");
				}
			}
		}
		mg_mgr_poll(&mgr, 10); // check every 10ms for incoming requests
	}
	mg_mgr_free(&mgr);

	return 0;
}

/*


// central event handler of mongoose, all kinds of request go here
static void ev_handler(struct mg_connection *nc, int ev, void *ev_data) {
	struct http_message *hm = (struct http_message *) ev_data;
	switch (ev) {
	case MG_EV_HTTP_REQUEST: {
			string uri(hm->uri.p, hm->uri.len);
			string query(hm->query_string.p, hm->query_string.len);
	        string body(hm->body.p, hm->body.len);

	        bool ok;
			string response;
			// if our dispatcher knows the command, it creates a response and returns true.
			// Otherwise assume that we deliver static content.
			bool processed = CommandDispatcher::getInstance().dispatch(uri, query, body, response, ok);
			if (processed) {
				if (ok) {
					mg_printf(nc, "HTTP/1.1 200 OK\r\n"
						"Content-Type: text/plain\r\n"
						"Content-Length: %d\r\n\r\n%s",
							(int) response.length(), response.c_str());
				} else {
					mg_printf(nc, "HTTP/1.1 500 Server Error\r\n"
							"Content-Length: %d\r\n\r\n%s",
							(int) response.length(), response.c_str());
				}
			} else {
				// no API call, serve static content
				mg_serve_http(nc, (http_message*) ev_data, s_http_server_opts);
			}
		break;
	}
	default:
		break;
	}
}

int main(void) {
	struct mg_mgr mgr;
	struct mg_connection *nc;
	cs_stat_t st;

	mg_mgr_init(&mgr, NULL);
	nc = mg_bind(&mgr, int_to_string(SERVER_PORT).c_str(), ev_handler);
	if (nc == NULL) {
		fprintf(stderr, "Cannot bind to %i\n", SERVER_PORT);
		exit(1);
	}

	// Set up HTTP server parameters
	mg_set_protocol_http_websocket(nc);
	s_http_server_opts.document_root = "web_root"; // Set up web root directory

	if (mg_stat(s_http_server_opts.document_root, &st) != 0) {
		fprintf(stderr, "%s", "Cannot find web_root directory, exiting\n");
		exit(1);
	}

	// catch SIGINT (ctrl-C)
    signal (SIGINT,signalHandler);

	// log into logs/walter.log
	setupLogger();

	// initialize kinematics and trajectory compilation
	Kinematics::getInstance().setup();

	// initialize communication to cortex
	bool ok = TrajectoryExecution::getInstance().setup(CortexSampleRate);
	if (!ok) {
		string error = getLastErrorMessage();
		printf("Communication with cortex failed (\"%s\"). No access to Walter's cortex.\n", error.c_str());
		CommandDispatcher::getInstance().addAlert("communication with Walters cortex failed");
	}

	printf("webserver running on port %i\n", SERVER_PORT);

	// ToDo change this loop to two threads running on different cores
	LOG(DEBUG) << "starting webservers main loop";
	while (true) {
		TrajectoryExecution::getInstance().loop();
		mg_mgr_poll(&mgr, 100); // check every 10ms for incoming requests
	}
	mg_mgr_free(&mgr);

	return 0;
}
*/
