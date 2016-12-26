/*
 * main.cpp
 *
 * Webserver main
 *
 *  Created on: 29.08.2016
 *      Author: JochenAlt
 */


#include "mongoose.h"
#include "CmdDispatcher.h"
#include "Util.h"

INITIALIZE_EASYLOGGINGPP

static const char *s_http_port = "8000";
static const struct mg_str s_get_method = MG_MK_STR("GET");

static struct mg_serve_http_opts s_http_server_opts;

// true if uri starts with prefix
static int has_prefix(const struct mg_str *uri, const struct mg_str *prefix) {
  return (uri->len > prefix->len) && memcmp(uri->p, prefix->p, prefix->len) == 0;
}

// guess
static int is_equal(const struct mg_str *s1, const struct mg_str *s2) {
  return s1->len == s2->len && memcmp(s1->p, s2->p, s2->len) == 0;
}

static void ev_handler(struct mg_connection *nc, int ev, void *ev_data) {
	static const struct mg_str api_prefix = MG_MK_STR("/api");
	struct http_message *hm = (struct http_message *) ev_data;

	if (ev == MG_EV_HTTP_REQUEST) {
      if (has_prefix(&hm->uri, &api_prefix)) {
        if (is_equal(&hm->method, &s_get_method)) {
        	string uri(hm->uri.p, hm->uri.len);
        	string query(hm->query_string.p, hm->query_string.len);

        	bool ok;
        	string response;
        	CommandDispatcher::getInstance().dispatch(uri.substr(api_prefix.len+1), query, response, ok);
        	if (ok) {
				mg_printf(nc, "HTTP/1.1 200 OK\r\n"
						  "Content-Type: text/plain\r\n"
						  "Content-Length: %d\r\n\r\n%s",(int)response.length(), response.c_str());
        	} else {
        	    mg_printf(nc, "HTTP/1.1 500 Server Error\r\n"
        	              "Content-Length: %d\r\n\r\n%s",(int)response.length(), response.c_str());
        	}
          } else {
            mg_printf(nc, "HTTP/1.1 404 Not Found\r\n"
                      "Content-Length: %d\r\n\r\n%s",(int)hm->method.len, hm->method.p);
          }
        } else {
        	// not api call, serve static content
        	mg_serve_http(nc, (http_message*)ev_data, s_http_server_opts);
        }
	}
}

int main(void) {
  struct mg_mgr mgr;
  struct mg_connection *nc;
  cs_stat_t st;

  mg_mgr_init(&mgr, NULL);
  nc = mg_bind(&mgr, s_http_port, ev_handler);
  if (nc == NULL) {
    fprintf(stderr, "Cannot bind to %s\n", s_http_port);
    exit(1);
  }

  // Set up HTTP server parameters
  mg_set_protocol_http_websocket(nc);
  s_http_server_opts.document_root = "web_root";  // Set up web root directory

  if (mg_stat(s_http_server_opts.document_root, &st) != 0) {
    fprintf(stderr, "%s", "Cannot find web_root directory, exiting\n");
    exit(1);
  }

  bool ok = CommandDispatcher::getInstance().setup();
  if (!ok)
	  printf("Initialization failed. No access to Walters cortex.");


  printf("webserver running on port %s\n", s_http_port);

  for (;;) {
    mg_mgr_poll(&mgr, 1000);
  }
  mg_mgr_free(&mgr);

  return 0;
}
