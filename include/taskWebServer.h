#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

#ifndef TASK_WEBSERVER
#define TASK_WEBSERVER


#ifdef KISS_PROTOCOL
  extern WiFiServer tncServer;
#endif
typedef struct {
  String callsign;
} tWebServerCfg;

[[noreturn]] void taskWebServer(void *parameter);
#endif