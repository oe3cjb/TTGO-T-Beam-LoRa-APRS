#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <BG_RF95.h>

#ifndef TASK_WEBSERVER
#define TASK_WEBSERVER

extern BG_RF95 rf95;
#ifdef KISS_PROTOCOL
  extern WiFiServer tncServer;
#endif
typedef struct {
  String callsign;
} tWebServerCfg;

typedef struct {
  struct tm rxTime;
  String *packet;
  int RSSI;
  int SNR;
} tReceivedPacketData;


extern QueueHandle_t webListReceivedQueue;

[[noreturn]] void taskWebServer(void *parameter);
#endif