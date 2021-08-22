//
// Created by Admin on 11.06.2021.
//

#ifndef TTGO_T_BEAM_LORA_APRS_WIFI_CLIENTS_H
#define TTGO_T_BEAM_LORA_APRS_WIFI_CLIENTS_H
#include <WiFiClient.h>
#include <WiFiServer.h>

typedef void (*f_connectedClientCallback_t) (WiFiClient *, int, const String *);
void iterateWifiClients(f_connectedClientCallback_t callback, const String *data, WiFiClient * wifiClients[], int maxWifiClients);
void check_for_new_clients(WiFiServer *wiFiServer, WiFiClient * wifiClients[], int maxWifiClients);
#endif //TTGO_T_BEAM_LORA_APRS_WIFI_CLIENTS_H
