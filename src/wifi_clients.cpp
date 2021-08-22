//
// Created by Admin on 11.06.2021.
//

#ifdef ENABLE_WIFI

#include <wifi_clients.h>

#include "wifi_clients.h"

void iterateWifiClients(f_connectedClientCallback_t callback, const String *data, WiFiClient * wifiClients[], int maxWifiClients){
  for (int i=0; i < maxWifiClients; i++) {
    auto client = wifiClients[i];
    if (client != nullptr) {
      if (client->connected()) {
        callback(client, i, data);
      } else {
        #ifdef ENABLE_WIFI_CLIENT_DEBUG
        Serial.println(String("Disconnected client ") + client->remoteIP().toString() + ":" + client->remotePort());
        #endif
        delete client;
        wifiClients[i] = nullptr;
      }
    }
  }
}

void check_for_new_clients(WiFiServer *wiFiServer, WiFiClient *wifiClients[], int maxWifiClients) {
  WiFiClient new_client = wiFiServer->available();
  if (new_client.connected()){
    bool new_client_handled = false;
    for (int i=0; i < maxWifiClients; i++) {
      auto client = wifiClients[i];
      if (client == nullptr) {
        client = new WiFiClient(new_client);
        wifiClients[i] = client;
        new_client_handled = true;
        #ifdef ENABLE_WIFI_CLIENT_DEBUG
        Serial.println(String("New client #") +String(i) + ": " + client->remoteIP().toString() + ":" + client->remotePort());
        #endif
        break;
      }
    }
    #ifdef ENABLE_WIFI_CLIENT_DEBUG
    for (int i = 0; i < maxWifiClients; ++i) {
            auto client = clients[i];

            if (client != nullptr){
              Serial.println(String("Client #") +String(i) + ": " + client->remoteIP().toString() + ":" + client->remotePort());
            }
          }
    #endif


    if (!new_client_handled){
      #ifdef ENABLE_WIFI_CLIENT_DEBUG
      Serial.println(String("Refusing client "));
      #endif
      new_client.stop();
    }
  }
}

#endif
