#include "taskTNC.h"


#ifdef ENABLE_BLUETOOTH
  BluetoothSerial SerialBT;
#endif
String inTNCData = "";
QueueHandle_t tncToSendQueue = nullptr;
QueueHandle_t tncReceivedQueue = nullptr;
#ifdef ENABLE_WIFI
  #define MAX_WIFI_CLIENTS 6
  WiFiClient * clients[MAX_WIFI_CLIENTS];

  typedef void (*f_connectedClientCallback_t) (WiFiClient *, const String *);
  
  void iterateWifiClients(f_connectedClientCallback_t callback, const String *data){
    for (int i=0; i<MAX_WIFI_CLIENTS; i++) {
      auto client = clients[i];
      if (client != nullptr) {
        if (client->connected()) {
          callback(client, data);
        } else {
          #ifdef ENABLE_WIFI_CLIENT_DEBUG
            Serial.println(String("Disconnected client ") + client->remoteIP().toString() + ":" + client->remotePort());
          #endif
          delete client;
          clients[i] = nullptr;
        }
      }
    }
  }
#endif


/**
 * Handle incoming TNC KISS data character
 * @param character
 */
void handleKISSData(char character) {
  inTNCData.concat(character);
  if (character == (char) FEND && inTNCData.length() > 3) {
    const String &TNC2DataFrame = decode_kiss(inTNCData);

    #ifdef LOCAL_KISS_ECHO
      Serial.print(inTNCData);
      #ifdef ENABLE_BLUETOOTH
        if (SerialBT.hasClient()) {
          SerialBT.print(inTNCData);
        }
      #endif
      #ifdef ENABLE_WIFI
        iterateWifiClients([](WiFiClient *client, const String *data){
          if (client->connected()){
            client->print(*data);
            client->flush();
          }
        }, &inTNCData);
      #endif
    #endif
    auto *buffer = new String();
    buffer->concat(TNC2DataFrame);
    if (xQueueSend(tncToSendQueue, &buffer, (1000 / portTICK_PERIOD_MS)) != pdPASS){
      delete buffer;
    }
    inTNCData = "";
  }
}


[[noreturn]] void taskTNC(void *parameter) {
  tncToSendQueue = xQueueCreate(4,sizeof(String *));
  tncReceivedQueue = xQueueCreate(4,sizeof(String *));
  String *loraReceivedFrameString = nullptr;

  while (true) {
    while (Serial.available() > 0) {
      char character = Serial.read();
      handleKISSData(character);
    }
    #ifdef ENABLE_BLUETOOTH
      if (SerialBT.hasClient()) {
        while (SerialBT.available() > 0) {
          char character = SerialBT.read();
          handleKISSData(character);
        }
      }
    #endif
    #ifdef ENABLE_WIFI
      WiFiClient new_client = tncServer.available();
      if (new_client.connected()){
        bool new_client_handled = false;
        for (int i=0; i < MAX_WIFI_CLIENTS; i++) {
          auto client = clients[i];
          if (client == nullptr) {
            client = new WiFiClient(new_client);
            clients[i] = client;
            new_client_handled = true;
            #ifdef ENABLE_WIFI_CLIENT_DEBUG
              Serial.println(String("New client #") +String(i) + ": " + client->remoteIP().toString() + ":" + client->remotePort());
            #endif
            break;
          }
        }
        #ifdef ENABLE_WIFI_CLIENT_DEBUG
          for (int i = 0; i < MAX_WIFI_CLIENTS; ++i) {
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
      iterateWifiClients([](WiFiClient * client, const String * unused){
        while (client->available() > 0) {
          char character = client->read();
          handleKISSData(character);
        }
      }, nullptr);

    #endif
    if (xQueueReceive(tncReceivedQueue, &loraReceivedFrameString, (1 / portTICK_PERIOD_MS)) == pdPASS) {
      const String &kissEncoded = encode_kiss(*loraReceivedFrameString);
      Serial.print(kissEncoded);
      #ifdef ENABLE_BLUETOOTH
        if (SerialBT.hasClient()){
          SerialBT.print(kissEncoded);
        }
      #endif
      #ifdef ENABLE_WIFI
        iterateWifiClients([](WiFiClient *client, const String *data){
          if (client->connected()){
            client->print(*data);
            client->flush();
          }
        }, &kissEncoded);
      #endif

      delete loraReceivedFrameString;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

