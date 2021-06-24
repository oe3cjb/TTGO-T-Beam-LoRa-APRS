#include "taskTNC.h"


#ifdef ENABLE_BLUETOOTH
  BluetoothSerial SerialBT;
#endif

QueueHandle_t tncReceivedQueue = nullptr;
#ifdef ENABLE_WIFI
  #define MAX_WIFI_CLIENTS 6
  WiFiClient * clients[MAX_WIFI_CLIENTS];
#endif
#ifdef ENABLE_WIFI
  #define IN_TNC_BUFFERS (2+MAX_WIFI_CLIENTS)
#else
  #define IN_TNC_BUFFERS 2
#endif

String inTNCDataBuffers[IN_TNC_BUFFERS];

QueueHandle_t tncToSendQueue = nullptr;


/**
 * Handle incoming TNC KISS data character
 * @param character
 */
void handleKISSData(char character, int bufferIndex) {
  String *inTNCData = &inTNCDataBuffers[bufferIndex];
  if (inTNCData->length() == 0 && character != (char) FEND){
    // kiss frame begins with C0
    return;
  }
  inTNCData->concat(character);
  if (character == (char) FEND && inTNCData->length() > 3) {
    const String &TNC2DataFrame = decode_kiss(*inTNCData);

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
        }, &inTNCData, clients, MAX_WIFI_CLIENTS);
      #endif
    #endif
    auto *buffer = new String();
    buffer->concat(TNC2DataFrame);
    if (xQueueSend(tncToSendQueue, &buffer, (1000 / portTICK_PERIOD_MS)) != pdPASS){
      delete buffer;
    }
    inTNCData->clear();
  }
  if (inTNCData->length() > 255){
    // just in case of garbage input reset data
    inTNCData->clear();
  }
}


[[noreturn]] void taskTNC(void *parameter) {
  tncToSendQueue = xQueueCreate(4,sizeof(String *));
  tncReceivedQueue = xQueueCreate(4,sizeof(String *));
  String *loraReceivedFrameString = nullptr;

  while (true) {
    while (Serial.available() > 0) {
      char character = Serial.read();
      handleKISSData(character, 0);
    }
    #ifdef ENABLE_BLUETOOTH
      if (SerialBT.hasClient()) {
        while (SerialBT.available() > 0) {
          char character = SerialBT.read();
          handleKISSData(character, 1);
        }
      }
    #endif
    #ifdef ENABLE_WIFI
      check_for_new_clients(&tncServer, clients, MAX_WIFI_CLIENTS);

      iterateWifiClients([](WiFiClient * client, int clientIdx, const String * unused){
        while (client->available() > 0) {
          char character = client->read();
          handleKISSData(character, 2+clientIdx);
        }
      }, nullptr, clients, MAX_WIFI_CLIENTS);

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
        iterateWifiClients([](WiFiClient *client, int clientIdx, const String *data){
          if (client->connected()){
            client->print(*data);
            client->flush();
          }
        }, &kissEncoded, clients, MAX_WIFI_CLIENTS);
      #endif

      delete loraReceivedFrameString;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

