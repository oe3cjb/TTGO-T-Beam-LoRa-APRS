#include <list>
#include "taskTNC.h"

#ifdef ENABLE_BLUETOOTH
  BluetoothSerial SerialBT;
#endif
String inTNCData = "";
QueueHandle_t tncToSendQueue = nullptr;
QueueHandle_t tncReceivedQueue = nullptr;

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
        if (client.connected()){
          client.print(inTNCData);
          client.flush();
        }
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

#ifdef ENABLE_WIFI
typedef void (*f_connectedClientCallback_t) (WiFiClient *, const String *);

void iterateWifiClients(std::list<WiFiClient *> clients, f_connectedClientCallback_t callback, const String *data){
  auto clientsIterator = clients.begin();
  while (clientsIterator != clients.end()){
    if ((*clientsIterator)->connected()){
      callback(*clientsIterator, data);
      clientsIterator++;
    } else {
      clientsIterator = clients.erase(clientsIterator);
    }
  }
}
#endif


[[noreturn]] void taskTNC(void *parameter) {
  tncToSendQueue = xQueueCreate(4,sizeof(String *));
  tncReceivedQueue = xQueueCreate(4,sizeof(String *));
  String *loraReceivedFrameString = nullptr;
  #ifdef ENABLE_WIFI
    std::list<WiFiClient *> clients;
  #endif

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
        clients.push_back(new WiFiClient(new_client));
      }
      iterateWifiClients(clients, [](WiFiClient * client, const String * unused){
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
        iterateWifiClients(clients, [](WiFiClient *client, const String *data){
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

