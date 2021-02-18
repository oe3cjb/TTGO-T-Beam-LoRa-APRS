#include "taskTNC.h"

#ifdef ENABLE_BLUETOOTH
  BluetoothSerial SerialBT;
#endif
String inTNCData = "";
QueueHandle_t tncToSendQueue = nullptr;
QueueHandle_t tncReceivedQueue = nullptr;
WiFiClient client;

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


[[noreturn]] void taskTNC(void *parameter) {
  tncToSendQueue = xQueueCreate(4,sizeof(String *));
  tncReceivedQueue = xQueueCreate(4,sizeof(String *));
  String *loraReceivedFrameString = nullptr;
  client = tncServer.available();
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
      if (!client.connected()){
        client = tncServer.available();
      }
      if (client.connected()){
        while (client.available() > 0) {
          char character = client.read();
          handleKISSData(character);
        }
      }
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
        if (client.connected()){
          client.print(kissEncoded);
          client.flush();
        }
      #endif

      delete loraReceivedFrameString;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

