#include <Arduino.h>
#include <KISS_TO_TNC2.h>

#if defined(ENABLE_BLUETOOTH)
  #include "BluetoothSerial.h"
  extern BluetoothSerial SerialBT;
#endif
#if defined(ENABLE_WIFI)
  #include "taskWebServer.h"
#endif
extern QueueHandle_t tncToSendQueue;
extern QueueHandle_t tncReceivedQueue;

[[noreturn]] void taskTNC(void *parameter);

