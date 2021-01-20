#include <stdint.h>
#include <stdlib.h>

#ifndef KISS_H
  #define KISS_H

  #define FEND              0xC0
  #define FESC              0xDB
  #define TFEND             0xDC
  #define TFESC             0xDD

  #define CMD_UNKNOWN       0xFE
  #define CMD_DATA          0x00
  #define CMD_HARDWARE      0x06

  #define HW_RSSI           0x21
  
  #define CMD_ERROR         0x90
  #define ERROR_INITRADIO   0x01
  #define ERROR_TXFAILED    0x02
  #define ERROR_QUEUE_FULL  0x04

  size_t frameLength;
  bool inFrame                = false;
  bool escape                 = false;
  bool SERIAL_READING         = false;
  uint8_t command             = CMD_UNKNOWN;
  uint32_t lastSerialRead     = 0;
  uint32_t serialReadTimeout  = 25;

#endif