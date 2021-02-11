#include <Arduino.h>
#include "KISS.h"

#define APRS_CONTROL_FIELD 0x03
#define APRS_INFORMATION_FIELD 0xf0

String encode_kiss(const String& tnc2FormattedFrame);

String decode_kiss(String kissFormattedFrame);
