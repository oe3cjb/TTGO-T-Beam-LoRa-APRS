#include <taskGPS.h>

// Pins for GPS
static const int RXPin = 12, TXPin = 34;  //  changed BG A3 A2
static const uint32_t GPSBaud = 9600; //GPS
HardwareSerial gpsSerial(1);        // TTGO has HW serial
TinyGPSPlus gps;             // The TinyGPS++ object
bool gpsInitialized = false;

void taskGPS(void *parameter) {
  if (!gpsInitialized){
    gpsSerial.begin(GPSBaud, SERIAL_8N1, TXPin, RXPin);        //Startup HW serial for GPS
  }
  for (;;) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
