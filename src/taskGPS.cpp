#include <taskGPS.h>
#include "TTGO_T-Beam_LoRa_APRS_config.h"
#include <SparkFun_Ublox_Arduino_Library.h>

SFE_UBLOX_GPS myGPS;

// Pins for GPS
#ifdef T_BEAM_V1_0
static const int RXPin = 12, TXPin = 34;
#else
static const int RXPin = 15, TXPin = 12;
#endif
static const uint32_t GPSBaud = 9600; //GPS
HardwareSerial gpsSerial(1);        // TTGO has HW serial
TinyGPSPlus gps;             // The TinyGPS++ object
bool gpsInitialized = false;

void taskGPS(void *parameter) {
  if (!gpsInitialized){
    gpsSerial.begin(GPSBaud, SERIAL_8N1, TXPin, RXPin);        //Startup HW serial for GPS

    // set GPS parameters on restart
    // Thanks Peter (https://github.com/peterus)
    // https://github.com/lora-aprs/TTGO-T-Beam_GPS-reset
    if(myGPS.begin(gpsSerial)){
          myGPS.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output NMEA only
          //myGPS.saveConfiguration(); //Save the current settings to flash and BBR
          myGPS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
          myGPS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
          myGPS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
          myGPS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
          myGPS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
          myGPS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
          //myGPS.saveConfiguration(); //Save the current settings to flash and BBR    
          delay(1000);
    }
  }

  for (;;) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
