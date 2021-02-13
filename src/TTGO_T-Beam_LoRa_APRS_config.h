// Tracker for LoRA APRS
// from OE1ACM and OE3CJB redesigned by SQ9MDD
// TTGO T-Beam v1.0 only
//
// licensed under CC BY-NC-SA

// USER DATA - USE THESE LINES TO MODIFY YOUR PREFERENCES
#define KISS_PROTOCOLL                                  // If enabled send and receive data in SIMPLE KISS format to serial port
#define CALLSIGN "SQ9MDD-11"                            // enter your callsign here - less then 6 letter callsigns please add "spaces" so total length is 6 (without SSID)
#define DIGI_PATH "WIDE1-1"                             // one hope please (WIDE1-1)
#define APRS_SYMBOL_TABLE "/"
#define APRS_SYMBOL "["                                 // other symbols are: "[" => RUNNER, "b" => BICYCLE, "<" => MOTORCYCLE, "R" => Recreation Vehicle
#define MY_COMMENT "test trakera LORA"                  // add your coment here - if empty then no comment is sent
//#define SHOW_ALT                                        // send Altitude in frame
#define SHOW_BATT                                       // send battery voltage at the end of comment (we need beggining for QSY message format)
#define SHOW_RX_PACKET                                  // uncomment to show received LoRa APS packets for the time given below
#define SHOW_RX_TIME 30000                              // show RX packet for milliseconds (5000 = 5secs)
#define TXFREQ  433.775                                 // Transmit frequency in MHz
#define TXdbmW  10                                      // Transmit power in dBm         17-50mW, 18-63mW, 19-80mW, 20-100mW
//#define SHOW_GPS_DATA                                 // uncomment to show on serial port, received data from GPS and debug information
#define ENABLE_BLUETOOTH
//#define BLUETOOTH_PIN "0000"
#define KISS_DEBUG
#define T_BEAM_V1_0                                     // if enabled t-beam v1.0 disabled t-beam V.0.7

unsigned long max_time_to_nextTX = 180000L;   // TRANSMIT INTERVAL set here MAXIMUM time in ms(!) for smart beaconing - minimum time is always 1 min = 60 secs = 60000L !!!