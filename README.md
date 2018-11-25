// one package is with position and battery voltage
// the next is with weather data in APRS format
//
// licensed under CC BY-NC-SA
//
// last update: 25.11.2018

// USER DATA - USE THESE LINES TO MODIFY YOUR PREFERENCES
// Your Callsign
String Tcall="OE3OOO-7";     //your Call Sign for normal position reports
String wxTcall="OE3OOO-7";   //your Call Sign for weather reports

// Your symbol table and symbol for position reports incl. battery voltage
String sTable="/";           //Primer
//String sTable="\";           //Alternativ

// String sSymbol="_";          //symbol code Weather Station
// String sSymbol=">";          //symbol code CAR
String sSymbol="[";          //symbol code RUNNER
// String sSymbol="b";          //symbol code BICYCLE
// String sSymbol="<";          //symbol code MOTORCYCLE

// SEND_WX - if true the tracker sends WX reports - needs DHT22 connected at Pin 10
// when FIXED_POSITION is false then it sends alternating normal position packets and weather report packets
#define SEND_WX false

// Your symbol table and symbol for weather reports
String wxTable="/";          //Primer
String wxSymbol="_";         //Symbol Code Weather Station
// String wxSymbol="W";        //Symbol Code Weather Station/

#define FIXED_POSITION false
// set to true if you want to use fixed position (position defined below) instead, or to false if you want to use GPS data
// also stops sending normal position reports when sending weather reports is active (SEND_WX true)

#define LATITUDE "4813.62N"  // please in APRS notation DDMM.mmN or DDMM.mmS used for FIXED_POSITION
#define LONGITUDE "01539.85E" // please in APRS notation DDDMM.mmE or DDDMM.mmW used for FIXED_POSITION
// ^^^^^LATITUDE and LONGITUDE only used when FIXED_POSITION is true

// Tracker setting: use these lines to modify the tracker behaviour
#define TXFREQ  433.775      // Transmit frequency in MHz
#define TXdbmW  18           // Transmit power in dBm
#define TXenablePA  0        // switch internal power amplifier on (1) or off (0)

// Transmit intervall
unsigned long nextTX = 60000L;   // Send every 60 secs
// unsigned long nextTX = 5000L; // Send every 5 secs - FOR TESTS ONLY - NO CONNECTION TO SERVER PLEASE!!!!
