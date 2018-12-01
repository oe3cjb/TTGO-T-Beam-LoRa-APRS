First of all I want to thank OE1ACM Bernd for his approval to use his code for my experiments.
He is the author of BG_RF95!

All user settings can be found in TTGO_T-Beam_LoRa_APRS.h

CHANGE CALLSIGN BEFORE UPLOADING FIRMWARE to your T-Beam!!!

You can define two different callsigns and symbols dependend on the transmitted packet.
"Tcall" is the standard callsign used for traditional APRS packets as used for trackers
"wxTcall" is the callsign used for APRS packets including a weather report (when DHT22 is used).
So you can change the SSID and make it easier to differentiate between normal and wahter packets.

"sTable" should contain the value for the primary or secondary symbol table
"sSymbol" conatins finally the symbol to be sent with the position report

"wxTable" should contain the value for the primary or secondary symbol table used for weather reports
"wxSymbol" conatins finally the symbol to be sent with the weather report 

"nextTX" is the transmit intervall in ms (milli seconds) - don't use to short intervalls as it overloads the APRS servers
60000L is a could starting point and means a intervall of 60secs or 1 minute.

Usage of the defines:
#define SEND_WX false => no weather reports are sent
#define SEND_WX true => weather reports are sent, but needs a connected DHT22 

#define FIXED_POSITION false
// set to true if you want to use fixed position (position defined below) instead, or to false if you want to use GPS data
// also stops sending normal position reports when sending weather reports is active (SEND_WX true)

the following lines are used to define the fixed position
#define LATITUDE "4813.62N"  // please in APRS notation DDMM.mmN or DDMM.mmS used for FIXED_POSITION
#define LONGITUDE "01539.85E"

