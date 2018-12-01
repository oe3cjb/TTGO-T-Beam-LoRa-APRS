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
