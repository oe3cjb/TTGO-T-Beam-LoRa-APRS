// Tracker for LoRA APRS Header for configuration
//
// TTGO T-Beam includes GPS module + optional DHT22 (not yet DONE)
//
// can be used as tracker only, tracker plus weather reports (temperature and humidity) or weather reports station only
//
// updated from OE1ACM sketch by OE3CJB to enable WX data to be sent via LoRa APRS.
// one package is with position and battery voltage
// the next is with weather data in APRS format
//
// licensed under CC BY-NC-SA
//
// version: V1.3
// last update: 27.08.2020
// change history
// symbol RV added
// compressed packets in tracker mode (base91)
//
// version: V1.1beta
// last update: 22.11.2019
//
// change history
// version V1.0
// added HW Version V1.0 support
// added presetting in the header TTGO...config.h to prevent long initial setup at first boot up
// added "SPACE" to allowed letters for callsign for shorter callsigns - has to be added at the end
// added smart beaconing
//
// version V1.2
// first released version

// SET HW version
#define T_BEAM_V1_0    // use this for newer Boards AKA Rev1 (second board release)
// #define T_BEAM_V0_7    // use this for older Boards AKA Rev0.x (first board release)

// SET temperature sensor type
// #define DS18B20    // use this if you use DS18B20, default is DHT22
// #define USE_BME280 // use this if you use BME280,m default is DHT22

// USER DATA - USE THESE LINES TO MODIFY YOUR PREFERENCES
// IF NOT CHANGED you have to go through the configuration routine at first boot up of the TTGO T-Beam

// #define DONT_USE_FLASH_MEMORY   // uncomment if you don't want to use Flashmemory - instead data below must be corrected
#define TRACKERMODE 0       // preset MODE here, if flash not used >> "0"=TRACKER, "1"=WX_TRACKER, "2"=WX_MOVE, "3"=WX_FIXED
#define CALLSIGN "XX9XXX-11"     // enter your callsign here - less then 6 letter callsigns please add "spaces" so total length is 6 (without SSID)
#define WX_CALLSIGN "XX9XXX-11"  // use same callsign but you can use different SSID
#define LONGITUDE_PRESET "01539.85E" // please in APRS notation DDDMM.mmE or DDDMM.mmW
#define LATIDUDE_PRESET "4813.62N"   // please in APRS notation DDMM.mmN or DDMM.mmS
#define APRS_SYMBOL ">"         // other symbols are
                                // "_" => Weather Station
                                // ">" => CAR
                                // "[" => RUNNER
                                // "b" => BICYCLE
                                // "<" => MOTORCYCLE
                                // "R" => Recreation Vehicle
// #define HW_COMMENT              // send Alt und Battery Voltage, UNcomment if you want to send it
#define MY_COMMENT "" // add your coment here - if empty then no comment is sent
// #define MY_COMMENT "TTGO by OE3CJB" // add your coment here - if empty then no comment is sent

// TRANSMIT INTERVAL
unsigned long max_time_to_nextTX = 180000L;   // set here MAXIMUM time in ms(!) for smart beaconing - minimum time is always 1 min = 60 secs = 60000L !!!
                                // when entering 60000L intervall is fixed to 1 min

// show RX values
// #define SHOW_RX_PACKET       // uncomment to show received LoRa APS packets for the time given below
#define SHOW_RX_TIME 5000       // show RX packet for milliseconds (5000 = 5secs)
