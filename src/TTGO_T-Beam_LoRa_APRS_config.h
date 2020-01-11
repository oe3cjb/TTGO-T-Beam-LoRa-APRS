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
// #define T_BEAM_V1_0    // use this for newer Boards AKA Rev1 (second board release)
#define T_BEAM_V0_7    // use this for older Boards AKA Rev0.x (first board release)

// USER DATA - USE THESE LINES TO MODIFY YOUR PREFERENCES
// IF NOT CHANGED you have to go through the configuration routine at first boot up of the TTGO T-Beam

// #define DONT_USE_FLASH_MEMORY   // uncomment if you don't want to use Flashmemory - instead data below must be corrected

#define CALLSIGN "OE1XYZ-0"     // enter your callsign here - less then 6 letter callsigns please add "spaces" so total length is 6 (without SSID)
#define WX_CALLSIGN "OE1XYZ-0"  // use same callsign but you can use different SSID
#define LONGITUDE_PRESET "01539.85E" // please in APRS notation DDDMM.mmE or DDDMM.mmW
#define LATIDUDE_PRESET "4813.62N"   // please in APRS notation DDMM.mmN or DDMM.mmS
#define APRS_SYMBOL ">"         // other symbols are
                                // "_" => Weather Station
                                // ">" => CAR
                                // "[" => RUNNER
                                // "b" => BICYCLE
                                // "<" => MOTORCYCLE

// TRANSMIT INTERVAL
unsigned long max_time_to_nextTX = 300000L;   // set here MAXIMUM time in ms(!) for smart beaconing - minimum time is always 1 min = 60 secs = 60000L !!!
                                // when entering 60000L intervall is fixed to 1 min
