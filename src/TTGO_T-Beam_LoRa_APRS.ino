
// Tracker for LoRA APRS
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
// version: V1.2
// last update: 02.01.2020
// change history
// added course change to smart Beaconing
// code cleaned
// change of mode with KEY (without display but with LED only)
// change of symbol with KEY (without display but with LED only)
//
// version V1.1
// added HW Version V1.0 support
// added presetting in the header TTGO...config.h to prevent long initial setup at first boot up
// added "SPACE" to allowed letters for callsign for shorter callsigns - has to be added at the end
// added smart beaconing
//
// version V1.0beta
// first released version//

// #define DEBUG             // used for debugging purposes , e.g. turning on special serial or display logging
// Includes

#include <TTGO_T-Beam_LoRa_APRS_config.h> // to config user parameters
#include <Arduino.h>
#include <Preferences.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <BG_RF95.h>         // library from OE1ACM

#include <TinyGPS++.h>
#include <math.h>
#ifdef DS18B20
   #include <OneWire.h>         // libraries for DS18B20
   #include <DallasTemperature.h>
#else
  #ifdef USE_BME280
    #include <Adafruit_BME280.h>  // BME280 Library
  #else
    #include <DHTesp.h>          // library from https://github.com/beegee-tokyo/DHTesp for DHT22
  #endif
#endif
#include <driver/adc.h>
#include <Wire.h>

#include <Adafruit_I2CDevice.h>
#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>
#include <axp20x.h>

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//PINs used for HW extensions

// Pin for battery voltage -> bei T-Beam ADC1_CHANNEL_7
// #define ANALOG_PIN_0 35      // connected to battery

// I2C LINES
#define I2C_SDA 21
#define I2C_SCL 22

// DISPLAY address
#define SSD1306_ADDRESS 0x3C

// AXP192 address
// #define AXP192_SLAVE_ADDRESS  0x34 // already defined in axp20x.h

/* for feather32u4
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
*/


// Variables for DHT22 temperature and humidity sensor
int chk;
boolean hum_temp = false;
uint8_t hum_temp_ctr, hum_temp_ctr_max = 3;
float hum=0;                 //Stores humidity value
float temp=99.99;            //Stores temperature value
float tempf=99.99;           //Stores temperature value

//other global Variables
String Textzeile1, Textzeile2;
int button=0;
int button_ctr=0;
// int version=0; // 0 = V0.7, 1 = V1.0
// bool ssd1306_found = false;
// bool axp192_found = false;

#define TRACKER 0
#define WX_TRACKER 1
#define WX_MOVE 2
#define WX_FIXED 3
// Position from GPS for TRACKER and WX_TRACKER
// Position for WX_ONLY from Headerfile!!!

uint8_t tracker_mode;

// Pins for GPS
#ifdef T_BEAM_V1_0
   static const int RXPin = 12, TXPin = 34;  //  changed BG A3 A2
#else
   static const int RXPin = 15, TXPin = 12;  //  changed BG A3 A2
#endif

static const uint32_t GPSBaud = 9600; //GPS

const byte TX_en  = 0;
const byte RX_en  = 0;       //TX/RX enable 1W modul

// LED for signalling
#ifdef T_BEAM_V1_0
   const byte TXLED  = 4;      //pin number for LED on TX Tracker
#else
   const byte TXLED  = 14;      //pin number for LED on TX Tracker
 #endif

// Button of TTGO T-Beam
#ifdef T_BEAM_V1_0
//   const byte BUTTON  = 38;      //pin number for Button on TTGO T-Beam
   #define BUTTON  38      //pin number for Button on TTGO T-Beam
#else
   #define BUTTON  39      //pin number for Button on TTGO T-Beam
#endif

// const byte GPSLED = 6;      // pin gps & Heartbeat
// const byte GPSLED1 = 9;     // pin gps & Heartbeat

// Pins for LoRa module
//#ifdef T_BEAM_V1_0
//   const byte lora_PReset = 14; //pin where LoRa device reset line is connected
//   const byte lora_PNSS = 18;   //pin number where the NSS line for the LoRa device is connected.
//#else
   const byte lora_PReset = 23; //pin where LoRa device reset line is connected
   const byte lora_PNSS = 18;   //pin number where the NSS line for the LoRa device is connected.
//#endif
                             // pin 11  MOSI
                             // pin 12  MISO
                             // pin 13  SCLK

// #define ModemConfig BG_RF95::Bw125Cr45Sf4096

#define DHTPIN 25            // the DHT22 is connected to PIN25
#define ONE_WIRE_BUS 25      // the DS18B20 is connected to PIN25


// Variables for APRS packaging
String Tcall;                //your Call Sign for normal position reports
String wxTcall;              //your Call Sign for weather reports
String sTable="/";           //Primer
String wxTable="/";          //Primer
String wxSymbol="_";         //Symbol Code Weather Station

// Tracker setting: use these lines to modify the tracker behaviour
#define TXFREQ  433.775      // Transmit frequency in MHz
#define TXdbmW  18           // Transmit power in dBm
#define TXenablePA  0        // switch internal power amplifier on (1) or off (0)

// Variables and Constants
Preferences prefs;

String InputString = "";     //data on buff is copied to this string
String Outputstring = "";
String outString="";         //The new Output String with GPS Conversion RAW

String LongShown="";
String LatShown="";

String LongFixed="";
String LatFixed="";

String TxSymbol="";

boolean wx;

//byte arrays
byte  lora_TXBUFF[128];      //buffer for packet to send
byte  lora_RXBUFF[128];      //buffer for packet to send
//byte Variables
byte  lora_TXStart;          //start of packet data in TXbuff
byte  lora_TXEnd;            //end of packet data in TXbuff
byte  lora_FTXOK;            //flag, set to 1 if TX OK
byte  lora_TXPacketType;     //type number of packet to send
byte  lora_TXDestination;    //destination address of packet to send
byte  lora_TXSource;         //source address of packet received
byte  lora_FDeviceError;     //flag, set to 1 if RFM98 device error
byte  lora_TXPacketL;        //length of packet to send, includes source, destination and packet type.


unsigned long lastTX = 0L;

float BattVolts;

// variables for smart beaconing
float average_speed[5] = {0,0,0,0,0}, average_speed_final=0, max_speed=30, min_speed=0;
float old_course = 0, new_course = 0;
int point_avg_speed = 0, point_avg_course = 0;
ulong min_time_to_nextTX=60000L;      // minimum time period between TX = 60000ms = 60secs = 1min
ulong nextTX=60000L;          // preset time period between TX = 60000ms = 60secs = 1min
#define ANGLE 60              // angle to send packet at smart beaconing
#define ANGLE_AVGS 3          // angle averaging - x times
float average_course[ANGLE_AVGS];
float avg_c_y, avg_c_x;

#ifdef DEBUG
  // debug Variables
  String TxRoot="0";
  float millis_angle[ANGLE_AVGS];
#endif

#define TX_BASE91         // if BASE91 is set, packets will be sent compressed (in TRACKER-mode only)

static const adc_atten_t atten = ADC_ATTEN_DB_6;
static const adc_unit_t unit = ADC_UNIT_1;

static void smartDelay(unsigned long);
void recalcGPS(void);
void sendpacket(void);
void loraSend(byte, byte, byte, byte, byte, long, byte, float);
void batt_read(void);
void writedisplaytext(String, String, String, String, String, String, int);
void setup_data(void);


#ifdef DS18B20
   OneWire oneWire(ONE_WIRE_BUS);
   DallasTemperature sensors(&oneWire);
#else
  #ifdef USE_BME280
    Adafruit_BME280 bme;         // if BME is used
  #else
    DHTesp dht;   // Initialize DHT sensor for normal 16mhz Arduino
  #endif
#endif
boolean tempsensoravailable=true;

// SoftwareSerial ss(RXPin, TXPin);   // The serial connection to the GPS device
HardwareSerial ss(1);        // TTGO has HW serial
TinyGPSPlus gps;             // The TinyGPS++ object
#ifdef T_BEAM_V1_0
  AXP20X_Class axp;
#endif

// checkRX
uint8_t buf[BG_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);

// Singleton instance of the radio driver

BG_RF95 rf95(18, 26);        // TTGO T-Beam has NSS @ Pin 18 and Interrupt IO @ Pin26

// initialize OLED display
#define OLED_RESET 4         // not used
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

// +---------------------------------------------------------------------+//
// + SETUP --------------------------------------------------------------+//
// +---------------------------------------------------------------------+//

void setup()
{
  bool bme_status;

  for (int i=0;i<ANGLE_AVGS;i++) {average_course[i]=0;} // set average_course to "0"

  #ifndef DONT_USE_FLASH_MEMORY
     prefs.begin("nvs", false);
     tracker_mode = (uint8_t) prefs.getChar("tracker_mode", TRACKERMODE);
     prefs.end();
  #else
    tracker_mode = TRACKERMODE;
  #endif

  //tracker_mode = current_mode;
  /////////////////
  // hier muss aus dem RAM der Modus gelesen werden!
  /////////////////
  if (tracker_mode == WX_TRACKER) {
    wx = true;
  } else  {
    wx = false;
  }

  pinMode(TXLED, OUTPUT);
  pinMode(BUTTON, INPUT);

  digitalWrite(TXLED, LOW);  // turn blue LED off
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);

  #ifdef T_BEAM_V1_0      // initialize power controller - new on HW V1.0
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
      Serial.println("LoRa-APRS / Init / AXP192 Begin PASS");
    } else {
      Serial.println("LoRa-APRS / Init / AXP192 Begin FAIL");
    }
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
    if (tracker_mode != WX_FIXED) {
      axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);   // switch on GPS in all modes except WX_FIXED
    } else {
      axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);  // switch off GPS in WX_FIXED mode
    }
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    axp.setDCDC1Voltage(3300);
  #endif

  if(!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_ADDRESS)) {
     for(;;); // Don't proceed, loop forever
  }
  writedisplaytext("LoRa-APRS","","Init:","Display OK!","","PRESS 3sec for config",1000);
  Serial.println("LoRa-APRS / Init / Display OK! / PRESS 3sec for config");

  #ifndef DONT_USE_FLASH_MEMORY
  //////////////////////////// Setup CALLSIGN
  prefs.begin("nvs", false);
  Tcall = prefs.getString("Tcall", CALLSIGN);
  wxTcall = prefs.getString("wxTcall", WX_CALLSIGN);
  LongFixed = prefs.getString("LongFixed", LONGITUDE_PRESET);
  LatFixed = prefs.getString("LatFixed", LATIDUDE_PRESET);
  TxSymbol = prefs.getString("TxSymbol", APRS_SYMBOL);
  prefs.end();
  #else
  Tcall = CALLSIGN;
  wxTcall = WX_CALLSIGN;
  LongFixed = LONGITUDE_PRESET;
  LatFixed = LATIDUDE_PRESET;
  TxSymbol = APRS_SYMBOL;
  #endif

  Serial.println("LoRa-APRS / Call="+Tcall+" / WX-Call="+wxTcall+" / TxSymbol="+TxSymbol);

  int start_button_pressed = millis();

  #ifndef DONT_USE_FLASH_MEMORY
  while ((digitalRead(BUTTON) == LOW) && (millis()<start_button_pressed+3000)) {

  }

  if ((digitalRead(BUTTON) == LOW) || (Tcall == "XX9XXX-0")) {  // into setup when no real data entered in TTGO...config.h
    writedisplaytext("LoRa-APRS","","","Entering Setup!","","",2000);
    setup_data();
  }
  #endif


  switch(tracker_mode) {
    case TRACKER:
      writedisplaytext("LoRa-APRS","","Init:","Mode","TRACKER","",1000);
      Serial.println("LoRa-APRS / Init / Mode / TRACKER");
      blinker(1);
      break;
    case WX_TRACKER:
      writedisplaytext("LoRa-APRS","","Init:","Mode","WX&TRACKER","",1000);
      Serial.println("LoRa-APRS / Init / Mode / WX & TRACKER");
      blinker(2);
      break;
    case WX_MOVE:
      writedisplaytext("LoRa-APRS","","Init:","Mode","WX_MOVE","",1000);
      Serial.println("LoRa-APRS / Init / Mode / WX only - moving");
      blinker(3);
      break;
    case WX_FIXED:
      writedisplaytext("LoRa-APRS","","Init:","Mode","WX_FIXED","(no GPS used)",1000);
      Serial.println("LoRa-APRS / Init / Mode / WX only - fixed Pos (no GPS used)");
      blinker(4);
      break;
    default:
      writedisplaytext("LoRa-APRS","","Init:","Mode","UNKNOWN","STOPPED",1000);
      Serial.println("LoRa-APRS / Init / Mode / UNKNOWN STOPPED!!!!");
      while (true) {
        blinker(1);
      }
      break;
  }

    if (!rf95.init()) {

    writedisplaytext("LoRa-APRS","","Init:","RF95 FAILED!",":-(","",250);
    Serial.println("LoRa-APRS / Init / RF95 FAILED!");
    for(;;); // Don't proceed, loop forever
  }

  if (max_time_to_nextTX < nextTX) {max_time_to_nextTX=nextTX;}

  // digitalWrite(TXLED, HIGH);
  writedisplaytext("LoRa-APRS","","Init:","RF95 OK!","","",250);
  // digitalWrite(TXLED, LOW);
  Serial.println("LoRa-APRS / Init / RF95 OK!");

  if (tracker_mode != WX_FIXED) {
    ss.begin(GPSBaud, SERIAL_8N1, TXPin, RXPin);        //Startup HW serial for GPS
    writedisplaytext("LoRa-APRS","","Init:","GPS Serial OK!","","",250);
    Serial.println("LoRa-APRS / Init / GPS Serial OK!");
    writedisplaytext(" "+Tcall,"","Init:","Waiting for GPS","","",250);
    Serial.println("LoRa-APRS / Init / Waiting for GPS");
    while (millis() < 5000 && gps.charsProcessed() < 10) {}
    if (millis() > 5000 && gps.charsProcessed() < 10) {
      writedisplaytext(" "+Tcall,"","Init:","ERROR!","No GPS data!","Please restart TTGO",0);
      Serial.println("LoRa-APRS / Init / GPS ERROR - no GPS data - please RESTART TTGO");
      while (true) {blinker(1);}
    }
    writedisplaytext(" "+Tcall,"","Init:","Data from GPS OK!","","",250);
    Serial.println("LoRa-APRS / Init / Data from GPS OK!");
  } else {
    writedisplaytext(" "+Tcall,"","Init:","GPS switched OFF!","","",250);
    Serial.println("LoRa-APRS / Init / GPS switched OFF!");
  }

  #ifdef T_BEAM_V1_0
    writedisplaytext("LoRa-APRS","","Init:","ADC OK!","BAT: "+String(axp.getBattVoltage()/1000,1),"",250);
    Serial.print("LoRa-APRS / Init / ADC OK! / BAT: ");
    Serial.println(String(axp.getBattVoltage()/1000,1));
#else
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_6);
    writedisplaytext("LoRa-APRS","","Init:","ADC OK!","BAT: "+String(analogRead(35)*7.221/4096,1),"",250);
    Serial.print("LoRa-APRS / Init / ADC OK! / BAT: ");
    Serial.println(String(analogRead(35)*7.221/4096,1));
  #endif

  rf95.setFrequency(433.775);
  rf95.setModemConfig(BG_RF95::Bw125Cr45Sf4096); // hard coded because of double definition
  rf95.setTxPower(5);

  #ifdef DS18B20
    sensors.begin();
  #else
    #ifdef USE_BME280
      bme_status = bme.begin(0x76);
      if (!bme_status)
      {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        writedisplaytext("LoRa-APRS","","Init:","BME280 ERROR!","","",3000);
        tempsensoravailable = false;
      }
    #else
      dht.setup(DHTPIN,dht.AUTO_DETECT); // initialize DHT22
    #endif
  #endif
  delay(250);

  #ifdef DS18B20
    sensors.requestTemperatures(); // Send the command to get temperature readings
    temp = sensors.getTempCByIndex(0); // get temp from 1st (!) sensor only
  #else
    #ifdef USE_BME280
      bme.takeForcedMeasurement();
      temp = bme.readTemperature();  // bme Temperatur auslesen
      hum = bme.readHumidity();
    #else
      temp = dht.getTemperature();
      hum = dht.getHumidity();
    #endif
  #endif
  writedisplaytext("LoRa-APRS","","Init:","Temp OK!","TEMP: "+String(temp,1),"HUM: "+String(hum,1),250);
  Serial.print("LoRa-APRS / Init / Temp OK! Temp=");
  Serial.print(String(temp));
  Serial.print(" Hum=");
  Serial.println(String(hum));
  writedisplaytext("LoRa-APRS","","Init:","FINISHED OK!","   =:-)   ","",250);
  Serial.println("LoRa-APRS / Init / FINISHED OK! / =:-)");
  writedisplaytext("","","","","","",0);

  hum_temp_ctr = 0;
}

// +---------------------------------------------------------------------+//
// + MAINLOOP -----------------------------------------------------------+//
// +---------------------------------------------------------------------+//

void loop() {
  if (digitalRead(BUTTON)==LOW) {
    ++button_ctr;
    if (button_ctr>=5) {
      switch(tracker_mode) {
        case TRACKER:
          tracker_mode = WX_TRACKER;
          writedisplaytext("LoRa-APRS","","New Mode","WX-TRACKER","","",500);
          Serial.println("LoRa-APRS / New Mode / WX-TRACKER");
          blinker(2);
          break;
        case WX_TRACKER:
          tracker_mode = WX_MOVE;
          writedisplaytext("LoRa-APRS","","New Mode","WX-MOVING","","",500);
          Serial.println("LoRa-APRS / New Mode / WX-MOVING");
          blinker(3);
          break;
        case WX_MOVE:
          tracker_mode = WX_FIXED;
          writedisplaytext("LoRa-APRS","","New Mode","WX-FIXED","","",500);
          Serial.println("LoRa-APRS / New Mode / WX-FIXED");
          #ifdef T_BEAM_V1_0
            axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);   // switch OFF GPS at mode WX_FIXED
          #endif
          blinker(4);
          break;
        case WX_FIXED:
        default:
          tracker_mode = TRACKER;
          writedisplaytext("LoRa-APRS","","New Mode","TRACKER","","",500);
          Serial.println("LoRa-APRS / New Mode / TRACKER");
          #ifdef T_BEAM_V1_0
            axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);   // switch on GPS in all modes except WX_FIXED
          #endif
          blinker(1);
          break;
      }
      prefs.begin("nvs", false);
      prefs.putChar("tracker_mode", (char) tracker_mode);
      prefs.end();
      button_ctr=0;
      // ESP.restart();
    }
  } else {
    button_ctr = 0;
  }

    if (hum_temp) {
      ++hum_temp_ctr;
      if (hum_temp_ctr>hum_temp_ctr_max) {
        hum_temp_ctr = 0;
        hum_temp=false;
      }
    #ifdef DS18B20
      sensors.requestTemperatures(); // Send the command to get temperature readings
      temp = sensors.getTempCByIndex(0); // get temp from 1st (!) sensor only
    #else
      #ifdef USE_BME280
        bme.takeForcedMeasurement();
        temp = bme.readTemperature();  // bme Temperatur auslesen
      #else
        temp = dht.getTemperature();
      #endif
    #endif
  } else {
    ++hum_temp_ctr;
    if (hum_temp_ctr>hum_temp_ctr_max) {
      hum_temp_ctr = 0;
      hum_temp=true;
    }
    #ifdef DS18B20
      hum = 0;
    #else
      #ifdef USE_BME280
        bme.takeForcedMeasurement();
        hum = bme.readHumidity();
      #else
        hum = dht.getHumidity();
      #endif
    #endif
  }

  if (tracker_mode != WX_FIXED) {
    while (ss.available() > 0) {
      gps.encode(ss.read());
    }
  }

  if (rf95.waitAvailableTimeout(100)) {
  #ifdef SHOW_RX_PACKET                                       // only show RX packets when activitated in config
    if (rf95.recvAPRS(lora_RXBUFF, &len)) {
      Serial.print("((RX)): ");
      InputString = "";
      for ( int i=0 ; i < len ; i++) {
        InputString += (char) lora_RXBUFF[i];
      }
      Serial.println(InputString);
      blinker(3);
      writedisplaytext("  ((RX))","",InputString,"","","",SHOW_RX_TIME);
    }
  #endif
  }

  if (tracker_mode != WX_FIXED) {
    LatShown = String(gps.location.lat(),5);
    LongShown = String(gps.location.lng(),5);

    average_speed[point_avg_speed] = gps.speed.kmph();   // calculate smart beaconing
    ++point_avg_speed;
    if (point_avg_speed>4) {point_avg_speed=0;}
    average_speed_final = (average_speed[0]+average_speed[1]+average_speed[2]+average_speed[3]+average_speed[4])/5;
    nextTX = (max_time_to_nextTX-min_time_to_nextTX)/(max_speed-min_speed)*(max_speed-average_speed_final)+min_time_to_nextTX;
    #ifdef DEBUG
      TxRoot="S";
    #endif

    if (nextTX < min_time_to_nextTX) {nextTX=min_time_to_nextTX;}
    if (nextTX > max_time_to_nextTX) {nextTX=max_time_to_nextTX;}

    average_course[point_avg_course] = gps.course.deg();   // calculate smart beaconing course
    #ifdef DEBUG
      millis_angle[point_avg_course]=millis();
    #endif
    ++point_avg_course;
    if (point_avg_course>(ANGLE_AVGS-1)) {
      point_avg_course=0;
      avg_c_y = 0;
      avg_c_x = 0;
      for (int i=0;i<ANGLE_AVGS;i++) {
        avg_c_y += sin(average_course[i]/180*3.1415);
        avg_c_x += cos(average_course[i]/180*3.1415);
      }
      new_course = atan2f(avg_c_y,avg_c_x)*180/3.1415;
      if (new_course < 0) {new_course=360+new_course;}
      if ((old_course < ANGLE) && (new_course > (360-ANGLE))) {
        if (abs(new_course-old_course-360)>=ANGLE) {
          nextTX = 0;
          // lastTX = min_time_to_nextTX
          #ifdef DEBUG
            TxRoot="W1";
            for (int i=0;i<2;i++)
            {
//              TxRoot += " c:" + String(average_course[i]) + " t:" + String(millis_angle[i]);
              TxRoot += " " + String(millis_angle[i],2);
            }
            TxRoot = TxRoot + " new:" + String(new_course) + " old:" +String(old_course);
          #endif
        }
      } else {
        if ((old_course > (360-ANGLE)) && (new_course < ANGLE)) {
          if (abs(new_course-old_course+360)>=ANGLE) {
            nextTX = 0;
            #ifdef DEBUG
              TxRoot="W2";
              for (int i=0;i<2;i++)
              {
                //              TxRoot += " c:" + String(average_course[i]) + " t:" + String(millis_angle[i]);
                TxRoot += " " + String(millis_angle[i],2);
              }
              TxRoot = TxRoot + " new:" + String(new_course) + " old:" +String(old_course);
            #endif
          }
        } else {
          if (abs(new_course-old_course)>=ANGLE) {
            nextTX = 0;
            #ifdef DEBUG
              TxRoot="W3";
              for (int i=0;i<2;i++)
              {
                //              TxRoot += " c:" + String(average_course[i]) + " t:" + String(millis_angle[i]);
                TxRoot += " " + String(millis_angle[i],2);
              }
              TxRoot = TxRoot + " new:" + String(new_course) + " old:" +String(old_course);
            #endif
          }
        }
      }
      old_course = new_course;
    }
  } else {
    LatShown = LatFixed;
    LongShown = LongFixed;
    nextTX = max_time_to_nextTX;
  }

  batt_read();

  if (button_ctr==2) {
    nextTX = 0;
    #ifdef DEBUG
      TxRoot="B";
    #endif
  }

  if ((millis()<max_time_to_nextTX)&&(lastTX == 0)) {
    nextTX = 0;
    #ifdef DEBUG
      TxRoot="1";
    #endif
  }

  if ( (lastTX+nextTX) <= millis()  ) {
    if (tracker_mode != WX_FIXED) {
      // if (gps.location.isValid()) {
      if (gps.location.age() < 2000) {
        digitalWrite(TXLED, HIGH);
        if (hum_temp) {
          writedisplaytext(" ((TX))","","LAT: "+LatShown,"LON: "+LongShown,"SPD: "+String(gps.speed.kmph(),1)+"  CRS: "+String(gps.course.deg(),1),"BAT: "+String(BattVolts,1)+"  HUM: "+String(hum,1),0);
        } else {
          writedisplaytext(" ((TX))","","LAT: "+LatShown,"LON: "+LongShown,"SPD: "+String(gps.speed.kmph(),1)+"  CRS: "+String(gps.course.deg(),1),"SAT: "+String(gps.satellites.value())+"   TEMP: "+String(temp,1),0);
        }
        sendpacket();
        Serial.print("((TX)) / LAT: ");
        Serial.print(LatShown);
        Serial.print(" / LON: ");
        Serial.print(LongShown);
        Serial.print(" / SPD: ");
        Serial.print(String(gps.speed.kmph(),1));
        Serial.print(" / CRS: ");
        Serial.print(String(gps.course.deg(),1));
        Serial.print(" / SAT: ");
        Serial.print(String(gps.satellites.value()));
        Serial.print(" / BAT: ");
        Serial.print(String(BattVolts,1));
        Serial.print(" / TEMP: ");
        Serial.print(String(temp,1));
        Serial.print(" / HUM: ");
        Serial.println(String(hum,1));
        digitalWrite(TXLED, LOW);
      } else {
        if (hum_temp) {
          writedisplaytext(" "+Tcall,"(TX) at valid GPS","LAT: not valid","LON: not valid","SPD: ---  CRS: ---","BAT: "+String(BattVolts,1)+"  HUM: "+String(hum,1),0);
        } else {
          writedisplaytext(" "+Tcall,"(TX) at valid GPS","LAT: not valid","LON: not valid","SPD: ---  CRS: ---","SAT: "+String(gps.satellites.value())+"   TEMP: "+String(temp,1),0);
        }
        Serial.print("(TX) at valid GPS / LAT: not valid / Lon: not valid / SPD: --- / CRS: ---");
        Serial.print(" / SAT: ");
        Serial.print(String(gps.satellites.value()));
        Serial.print(" / BAT: ");
        Serial.print(String(BattVolts,1));
        Serial.print(" / TEMP: ");
        Serial.print(String(temp,1));
        Serial.print(" / HUM: ");
        Serial.println(String(hum,1));
      }
    } else {                           // ab hier Code für WX_FIXED
      digitalWrite(TXLED, HIGH);
      if (hum_temp) {
        writedisplaytext(" ((TX))","","LAT: "+LatShown,"LON: "+LongShown,"No GPS used","BAT: "+String(BattVolts,1)+"  HUM: "+String(hum,1),0);
      } else {
        writedisplaytext(" ((TX))","","LAT: "+LatShown,"LON: "+LongShown,"No GPS used","SAT: --- TEMP: "+String(temp,1),0);
      }
      Serial.print("((TX)) / LAT: ");
      Serial.print(LatShown);
      Serial.print(" / LON: ");
      Serial.print(LongShown);
      Serial.print(" / No GPS used / SAT: --- / BAT: ");
      Serial.print(String(BattVolts,1));
      Serial.print(" / TEMP: ");
      Serial.print(String(temp,1));
      Serial.print(" / HUM: ");
      Serial.println(String(hum,1));
      sendpacket();
      Serial.println("State: Packet sent!");
      digitalWrite(TXLED, LOW);
    }
  } else {
    if (tracker_mode != WX_FIXED) {
      if (gps.location.age() < 2000) {
        if (hum_temp) {
          writedisplaytext(" "+Tcall,"Time to TX: "+String(((lastTX+nextTX)-millis())/1000)+"sec","LAT: "+LatShown,"LON: "+LongShown,"SPD: "+String(gps.speed.kmph(),1)+"  CRS: "+String(gps.course.deg(),1),"BAT: "+String(BattVolts,1)+"  HUM: "+String(hum,1),0);
        } else {
          writedisplaytext(" "+Tcall,"Time to TX: "+String(((lastTX+nextTX)-millis())/1000)+"sec","LAT: "+LatShown,"LON: "+LongShown,"SPD: "+String(gps.speed.kmph(),1)+"  CRS: "+String(gps.course.deg(),1),"SAT: "+String(gps.satellites.value())+"   TEMP: "+String(temp,1),0);
        }
      } else {
        if (hum_temp) {
          writedisplaytext(" "+Tcall,"Time to TX: "+String(((lastTX+nextTX)-millis())/1000)+"sec","LAT: not valid","LON: not valid","SPD: ---  CRS: ---","BAT: "+String(BattVolts,1)+"  HUM: "+String(hum,1),0);
        } else {
          writedisplaytext(" "+Tcall,"Time to TX: "+String(((lastTX+nextTX)-millis())/1000)+"sec","LAT: not valid","LON: not valid","SPD: ---  CRS: ---","SAT: "+String(gps.satellites.value())+"   TEMP: "+String(temp,1),0);
        }
      }
    } else {                              // ab hier WX_FIXED code
      if (hum_temp) {
        writedisplaytext(" "+wxTcall,"Time to TX: "+String(((lastTX+nextTX)-millis())/1000)+"sec","LAT: "+LatShown,"LON: "+LongShown,"SPD: ---  CRS: ---","BAT: "+String(BattVolts,1)+"  HUM: "+String(hum,1),0);
      } else {
        writedisplaytext(" "+wxTcall,"Time to TX: "+String(((lastTX+nextTX)-millis())/1000)+"sec","LAT: "+LatShown,"LON: "+LongShown,"SPD: ---  CRS: ---","SAT: ---  TEMP: "+String(temp,1),0);
      }
    }
  }
  smartDelay(900);
}

/////////////////////////////////////////////////////////////////////////////////////////
// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    if (tracker_mode != WX_FIXED) {
      while (ss.available())
        gps.encode(ss.read());
    }
  } while (millis() - start < ms);
}

char *ax25_base91enc(char *s, uint8_t n, uint32_t v)
{
  /* Creates a Base-91 representation of the value in v in the string */
  /* pointed to by s, n-characters long. String length should be n+1. */

  for(s += n, *s = '\0'; n; n--)
  {
    *(--s) = v % 91 + 33;
    v /= 91;
  }

  return(s);
}

/////////////////////////////////////////////////////////////////////////////////////////
//@APA Recalc GPS Position == generate APRS string
void recalcGPS(){

  String Ns, Ew, helper;
  char helper_base91[] = {"0000\0"};
  float Tlat=48.2012, Tlon=15.6361;
  int i, Talt, lenalt;
  uint32_t aprs_lat, aprs_lon;
  float Lat=0.0;
  float Lon=0.0;
  float Tspeed=0, Tcourse=0;
  String Speedx, Coursex, Altx;

  if (tracker_mode != WX_FIXED) {
    Tlat=gps.location.lat();
    Tlon=gps.location.lng();
    Talt=gps.altitude.meters() * 3.28;
    Altx = Talt;
    lenalt = Altx.length();
    Altx = "";
    for (i = 0; i < (6-lenalt); i++) {
      Altx += "0";
    }
    Altx += Talt;
    Tcourse=gps.course.deg();
    Tspeed=gps.speed.knots();

    aprs_lat = 900000000 - Tlat * 10000000;
    aprs_lat = aprs_lat / 26 - aprs_lat / 2710 + aprs_lat / 15384615;
    aprs_lon = 900000000 + Tlon * 10000000 / 2;
    aprs_lon = aprs_lon / 26 - aprs_lon / 2710 + aprs_lon / 15384615;

    if(Tlat<0) { Ns = "S"; } else { Ns = "N"; }
    if(Tlat < 0) { Tlat= -Tlat; }
    unsigned int Deg_Lat = Tlat;
    Lat = 100*(Deg_Lat) + (Tlat - Deg_Lat)*60;

    if(Tlon<0) { Ew = "W"; } else { Ew = "E"; }
    if(Tlon < 0) { Tlon= -Tlon; }
    unsigned int Deg_Lon = Tlon;
    Lon = 100*(Deg_Lon) + (Tlon - Deg_Lon)*60;
  }

outString = "";

switch(tracker_mode) {
  case WX_FIXED:
    #ifdef DS18B20
      sensors.requestTemperatures(); // Send the command to get temperature readings
      tempf = sensors.getTempFByIndex(0); // get temp from 1st (!) sensor only
      hum = 0;
    #else
      #ifdef USE_BME280
        bme.takeForcedMeasurement();
        tempf = bme.readTemperature()*9/5+32;
        hum = bme.readHumidity();
      #else
        hum = dht.getHumidity();
        tempf = dht.getTemperature()*9/5+32;
      #endif
    #endif
    for (i=0; i<wxTcall.length();++i){  // remove unneeded "spaces" from callsign field
      if (wxTcall.charAt(i) != ' ') {
        outString += wxTcall.charAt(i);
      }
    }
    // outString = wxTcall;
    outString += ">APRS:!";
    outString += LatFixed;
    outString += wxTable;
    outString += LongFixed;
    outString += wxSymbol;
    outString += ".../...g...t";
    if (tempf < 0) {     // negative Werte erstellen
      outString += "-";
      if(tempf>-10) {outString += "0"; }
      tempf = abs(tempf);
    } else {                // positive Werte erstellen
      if(tempf<100) {outString += "0"; }
      if(tempf<10) {outString += "0"; }
    }
    helper = String(tempf,0);
    helper.trim();
    outString += helper;
    outString += "r...p...P...h";
    if(hum<10) {outString += "0"; }
    helper = String(hum,0);
    helper.trim();
    outString += helper;
    outString += "b.....";
    outString += MY_COMMENT;
    break;
  case WX_TRACKER:
    if (wx) {
      #ifdef DS18B20
        sensors.requestTemperatures(); // Send the command to get temperature readings
        tempf = sensors.getTempFByIndex(0); // get temp from 1st (!) sensor only
        hum = 0;
      #else
        #ifdef USE_BME280
          bme.takeForcedMeasurement();
          tempf = bme.readTemperature()*9/5+32;  // bme Temperatur auslesen
          hum = bme.readHumidity();
        #else
          hum = dht.getHumidity();
          tempf = dht.getTemperature()*9/5+32;
        #endif
      #endif
      #ifndef TX_BASE91
        for (i=0; i<wxTcall.length();++i){  // remove unneeded "spaces" from callsign field
          if (wxTcall.charAt(i) != ' ') {
            outString += wxTcall.charAt(i);
          }
        }
        // outString = (wxTcall);
        outString += ">APRS:!";
        if(Tlat<10) {outString += "0"; }
        outString += String(Lat,2);
        outString += Ns;
        outString += wxTable;
        if(Tlon<100) {outString += "0"; }
        if(Tlon<10) {outString += "0"; }
        outString += String(Lon,2);
        outString += Ew;
        outString += wxSymbol;
      #else
        for (i=0; i<wxTcall.length();++i){  // remove unneeded "spaces" from callsign field
          if (wxTcall.charAt(i) != ' ') {
            outString += wxTcall.charAt(i);
          }
        }
        // outString = (Tcall);
        outString += ">APRS:!/";
        ax25_base91enc(helper_base91, 4, aprs_lat);
        for (i=0; i<4; i++) {
          outString += helper_base91[i];
        }
        ax25_base91enc(helper_base91, 4, aprs_lon);
        for (i=0; i<4; i++) {
          outString += helper_base91[i];
        }
        outString += wxSymbol;
        ax25_base91enc(helper_base91, 1, (uint32_t) Tcourse/4 );
        outString += helper_base91[0];
        ax25_base91enc(helper_base91, 1, (uint32_t) (log1p(Tspeed)/0.07696));
        outString += helper_base91[0];
        outString += "\x48";
      #endif
      outString += ".../...g...t";
      if (tempf < 0) {     // negative Werte erstellen
        outString += "-";
        if(tempf>-10) {outString += "0"; }
        tempf = abs(tempf);
      } else {                // positive Werte erstellen
        if(tempf<100) {outString += "0"; }
        if(tempf<10) {outString += "0"; }
      }
      helper = String(tempf,0);
      helper.trim();
      outString += helper;
      outString += "r...p...P...h";
      if(hum<10) {outString += "0"; }
      helper = String(hum,0);
      helper.trim();
      outString += helper;
      outString += "b.....";
      outString += MY_COMMENT;
      wx = !wx;
    } else {
      #ifndef TX_BASE91
        for (i=0; i<Tcall.length();++i){  // remove unneeded "spaces" from callsign field
          if (Tcall.charAt(i) != ' ') {
            outString += Tcall.charAt(i);
          }
        }
        // outString = (Tcall);
        outString += ">APRS:!";
        if(Tlat<10) {outString += "0"; }
        outString += String(Lat,2);
        outString += Ns;
        outString += sTable;
        if(Tlon<100) {outString += "0"; }
        if(Tlon<10) {outString += "0"; }
        outString += String(Lon,2);
        outString += Ew;
        outString += TxSymbol;
      #else
        for (i=0; i<Tcall.length();++i){  // remove unneeded "spaces" from callsign field
          if (Tcall.charAt(i) != ' ') {
            outString += Tcall.charAt(i);
          }
        }
        // outString = (Tcall);
        outString += ">APRS:!/";
        ax25_base91enc(helper_base91, 4, aprs_lat);
        for (i=0; i<4; i++) {
          outString += helper_base91[i];
        }
        ax25_base91enc(helper_base91, 4, aprs_lon);
        for (i=0; i<4; i++) {
          outString += helper_base91[i];
        }
        outString += TxSymbol;
        ax25_base91enc(helper_base91, 1, (uint32_t) Tcourse/4 );
        outString += helper_base91[0];
        ax25_base91enc(helper_base91, 1, (uint32_t) (log1p(Tspeed)/0.07696));
        outString += helper_base91[0];
        outString += "\x48";
      #endif

      #ifdef HW_COMMENT
        outString += "/A=";
        outString += Altx;
        outString += " Batt=";
        outString += String(BattVolts,2);
        outString += ("V");
      #endif
      outString += MY_COMMENT;
      wx = !wx;
    }
  break;
case WX_MOVE:
    #ifdef DS18B20
      sensors.requestTemperatures(); // Send the command to get temperature readings
      tempf = sensors.getTempFByIndex(0); // get temp from 1st (!) sensor only
      hum = 0;
    #else
      #ifdef USE_BME280
        bme.takeForcedMeasurement();
        tempf = bme.readTemperature()*9/5+32;  // bme Temperatur auslesen
        hum = bme.readHumidity();
      #else
        hum = dht.getHumidity();
        tempf = dht.getTemperature()*9/5+32;
      #endif
    #endif


    #ifndef TX_BASE91
      for (i=0; i<wxTcall.length();++i){  // remove unneeded "spaces" from callsign field
        if (wxTcall.charAt(i) != ' ') {
          outString += wxTcall.charAt(i);
        }
      }
      // outString = (wxTcall);
      outString += ">APRS:!";
      if(Tlat<10) {outString += "0"; }
      outString += String(Lat,2);
      outString += Ns;
      outString += wxTable;
      if(Tlon<100) {outString += "0"; }
      if(Tlon<10) {outString += "0"; }
      outString += String(Lon,2);
      outString += Ew;
      outString += wxSymbol;
      #else
        for (i=0; i<wxTcall.length();++i){  // remove unneeded "spaces" from callsign field
          if (wxTcall.charAt(i) != ' ') {
            outString += wxTcall.charAt(i);
          }
        }
        // outString = (Tcall);
        outString += ">APRS:!/";
        ax25_base91enc(helper_base91, 4, aprs_lat);
        for (i=0; i<4; i++) {
          outString += helper_base91[i];
        }
        ax25_base91enc(helper_base91, 4, aprs_lon);
        for (i=0; i<4; i++) {
          outString += helper_base91[i];
        }
        outString += wxSymbol;
        ax25_base91enc(helper_base91, 1, (uint32_t) Tcourse/4 );
        outString += helper_base91[0];
        ax25_base91enc(helper_base91, 1, (uint32_t) (log1p(Tspeed)/0.07696));
        outString += helper_base91[0];
        outString += "\x48";
      #endif
    outString += ".../...g...t";
    if (tempf < 0) {     // negative Werte erstellen
      outString += "-";
      if(tempf>-10) {outString += "0"; }
      tempf = abs(tempf);
    } else {                // positive Werte erstellen
      if(tempf<100) {outString += "0"; }
      if(tempf<10) {outString += "0"; }
    }
    helper = String(tempf,0);
    helper.trim();
    outString += helper;
    outString += "r...p...P...h";
    if(hum<10) {outString += "0"; }
    helper = String(hum,0);
    helper.trim();
    outString += helper;
    outString += "b.....";
    outString += MY_COMMENT;
    break;
  case TRACKER:
  default:
    #ifndef TX_BASE91
      for (i=0; i<Tcall.length();++i){  // remove unneeded "spaces" from callsign field
        if (Tcall.charAt(i) != ' ') {
          outString += Tcall.charAt(i);
        }
      }
      // outString = (Tcall);
      outString += ">APRS:!";
      if(Tlat<10) {outString += "0"; }
      outString += String(Lat,2);
      outString += Ns;
      outString += sTable;
      if(Tlon<100) {outString += "0"; }
      if(Tlon<10) {outString += "0"; }
      outString += String(Lon,2);
      outString += Ew;
      outString += TxSymbol;
      if(Tcourse<100) {outString += "0"; }
      if(Tcourse<10) {outString += "0"; }
      Coursex = String(Tcourse,0);
      Coursex.replace(" ","");
      outString += Coursex;
      outString += "/";
      if(Tspeed<100) {outString += "0"; }
      if(Tspeed<10) {outString += "0"; }
      Speedx = String(Tspeed,0);
      Speedx.replace(" ","");
      outString += Speedx;
      #ifdef HW_COMMENT
        outString += "/A=";
        outString += Altx;
        outString += " Batt=";
        outString += String(BattVolts,2);
        outString += ("V");
      #endif
      outString += MY_COMMENT;
      #ifdef DEBUG
        outString += (" Debug: ");
        outString += TxRoot;
      #endif
    #else
      for (i=0; i<Tcall.length();++i){  // remove unneeded "spaces" from callsign field
        if (Tcall.charAt(i) != ' ') {
          outString += Tcall.charAt(i);
        }
      }
      // outString = (Tcall);
      outString += ">APRS:!/";
      ax25_base91enc(helper_base91, 4, aprs_lat);
      for (i=0; i<4; i++) {
        outString += helper_base91[i];
      }
      ax25_base91enc(helper_base91, 4, aprs_lon);
      for (i=0; i<4; i++) {
        outString += helper_base91[i];
      }
      outString += TxSymbol;
      ax25_base91enc(helper_base91, 1, (uint32_t) Tcourse/4 );
      outString += helper_base91[0];
      ax25_base91enc(helper_base91, 1, (uint32_t) (log1p(Tspeed)/0.07696));
      outString += helper_base91[0];
      outString += "\x48";
      #ifdef HW_COMMENT
        outString += "/A=";
        outString += Altx;
        outString += " Batt=";
        outString += String(BattVolts,2);
        outString += ("V");
      #endif
      outString += MY_COMMENT;
    #endif
    Serial.print("outString=");
    // Speedx = String(Tspeed,0);
    // Speedx.replace(" ","");
    Serial.println(outString);
    // Serial.println("=");
  break;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////
void sendpacket()
{

  batt_read();
  Outputstring = "";

switch(tracker_mode) {
  case WX_FIXED:
    recalcGPS();                        //
    Outputstring =outString;
    loraSend(lora_TXStart, lora_TXEnd, 60, 255, 1, 10, TXdbmW, TXFREQ);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
    break;
  case TRACKER:
  case WX_TRACKER:
  case WX_MOVE:
  default:
    if ( gps.location.isValid()   || gps.location.isUpdated() ) {
      recalcGPS();                        //
      Outputstring =outString;
      loraSend(lora_TXStart, lora_TXEnd, 60, 255, 1, 10, TXdbmW, TXFREQ);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
    }  else {
      Outputstring = (Tcall);
      Outputstring += " No GPS-Fix";
      Outputstring += " Batt=";
      Outputstring += String(BattVolts,2);
      Outputstring += ("V ");
      loraSend(lora_TXStart, lora_TXEnd, 60, 255, 1, 10, 5, TXFREQ);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
    }
    break;
  }
}

///////////////////////////////////////////////////////////////////////////////////////
void loraSend(byte lora_LTXStart, byte lora_LTXEnd, byte lora_LTXPacketType, byte lora_LTXDestination, byte lora_LTXSource, long lora_LTXTimeout, byte lora_LTXPower, float lora_FREQ)
{
  byte i;
  byte ltemp;

  if (rf95.waitAvailableTimeout(100)) {
    if (rf95.recvAPRS(buf, &len)) {
    }
  }

  // time of last TX
  lastTX = millis();

  ltemp = Outputstring.length();
  for (i = 0; i <= ltemp; i++)
  {
    lora_TXBUFF[i] = Outputstring.charAt(i);
  }

  i--;
  lora_TXEnd = i;
  lora_TXBUFF[i] ='\0';

  // digitalWrite(PLED1, HIGH);       //LED on during packet

  rf95.setModemConfig(BG_RF95::Bw125Cr45Sf4096);
  rf95.setFrequency(lora_FREQ);
  rf95.setTxPower(lora_LTXPower);
  rf95.sendAPRS(lora_TXBUFF, Outputstring.length());
  rf95.waitPacketSent();
}
///////////////////////////////////////////////////////////////////////////////////////
void batt_read()
{
  float BattRead = analogRead(35)*7.221;
#ifdef T_BEAM_V1_0
  BattVolts = axp.getBattVoltage()/1000;
#else
  BattVolts = (BattRead / 4096);
#endif
}

///////////////////////////////////////////////////////////////////////////////////////
void writedisplaytext(String HeaderTxt, String Line1, String Line2, String Line3, String Line4, String Line5, int warten) {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println(HeaderTxt);
  display.setTextSize(1);
  display.setCursor(0,16);
  display.println(Line1);
  display.setCursor(0,26);
  display.println(Line2);
  display.setCursor(0,36);
  display.println(Line3);
  display.setCursor(0,46);
  display.println(Line4);
  display.setCursor(0,56);
  display.println(Line5);
  display.display();
  smartDelay(warten);
}

///////////////////////////////////////////////////////////////////////////////////////
void setup_data(void) {
  char werte_call[37] = {' ','A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','1','2','3','4','5','6','7','8','9','0'};
  String werte_SSID[16] = {"0","1","2","3","4","5","6","7","8","9","10","11","12","13","14","15"};
  char werte_latlon[14] = {'0','1','2','3','4','5','6','7','8','9','N','S','E','W'};
  String werte_TxSymbol_text[6] = {"WX Station","       Car","    Person","   Bicycle","Motorcycle","        RV"};
  String werte_TxSymbol_symbol[6] = {"_",">","[","b","<","R"};
  String werte_weiter_symbol[2] = {"yes","no"};
  int8_t pos_in_string;
  int8_t pos_ssid;
  bool key_pressed = false;
  int waiter;
  int initial_waiter = 2000;
  char aktueller_letter;
  int8_t pos_letter;
  String pfeile = "^";
  int8_t initial_ssid;


  // set Tx Symbol und die normale SSID - gleich zu Beginn, falls man nur das Symbol ändern möchte
  pos_ssid = 0;
  while (true) {
    TxSymbol = werte_TxSymbol_symbol[pos_ssid];
    blinker(pos_ssid+1);
    writedisplaytext("  SETUP", "    Symbol",werte_TxSymbol_text[pos_ssid], "", "PRESS KEY to select", "", 0);
    waiter = millis();
    while (millis()<(waiter+1000+initial_waiter)) {
      if (digitalRead(BUTTON)==LOW) {
        key_pressed = true;
      }
    }
    initial_waiter = 0;
    if (key_pressed==true) {
      key_pressed = false;
      writedisplaytext("  SETUP", "    Symbol",werte_TxSymbol_text[pos_ssid], "", "programmed", "", 2000);
      break;
    }
    ++pos_ssid;
    if (pos_ssid>=6) {pos_ssid=0;}
  }

  // set normal SSID
  initial_ssid = (int8_t) (Tcall.substring(7,9)).toInt();

  pos_ssid = initial_ssid;
  pfeile = "          ^";
  key_pressed = false;
  initial_waiter = 2000;
  while (true) {
    writedisplaytext("  SETUP", "  normal SSID","   "+Tcall, pfeile, "PRESS KEY to select", "", 0);
    waiter = millis();
    while (millis()<(waiter+1000+initial_waiter)) {
      if (digitalRead(BUTTON)==LOW) {
        key_pressed = true;
      }
    }
    initial_waiter = 0;
    if (key_pressed==true) {
      key_pressed = false;
      break;
    }
    ++pos_ssid;
    if (pos_ssid>=16) {pos_ssid=0;}
    Tcall = Tcall.substring(0,6)+"-"+werte_SSID[pos_ssid];
  }

  writedisplaytext("  SETUP", "     SSID","   "+Tcall,"   ", "programmed", "", 2000);

  // fragen, ob es weiter gehen soll
  pos_ssid = 0;
  key_pressed = false;
  initial_waiter = 2000;
  while (true) {
    // TxSymbol = werte_TxSymbol_symbol[pos_ssid];
    blinker(2-pos_ssid);
    writedisplaytext("  SETUP", "  stop it?","   "+werte_weiter_symbol[pos_ssid], "", "PRESS KEY to select", "", 0);
    waiter = millis();
    while (millis()<(waiter+1000+initial_waiter)) {
      if (digitalRead(BUTTON)==LOW) {
        key_pressed = true;
      }
    }
    initial_waiter = 0;
    if (key_pressed==true) {
      key_pressed = false;
      writedisplaytext("  SETUP", "  stop it?","   "+werte_weiter_symbol[pos_ssid], "", "selected", "", 2000);
      break;
    }
    ++pos_ssid;
    if (pos_ssid>=2) {pos_ssid=0;}
  }

  if (pos_ssid != 0) {
    // set callsign - one for both reports

    pos_in_string = 0;
    key_pressed = false;
    initial_waiter = 2000;
    while (pos_in_string < 6) {
      key_pressed = false;
      aktueller_letter = (char) Tcall.charAt(pos_in_string);// ist Buchstabe holen
      for (pos_letter=0;pos_letter<37;pos_letter++) {
        if (aktueller_letter == werte_call[pos_letter]) {
          break;
        }
      }
      while (true) {
        Tcall.setCharAt(pos_in_string, aktueller_letter);
        writedisplaytext("  SETUP", "     Call","   "+Tcall,"   "+pfeile, "PRESS KEY to select", "", 0);
        waiter = millis();
        while (millis()<(waiter+1000+initial_waiter)) {
          if (digitalRead(BUTTON)==LOW) {
            key_pressed = true;
          }
        }
        initial_waiter = 0;
        if (key_pressed==true) {
          key_pressed = false;
          break;
        }
        // nächster Buchstabe
        ++pos_letter;
        if (pos_letter>=37) {pos_letter=0;}
        aktueller_letter=werte_call[pos_letter];
      }
      initial_waiter = 2000;
      pfeile = " "+pfeile;
      ++pos_in_string;
    }

    writedisplaytext("  SETUP", "     Call","   "+Tcall,"   ", "programmed", "", 2000);


    // set WX SSID
    initial_ssid = (int8_t) (wxTcall.substring(7,9)).toInt();

    pos_ssid = initial_ssid;
    key_pressed = false;
    initial_waiter = 2000;
    while (true) {
      writedisplaytext("  SETUP", "    WX SSID","   "+wxTcall, pfeile, "PRESS KEY to select", "", 0);
      waiter = millis();
      while (millis()<(waiter+1000+initial_waiter)) {
        if (digitalRead(BUTTON)==LOW) {
          key_pressed = true;
        }
      }
      initial_waiter = 0;
      if (key_pressed==true) {
        key_pressed = false;
        break;
      }
      ++pos_ssid;
      if (pos_ssid>=16) {pos_ssid=0;}
      wxTcall = wxTcall.substring(0,6)+"-"+werte_SSID[pos_ssid];
    }

    writedisplaytext("  SETUP", "  WX-Call","   "+wxTcall,"   ", "programmed", "", 2000);

    // set LONGITUDE
    pfeile = "^";
    pos_in_string = 0;
    key_pressed = false;
    initial_waiter = 2000;
    while (pos_in_string < 9) {
      key_pressed = false;
      aktueller_letter = (char) LongFixed.charAt(pos_in_string);// ist Buchstabe holen
      for (pos_letter=0;pos_letter<14;pos_letter++) {
        if (aktueller_letter == werte_latlon[pos_letter]) {
          break;
        }
      }
      while (true) {
        LongFixed.setCharAt(pos_in_string, aktueller_letter);
        writedisplaytext("  SETUP", "    Longitude","  "+LongFixed,"  "+pfeile, "for fixed POS", "PRESS KEY to select", 0);
        waiter = millis();
        while (millis()<(waiter+1000+initial_waiter)) {
          if (digitalRead(BUTTON)==LOW) {
            key_pressed = true;
          }
        }
        initial_waiter = 0;
        if (key_pressed==true) {
          key_pressed = false;
          break;
        }
        // nächster Buchstabe
        ++pos_letter;
        if (pos_letter>=14) {pos_letter=0;}
        aktueller_letter=werte_latlon[pos_letter];
      }
      initial_waiter = 2000;
      pfeile = " "+pfeile;
      ++pos_in_string;
      if (pos_in_string == 5) {
        ++pos_in_string;
        pfeile = " "+pfeile;
      }
    }

    writedisplaytext("  SETUP", "    Longitude","  "+LongFixed,"", "for fixed POS", "programmed", 2000);

    // set LATITUDE
    pfeile = "^";
    pos_in_string = 0;
    key_pressed = false;
    initial_waiter = 2000;
    while (pos_in_string < 8) {
      key_pressed = false;
      aktueller_letter = (char) LatFixed.charAt(pos_in_string);// ist Buchstabe holen
      for (pos_letter=0;pos_letter<14;pos_letter++) {
        if (aktueller_letter == werte_latlon[pos_letter]) {
          break;
        }
      }
      while (true) {
        LatFixed.setCharAt(pos_in_string, aktueller_letter);
        writedisplaytext("  SETUP", "    Latitude","  "+LatFixed,"  "+pfeile, "for fixed POS", "PRESS KEY to select", 0);
        waiter = millis();
        while (millis()<(waiter+1000+initial_waiter)) {
          if (digitalRead(BUTTON)==LOW) {
            key_pressed = true;
          }
        }
        initial_waiter = 0;
        if (key_pressed==true) {
          key_pressed = false;
          break;
        }
        // nächster Buchstabe
        ++pos_letter;
        if (pos_letter>=14) {pos_letter=0;}
        aktueller_letter=werte_latlon[pos_letter];
      }
      initial_waiter = 2000;
      pfeile = " "+pfeile;
      ++pos_in_string;
      if (pos_in_string == 4) {
        ++pos_in_string;
        pfeile = " "+pfeile;
      }
    }
    writedisplaytext("  SETUP", "     Latitude","  "+LatFixed,"", "for fixed POS", "programmed", 2000);

}
  // write all values to NVRAM
  prefs.begin("nvs", false);
  prefs.putString("Tcall", Tcall);
  prefs.putString("wxTcall", wxTcall);
  prefs.putString("LatFixed", LatFixed);
  prefs.putString("LongFixed", LongFixed);
  prefs.putString("TxSymbol", TxSymbol);
  prefs.end();
  writedisplaytext("  SETUP", "ALL DONE","", "stored in NVS", "", "", 2000);
}

void blinker(int counter) {
  for (int i = 0; i < (counter-1); i++) {
    digitalWrite(TXLED, HIGH);  // turn blue LED ON
    smartDelay(150);
    digitalWrite(TXLED, LOW);  // turn blue LED OFF
    smartDelay(100);
  }
  digitalWrite(TXLED, HIGH);  // turn blue LED ON
  smartDelay(150);
  digitalWrite(TXLED, LOW);  // turn blue LED OFF
}
