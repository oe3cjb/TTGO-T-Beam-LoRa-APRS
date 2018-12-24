
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
// last update: 01.12.2018
// modifications: select mode during compilation to select model


#define DEBUG false           // used for debugging purposes , e.g. turning on special serial or display logging
// Includes

#include <TTGO_T-Beam_LoRa_APRS_config.h> // to config user parameters
#include <Arduino.h>
#include <Preferences.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <BG_RF95.h>         // library from OE1ACM

#include <TinyGPS++.h>
#include <math.h>
#include <DHTesp.h>          // library from https://github.com/beegee-tokyo/DHTesp
#include <driver/adc.h>
#include <Wire.h>

#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>

//Hardware definitions

/* for feather32u4
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
*/

//Variables for DHT22 temperature and humidity sensor
int chk;
boolean hum_temp = false;
float hum=0;                 //Stores humidity value
float temp=99.99;            //Stores temperature value
float tempf=99.99;           //Stores temperature value

//other global Variables
String Textzeile1, Textzeile2;
int button=0;
int button_ctr=0;

enum Tx_Mode {TRACKER, WX_TRACKER, WX_MOVE, WX_FIXED};
// Position from GPS for TRACKER and WX_TRACKER
// Position for WX_ONLY from Headerfile!!!

Tx_Mode tracker_mode;

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//PINs used for HW extensions

// Pin for battery voltage -> bei T-Beam ADC1_CHANNEL_7
// #define ANALOG_PIN_0 35      // connected to battery

// Pins for GPS
static const int RXPin = 15, TXPin = 12;  //  changed BG A3 A2
static const uint32_t GPSBaud = 9600; //GPS

const byte TX_en  = 0;
const byte RX_en  = 0;       //TX/RX enable 1W modul

const byte TXLED  = 14;      //pin number for LED on TX Tracker
const byte BUTTON  = 39;      //pin number for Button on TTGO T-Beam
// const byte GPSLED = 6;      // pin gps & Heartbeat
// const byte GPSLED1 = 9;     // pin gps & Heartbeat

// Pins for LoRa module
const byte lora_PReset = 23; //pin where LoRa device reset line is connected
const byte lora_PNSS = 18;   //pin number where the NSS line for the LoRa device is connected.
                             // pin 11  MOSI
                             // pin 12  MISO
                             // pin 13  SCLK

// #define ModemConfig BG_RF95::Bw125Cr45Sf4096

#define DHTPIN 25            // pin the DHT22 is connected to Pin25
//#define DHTTYPE DHT22        // DHT 22  (AM2302)

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

static const adc_atten_t atten = ADC_ATTEN_DB_6;
static const adc_unit_t unit = ADC_UNIT_1;

static void smartDelay(unsigned long);
void recalcGPS(void);
void sendpacket(void);
void loraSend(byte, byte, byte, byte, byte, long, byte, float);
void batt_read(void);
void writedisplaytext(String, String, String, String, String, String, int);
void setup_data(void);


DHTesp dht;

// SoftwareSerial ss(RXPin, TXPin);   // The serial connection to the GPS device
HardwareSerial ss(1);        // TTGO has HW serial
TinyGPSPlus gps;             // The TinyGPS++ object

// checkRX
uint8_t buf[BG_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);

// Singleton instance of the radio driver

BG_RF95 rf95(18, 26);        // TTGO T-Beam has NSS @ Pin 18 and Interrupt IO @ Pin26

// initialize OLED display
#define OLED_RESET 4         // not used
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

void setup()
{
  prefs.begin("nvs", false);
  tracker_mode = (Tx_Mode)prefs.getChar("tracker_mode", 0);
  prefs.end();
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
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
     for(;;); // Don't proceed, loop forever
  }
  digitalWrite(TXLED, HIGH);
  writedisplaytext("LoRa-APRS","","Init:","Display OK!","","Press 3sec for Config",250);
  digitalWrite(TXLED, LOW);
  Serial.println("Init: Display OK!");

  //////////////////////////// Setup CALLSIGN
  prefs.begin("nvs", false);
  Tcall = prefs.getString("Tcall", "OE1XYZ-0");
  wxTcall = prefs.getString("wxTcall", "OE1XYZ-0");
  LongFixed = prefs.getString("LongFixed", "01539.85E");
  LatFixed = prefs.getString("LatFixed", "4813.62N");
  TxSymbol = prefs.getString("TxSymbol", "[");
  prefs.end();

  int start_button_pressed = millis();

  while ((digitalRead(BUTTON) == LOW) && (millis()<start_button_pressed+3000)) {

  }
  //if (((start_button_pressed+3000<millis())&&(digitalRead(BUTTON) == LOW)) || (Tcall == "OE1000-0")) {
  if ((digitalRead(BUTTON) == LOW) || (Tcall == "OE1XYZ-0")) {
    setup_data();
  }

  switch(tracker_mode) {
    case TRACKER:
      writedisplaytext("LoRa-APRS","","Init:","Mode","TRACKER","",1000);
      Serial.println("Init: Mode TRACKER");
      break;
    case WX_MOVE:
      writedisplaytext("LoRa-APRS","","Init:","Mode","WX_MOVE","",1000);
      Serial.println("Init: Mode WX only - moving");
      break;
    case WX_FIXED:
      writedisplaytext("LoRa-APRS","","Init:","Mode","WX_FIXED","(no GPS used)",1000);
      Serial.println("Init: Mode WX only - fixed Pos");
      break;
    case WX_TRACKER:
      writedisplaytext("LoRa-APRS","","Init:","Mode","WX&TRACKER","",1000);
      Serial.println("Init: Mode WX & TRACKER");
      break;
    default:
      writedisplaytext("LoRa-APRS","","Init:","Mode","UNKNOWN","STOPPED",1000);
      Serial.println("Init: Mode UNKNOWN STOPPED!!!!");
      while (true) {}
      break;
  }

    if (!rf95.init()) {

    writedisplaytext("LoRa-APRS","","Init:","RF95 FAILED!",":-(","",250);
    Serial.println("Init: RF95 FAILED!");
    for(;;); // Don't proceed, loop forever
  }

  digitalWrite(TXLED, HIGH);
  writedisplaytext("LoRa-APRS","","Init:","RF95 OK!","","",250);
  digitalWrite(TXLED, LOW);
  Serial.println("Init: RF95 OK!");

  if (tracker_mode != WX_FIXED) {
    ss.begin(GPSBaud, SERIAL_8N1, 12, 15);        //Startup HW serial for GPS
  }
  digitalWrite(TXLED, HIGH);
  writedisplaytext("LoRa-APRS","","Init:","GPS Serial OK!","","",250);
  digitalWrite(TXLED, LOW);
  Serial.println("Init: GPS Serial OK!");

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_6);
  writedisplaytext("LoRa-APRS","","Init:","ADC OK!","BAT: "+String(analogRead(35)*7.221/4096,1),"",250);
  Serial.println("Init: ADC OK!");

  rf95.setFrequency(433.775);
  rf95.setModemConfig(BG_RF95::Bw125Cr45Sf4096); // hard coded because of double definition
  // rf95.setModemConfig(ModemConfig); // das ist irgendwo doppelt definiert ???
  rf95.setTxPower(5);
  //rf95.printRegisters();
  //rf95.setPromiscuousbg();

  dht.setup(DHTPIN,dht.AUTO_DETECT); // initialize DHT22
  delay(250);
  temp = dht.getTemperature();
  hum = dht.getHumidity();
  writedisplaytext("LoRa-APRS","","Init:","DHT OK!","TEMP: "+String(temp,1),"HUM: "+String(hum,1),250);
  Serial.print("Init: DHT OK! Temp=");
  Serial.print(String(temp));
  Serial.print(" Hum=");
  Serial.println(String(hum));

  digitalWrite(TXLED, HIGH);
  writedisplaytext("LoRa-APRS","","Init:","All DONE OK!",":-D","",500);
  digitalWrite(TXLED, LOW);
  Serial.println("Init: ALL DONE OK! :-D");
  writedisplaytext("","","","","","",0);
}


//   LOOP

void loop()
{
if (digitalRead(BUTTON)==LOW) {
  ++button_ctr;
  if (button_ctr>=3){
    switch(tracker_mode) {
      case TRACKER:
        tracker_mode = WX_TRACKER;
        writedisplaytext("LoRa-APRS","","New Mode","WX-TRACKER","","",500);
        break;
      case WX_TRACKER:
        tracker_mode = WX_MOVE;
        writedisplaytext("LoRa-APRS","","New Mode","WX-MOVING","","",500);
        break;
      case WX_MOVE:
        tracker_mode = WX_FIXED;
        writedisplaytext("LoRa-APRS","","New Mode","WX-FIXED","","",500);
        break;
      case WX_FIXED:
      default:
        tracker_mode = TRACKER;
        writedisplaytext("LoRa-APRS","","New Mode","TRACKER","","",500);
        break;
    }
    prefs.begin("nvs", false);
    prefs.putChar("tracker_mode", (char)tracker_mode);
    prefs.end();
    button_ctr=0;
  }
}
if (hum_temp){
  hum_temp=false;
  temp = dht.getTemperature();
} else {
  hum_temp=true;
  hum = dht.getHumidity();
}
// temp = dht.readTemperature();
Serial.print("Init: DHT OK! Temp=");
Serial.println(String(temp));

#if DEBUG
  writedisplaytext("LoRa-APRS","","DEBUG",millis(),String(millis()),"",0);
#endif
 //while(1) { if ( ss.available() ) Serial.write(ss.read());}
// if (tracker_mode != WX_FIXED) {
while (ss.available() > 0) {
    gps.encode(ss.read());
//  }
}

if (rf95.waitAvailableTimeout(100)) {
  if (rf95.recvAPRS(buf, &len)) {
  }
}
if (tracker_mode != WX_FIXED) {
  LatShown = String(gps.location.lat(),5);
  LongShown = String(gps.location.lng(),5);
} else {
  LatShown = LatFixed;
  LongShown = LongFixed;
}
if (hum_temp)
{
  writedisplaytext(" "+Tcall,"Time to TX: "+String(((lastTX+nextTX)-millis())/1000),"LAT: "+LatShown,"LON: "+LongShown,"SPD: "+String(gps.speed.kmph(),1)+"  CRS: "+String(gps.course.deg(),1),"BAT: "+String(analogRead(35)*7.221/4096,1)+"  HUM: "+String(hum,1),0);
} else {
  writedisplaytext(" "+Tcall,"Time to TX: "+String(((lastTX+nextTX)-millis())/1000),"LAT: "+LatShown,"LON: "+LongShown,"SPD: "+String(gps.speed.kmph(),1)+"  CRS: "+String(gps.course.deg(),1),"BAT: "+String(analogRead(35)*7.221/4096,1)+" TEMP: "+String(temp,1),0);
}
smartDelay(1000);

if ( (lastTX+nextTX) <= millis()  ) {
  if (tracker_mode != WX_FIXED) {
    if (gps.location.isValid()) {
      digitalWrite(TXLED, HIGH);
      batt_read();
      if (hum_temp) {
        writedisplaytext(" ((TX))","","LAT: "+LatShown,"LON: "+LongShown,"SPD: "+String(gps.speed.kmph(),1)+"  CRS: "+String(gps.course.deg(),1),"BAT: "+String(analogRead(35)*7.221/4096,1)+"  HUM: "+String(hum,1),0);
      } else {
        writedisplaytext(" ((TX))","","LAT: "+LatShown,"LON: "+LongShown,"SPD: "+String(gps.speed.kmph(),1)+"  CRS: "+String(gps.course.deg(),1),"BAT: "+String(analogRead(35)*7.221/4096,1)+" TEMP: "+String(temp,1),0);
      }
      sendpacket();
      Serial.println("State: Packet sent!");
      digitalWrite(TXLED, LOW);
    } else {
      if ( (lastTX+nextTX*2) <= millis()  ) {
      digitalWrite(TXLED, HIGH);
      batt_read();
      if (hum_temp) {
        writedisplaytext(" ((TX))","","LAT: "+LatShown,"LON: "+LongShown,"SPD: "+String(gps.speed.kmph(),1)+"  CRS: "+String(gps.course.deg(),1),"BAR: "+String(analogRead(35)*7.221/4096,1)+"  HUM: "+String(hum,1),0);
      } else {
        writedisplaytext(" ((TX))","","LAT: "+LatShown,"LON: "+LongShown,"SPD: "+String(gps.speed.kmph(),1)+"  CRS: "+String(gps.course.deg(),1),"BAR: "+String(analogRead(35)*7.221/4096,1)+" TEMP: "+String(temp,1),0);
      }
      sendpacket();
      Serial.println("State: Packet sent!");
      digitalWrite(TXLED, LOW);
      }
    }
  } else {
    digitalWrite(TXLED, HIGH);
    batt_read();
    if (hum_temp) {
      writedisplaytext(" ((TX))","","LAT: "+LatShown,"LON: "+LongShown,"No GPS used","BAT: "+String(analogRead(35)*7.221/4096,1)+"  HUM: "+String(hum,1),0);
    } else {
      writedisplaytext(" ((TX))","","LAT: "+LatShown,"LON: "+LongShown,"No GPS used","BAT: "+String(analogRead(35)*7.221/4096,1)+" TEMP: "+String(temp,1),0);
    }
    sendpacket();
    Serial.println("State: Packet sent!");
    digitalWrite(TXLED, LOW);
  }
}

  smartDelay(1000);

  if (tracker_mode != WX_FIXED) {
    if (millis() > 200000 && gps.charsProcessed() < 10) {
      writedisplaytext(" "+Tcall,"","Warning","No GPS Signal!","","",1000);
      Serial.println("Warning: No GPS Signal!");
    }
  }
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


/////////////////////////////////////////////////////////////////////////////////////////
//@APA Recalc GPS Position
void recalcGPS(){

  String Ns, Ew, helper;
  float Tlat, Tlon;
  int Talt;
  float Lat;
  float Lon;

  if (tracker_mode != WX_FIXED) {
    Tlat=gps.location.lat();
    Tlon=gps.location.lng();
    Talt=gps.altitude.meters();
    if(Tlat<0) { Ns = "S"; } else { Ns = "N"; }
    if(Tlon<0) { Ew = "W"; } else { Ew = "E"; }
    if(Tlat < 0) { Tlat= -Tlat; }
    unsigned int Deg_Lat = Tlat;
    Lat = 100*(Deg_Lat) + (Tlat - Deg_Lat)*60;

    if(Tlon < 0) { Tlon= -Tlon; }
    unsigned int Deg_Lon = Tlon;
    Lon = 100*(Deg_Lon) + (Tlon - Deg_Lon)*60;
  }

outString = "";

switch(tracker_mode) {
  case WX_FIXED:
    hum = dht.getHumidity();
    tempf = dht.getTemperature()*9/5+32;
    outString = (wxTcall);
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
    outString += "b......DHT22";
    break;
  case WX_TRACKER:
    if (wx) {
      hum = dht.getHumidity();
      tempf = dht.getTemperature()*9/5+32;
      outString = (wxTcall);
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
      outString += "b......DHT22";
      wx = !wx;
    } else {
      outString = (Tcall);
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
      outString += " /A=";
      outString += Talt;
      outString += "m Batt=";
      outString += String(BattVolts,2);
      outString += ("V");
      wx = !wx;
    }
  break;
case WX_MOVE:
    hum = dht.getHumidity();
    tempf = dht.getTemperature()*9/5+32;
    outString = (wxTcall);
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
    outString += "b......DHT22";
    break;
  case TRACKER:
  default:
    outString = (Tcall);
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
    outString += " /A=";
    outString += Talt;
    outString += "m Batt=";
    outString += String(BattVolts,2);
    outString += ("V");
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

  BattVolts = (BattRead / 4096);
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
  char werte_call[36] = {'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','1','2','3','4','5','6','7','8','9','0'};
  String werte_SSID[16] = {"0","1","2","3","4","5","6","7","8","9","10","11","12","13","14","15"};
  char werte_latlon[14] = {'0','1','2','3','4','5','6','7','8','9','N','S','E','W'};
  String werte_TxSymbol_text[5] = {"WX Station","Auto","Fussgaenger","Fahrrad","Motorrad"};
  String werte_TxSymbol_symbol[5] = {"_",">","[","b","<"};
  int8_t pos_in_string;
  int8_t pos_ssid, pos_latlon;
  bool key_pressed = false;
  int waiter;
  int initial_waiter = 2000;
  int ii;
  char aktueller_letter;
  int8_t pos_letter;
  String pfeile = "^";
  int8_t initial_ssid, initial_latlon, inital_TxSymbnol;


  // set callsign - one for both reports
  pos_in_string = 0;
  while (pos_in_string < 6) {
    key_pressed = false;
    aktueller_letter = (char) Tcall.charAt(pos_in_string);// ist Buchstabe holen
    for (pos_letter=0;pos_letter<36;pos_letter++) {
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
      if (pos_letter>=36) {pos_letter=0;}
      aktueller_letter=werte_call[pos_letter];
    }
    initial_waiter = 2000;
    pfeile = " "+pfeile;
    ++pos_in_string;
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

  // set Tx Symbol
  pos_ssid = 0;
  key_pressed = false;
  initial_waiter = 2000;
  while (true) {
    TxSymbol = werte_TxSymbol_symbol[pos_ssid];
    writedisplaytext("  SETUP", "    Symbol","   "+werte_TxSymbol_text[pos_ssid], "", "PRESS KEY to select", "", 0);
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
    if (pos_ssid>=5) {pos_ssid=0;}
    }

  // write all values to NVRAM
  prefs.begin("nvs", false);
  prefs.putString("Tcall", Tcall);
  prefs.putString("wxTcall", wxTcall);
  prefs.putString("LatFixed", LatFixed);
  prefs.putString("LongFixed", LongFixed);
  prefs.putString("TxSymbol", TxSymbol);
  prefs.end();
  writedisplaytext("  SETUP", "DONE","", "stored in NVS", "", "", 2500);
}
