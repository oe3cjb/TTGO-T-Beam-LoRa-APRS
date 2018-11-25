
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
// last update: 25.11.2018
// modifications: select mode during compilation to select model

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

// STOP EDITING from here on - except you know what you do :-)
#define DEBUG false           // used for debugging purposes , e.g. turning on special serial or display logging

//Hardware definitions

/* for feather32u4
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
*/

//Variables for DHT22 temperature and humidity sensor
int chk;
float hum;                  //Stores humidity value
float temp;                 //Stores temperature value

//other global Variables
String Textzeile1, Textzeile2;

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
// const byte GPSLED = 6;      // pin gps & Heartbeat
// const byte GPSLED1 = 9;     // pin gps & Heartbeat

// Pins for LoRa module
const byte lora_PReset = 23; //pin where LoRa device reset line is connected
const byte lora_PNSS = 18;   //pin number where the NSS line for the LoRa device is connected.
                             // pin 11  MOSI
                             // pin 12  MISO
                             // pin 13  SCLK

// #define ModemConfig BG_RF95::Bw125Cr45Sf4096

#define DHTPIN 10            // what pin we're connected to
#define DHTTYPE DHT22        // DHT 22  (AM2302)

// Variables and Constants

String InputString = "";     //data on buff is copied to this string
String Outputstring = "";
String outString="";         //The new Output String with GPS Conversion RAW
float BattVolts;

#if (FIXED_POSITION)
boolean wx = true;
#else
boolean wx = false;
#endif

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

// Includes

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <BG_RF95.h>
// #include <string>

#include <TinyGPS++.h>
// #include <SoftwareSerial.h>
#include <math.h>
#include <DHT.h>
#include <driver/adc.h>
#include <Wire.h>

#include <Adafruit_SSD1306.h>
#include <splash.h>

#include "xtest_bw.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>

static void smartDelay(unsigned long);
void recalcGPS(void);
void sendpacket(void);
void loraSend(byte, byte, byte, byte, byte, long, byte, float);
void batt_read(void);
void writedisplaytext(String, String, String, int);


#if (SEND_WX)
DHT dht(DHTPIN, DHTTYPE);    // Initialize DHT sensor for normal 16mhz Arduino
#endif

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

  pinMode(TXLED, OUTPUT);
  digitalWrite(TXLED, LOW);
  Serial.begin(115200);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
     for(;;); // Don't proceed, loop forever
  }
  digitalWrite(TXLED, HIGH);
  writedisplaytext("Init:","Display OK!","",1000);
  digitalWrite(TXLED, LOW);
  Serial.println("Init: Display OK!");
  if (!rf95.init()) {
    // Serial.println("init failed");

    writedisplaytext("Init:","RF95 FAILED!",":-(",1000);
    Serial.println("Init: RF95 FAILED!");
    for(;;); // Don't proceed, loop forever
  }

  digitalWrite(TXLED, HIGH);
  writedisplaytext("Init:","RF95 OK!","",1000);
  digitalWrite(TXLED, LOW);
  Serial.println("Init: RF95 OK!");

  #if !(FIXED_POSITION)
    ss.begin(GPSBaud, SERIAL_8N1, 12, 15);        //Startup HW serial for GPS
  #endif //   #if !(FIXED_POSITION)
  digitalWrite(TXLED, HIGH);
  writedisplaytext("Init:","GPS Serial OK!","",1000);
  digitalWrite(TXLED, LOW);
  Serial.println("Init: GPS Serial OK!");

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_6);
  writedisplaytext("Init:","ADC OK!","",1000);
  Serial.println("Init: ADC OK!");

  rf95.setFrequency(433.775);
  rf95.setModemConfig(BG_RF95::Bw125Cr45Sf4096); // hard coded because of double definition
  // rf95.setModemConfig(ModemConfig); // das ist irgendwo doppelt definiert ???
  rf95.setTxPower(5);
  //rf95.printRegisters();
  //rf95.setPromiscuousbg();

#if (SEND_WX)
  dht.begin();               // DHT22 initialisieren
  writedisplaytext("Init:","DHT OK!","",1000);
  Serial.println("Init: DHT OK!");
#else                        //#if (SEND_WX)
  writedisplaytext("Init:","no DHT configuration","",1000);
  Serial.println("Init: no DHT configuration");
#endif                       //#if (SEND_WX)

digitalWrite(TXLED, HIGH);
writedisplaytext("Init:","All DONE OK!",":-D",1000);
digitalWrite(TXLED, LOW);
Serial.println("Init: ALL DONE OK! :-D");
writedisplaytext("","","",0);
}


//   LOOP

void loop()
{
#if DEBUG
  writedisplaytext("DEBUG","millis()",String(millis()),0);
#endif
 //while(1) { if ( ss.available() ) Serial.write(ss.read());}
#if !(FIXED_POSITION)
  // digitalWrite(GPSLED, HIGH);

  while (ss.available() > 0) {
    gps.encode(ss.read());
  }
#endif                                                // #if !(FIXED_POSITION)

if (rf95.waitAvailableTimeout(100))
{
  // Should be a reply message for us now
  if (rf95.recvAPRS(buf, &len))
  {
    // Serial.print("RX: ");
    // Serial.println((char*)buf);
    // Serial.print("RSSI: ");
    // Serial.println(rf95.lastRssi(), DEC);

  }
}

display.clearDisplay();
display.setTextColor(WHITE);
display.setTextSize(2);
display.setCursor(0,0);
display.println("LoRa-APRS");
display.setTextSize(1);
display.setCursor(0,36);
display.print("LAT: ");
display.println(String(gps.location.lat(),5));
display.setCursor(0,46);
display.print("LON: ");
display.println(String(gps.location.lng(),5));
display.setCursor(0,56);
display.print("SPD: ");
display.print(String(gps.speed.kmph(),1));
display.print(" CRS: ");
display.println(String(gps.course.deg(),0));
display.display();

smartDelay(1000);

// digitalWrite(GPSLED, LOW);
#if (FIXED_POSITION)
  // if (gps.location.isUpdated() || ( (lastTX+nextTX) <= millis()  ) )
  if ( (lastTX+nextTX) <= millis()  )
#else
  if (gps.location.isValid() && ( (lastTX+nextTX) <= millis()  ) )
#endif
{
  digitalWrite(TXLED, HIGH);
  sendpacket();
  writedisplaytext("State:","Packet sent!","",250);
  Serial.println("State: Packet sent!");
  digitalWrite(TXLED, LOW);
} else {
  if ( (lastTX+nextTX*2) <= millis()  )
  {
    digitalWrite(TXLED, HIGH);
    sendpacket();
    writedisplaytext("State:","Packet sent!","",250);
    Serial.println("State: Packet sent!");
    digitalWrite(TXLED, LOW);
  }
}

  smartDelay(1000);

  #if !(FIXED_POSITION)
    if (millis() > 200000 && gps.charsProcessed() < 10)
    {
      writedisplaytext("Warning","No GPS Signal!","",1000);
      Serial.println("Warning: No GPS Signal!");
    }
  #endif
}

/////////////////////////////////////////////////////////////////////////////////////////
// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    #if !(FIXED_POSITION)
      while (ss.available())
        gps.encode(ss.read());
    #endif
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

  #if !(FIXED_POSITION)
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
  #endif

#if !(SEND_WX)
  outString = "";
  outString = (Tcall);
  outString += ">APRS:!";
  #if (FIXED_POSITION)
    outString += LATITUDE;
  #else
    if(Tlat<10) {outString += "0"; }
    outString += String(Lat,2);
    outString += Ns;
  #endif
  outString += wxTable;
  #if (FIXED_POSITION)
    outString += LONGITUDE;
  #else
    if(Tlon<100) {outString += "0"; }
    if(Tlon<10) {outString += "0"; }
    outString += String(Lon,2);
    outString += Ew;
  #endif
  outString += sSymbol;
  outString += " /A=";
  outString += Talt;
  outString += "m Batt=";
  outString += String(BattVolts,2);
  outString += ("V");
#else
  if ( !wx ) {              // create standard position string
  #if !(FIXED_POSITION)
      outString = "";
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
      outString += sSymbol;
      outString += " /A=";
      outString += Talt;
      outString += "m Batt=";
      outString += String(BattVolts,2);
      outString += ("V");
      wx = true;
  #endif
  } else {                  // create weather report string
    hum = dht.readHumidity();
// hum = 88.67;
// temp = 50.23;
    temp = (dht.readTemperature() * 9/5) +32;
    outString = "";
    outString = (wxTcall);
    outString += ">APRS:!";
    #if (FIXED_POSITION)
      outString += LATITUDE;
    #else
      if(Tlat<10) {outString += "0"; }
      outString += String(Lat,2);
      outString += Ns;
    #endif
    outString += wxTable;
    #if (FIXED_POSITION)
      outString += LONGITUDE;
    #else
      if(Tlon<100) {outString += "0"; }
      if(Tlon<10) {outString += "0"; }
      outString += String(Lon,2);
      outString += Ew;
    #endif
    outString += wxSymbol;
    outString += ".../...g...t";
    if (temp < 0) {     // negative Werte erstellen
      outString += "-";
      if(temp>-10) {outString += "0"; }
      temp = abs(temp);
    } else {                // positive Werte erstellen
      if(temp<100) {outString += "0"; }
      if(temp<10) {outString += "0"; }
    }
    helper = String(temp,0);
    helper.trim();
    outString += helper;
    outString += "r...p...P...h";
    if(hum<10) {outString += "0"; }
    helper = String(hum,0);
    helper.trim();
    outString += helper;
    outString += "b......DHT22";
    #if !(FIXED_POSITION)
      wx = false;
    #endif
  }
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////
void sendpacket()
{

  batt_read();
  Outputstring = "";

#if !(FIXED_POSITION)
  if ( gps.location.isValid()   || gps.location.isUpdated() )
  {
    // digitalWrite(GPSLED, HIGH);
    //New System
    //recalcEncodedGPS();
#endif
    recalcGPS();                        //
    // digitalWrite(PLED1, HIGH);
    Outputstring =outString;

        loraSend(lora_TXStart, lora_TXEnd, 60, 255, 1, 10, TXdbmW, TXFREQ);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
 #if !(FIXED_POSITION)
  }  else {
    Outputstring = (Tcall);
    Outputstring += " No GPS-Fix";
    Outputstring += " Batt=";
    Outputstring += String(BattVolts,2);
    Outputstring += ("V ");

    loraSend(lora_TXStart, lora_TXEnd, 60, 255, 1, 10, 5, TXFREQ);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
    // digitalWrite(GPSLED, LOW);
  }
    #endif

    // digitalWrite(PLED1, LOW);
}

///////////////////////////////////////////////////////////////////////////////////////
void loraSend(byte lora_LTXStart, byte lora_LTXEnd, byte lora_LTXPacketType, byte lora_LTXDestination, byte lora_LTXSource, long lora_LTXTimeout, byte lora_LTXPower, float lora_FREQ)
{
  byte i;
  byte ltemp;

 if (rf95.waitAvailableTimeout(100))
  {
    if (rf95.recvAPRS(buf, &len))
   {
//      Serial.print("RX before TX: ");
//      Serial.println((char*)buf);
//      Serial.print("RSSI: ");
//      Serial.println(rf95.lastRssi(), DEC);

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

//  Serial.print(Outputstring);
//  Serial.print(" len: ");
//  Serial.println(strlen(lora_TXBUFF) );

  //digitalWrite(RX_en, LOW);       //RX lo
  //digitalWrite(TX_en, HIGH);       //TX HIGH
  //rf95.sendAPRS(lora_TXBUFF, sizeof(lora_TXBUFF));
  rf95.sendAPRS(lora_TXBUFF, Outputstring.length());

  // rf95.sendAPRS(lora_TXBUFF, lora_TXBUFF.length());
  rf95.waitPacketSent();

  //digitalWrite(TX_en, LOW);       //TX lo
  //digitalWrite(RX_en,HIGH);       //RX HIGH




   // digitalWrite(PLED1, LOW);

}
///////////////////////////////////////////////////////////////////////////////////////
void batt_read()
{
  //int BattRead = analogRead(ANALOG_PI);
  int BattRead = adc1_get_raw(ADC1_CHANNEL_7);
  //lora_TXBUFF[1] = (BattRead / 256);                     //MSB of battery volts
  //lora_TXBUFF[0] = (BattRead - (lora_TXBUFF[1] * 256));  //LSB of battery volts

  BattVolts = (BattRead * (2.2 / 4096.0));

  //Serial.print("lora_TXBUFF[0]  ");
  //Serial.println(lora_TXBUFF[0]);
  //Serial.print("lora_TXBUFF[1]  ");
  //Serial.println(lora_TXBUFF[1]);
  //Serial.println("Battery ");
  //Serial.print(BattVolts, 2);
  //Serial.println("V");
}

///////////////////////////////////////////////////////////////////////////////////////
void writedisplaytext(String Line1, String Line2, String Line3, int warten)
{
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println("LoRa-APRS");
  display.setTextSize(1);
  display.setCursor(0,36);
  display.println(Line1);
  display.setCursor(0,46);
  display.println(Line2);
  display.setCursor(0,56);
  display.println(Line3);
  display.display();
  smartDelay(warten);
}
