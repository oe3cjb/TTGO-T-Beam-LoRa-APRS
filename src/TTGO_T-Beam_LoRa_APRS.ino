
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

#include <TTGO_T-Beam_LoRa_APRS_config.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <BG_RF95.h>
// #include <string>

#include <TinyGPS++.h>
// #include <SoftwareSerial.h>
#include <math.h>
//#include <DHT.h>
#include <DHTesp.h>
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
float hum=0;                  //Stores humidity value
float temp=99.99;                 //Stores temperature value

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

#define DHTPIN 25            // pin the DHT22 is connected to Pin25
//#define DHTTYPE DHT22        // DHT 22  (AM2302)

// Variables and Constants

String InputString = "";     //data on buff is copied to this string
String Outputstring = "";
String outString="";         //The new Output String with GPS Conversion RAW

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


float BattVolts;

static adc1_channel_t adc_channel = ADC1_GPIO35_CHANNEL;
static const adc_atten_t atten = ADC_ATTEN_DB_6;
static const adc_unit_t unit = ADC_UNIT_1;

static void smartDelay(unsigned long);
void recalcGPS(void);
void sendpacket(void);
void loraSend(byte, byte, byte, byte, byte, long, byte, float);
void batt_read(void);
void writedisplaytext(String, String, String, String, String, int);


// #if (SEND_WX)
// DHT dht(DHTPIN, DHTTYPE);    // Initialize DHT sensor for normal 16mhz Arduino
DHTesp dht;
// #endif

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
  digitalWrite(TXLED, LOW);  // turn blue LED off
  Serial.begin(115200);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
     for(;;); // Don't proceed, loop forever
  }
  digitalWrite(TXLED, HIGH);
  writedisplaytext("","Init:","Display OK!","","",1000);
  digitalWrite(TXLED, LOW);
  Serial.println("Init: Display OK!");
  if (!rf95.init()) {
    // Serial.println("init failed");

    writedisplaytext("","Init:","RF95 FAILED!",":-(","",1000);
    Serial.println("Init: RF95 FAILED!");
    for(;;); // Don't proceed, loop forever
  }

  digitalWrite(TXLED, HIGH);
  writedisplaytext("","Init:","RF95 OK!","","",1000);
  digitalWrite(TXLED, LOW);
  Serial.println("Init: RF95 OK!");

  #if !(FIXED_POSITION)
    ss.begin(GPSBaud, SERIAL_8N1, 12, 15);        //Startup HW serial for GPS
  #endif //   #if !(FIXED_POSITION)
  digitalWrite(TXLED, HIGH);
  writedisplaytext("","Init:","GPS Serial OK!","","",1000);
  digitalWrite(TXLED, LOW);
  Serial.println("Init: GPS Serial OK!");

  // adc_power_on();
  // adc_gpio_init(ADC_UNIT_1,ADC_CHANNEL_7);
  //adc_set_clk_div(1);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_6);
  // adc1_config_channel_atten(adc_channel, atten);
  writedisplaytext("","Init:","ADC OK!",String(analogRead(35)*7.221/4096,4),"",1000);
  Serial.println("Init: ADC OK!");

  rf95.setFrequency(433.775);
  rf95.setModemConfig(BG_RF95::Bw125Cr45Sf4096); // hard coded because of double definition
  // rf95.setModemConfig(ModemConfig); // das ist irgendwo doppelt definiert ???
  rf95.setTxPower(5);
  //rf95.printRegisters();
  //rf95.setPromiscuousbg();

//#if (SEND_WX)
  //dht.begin();               // DHT22 initialisieren
  dht.setup(DHTPIN,dht.AUTO_DETECT);
  delay(250);
  //temp = dht.readTemperature();
  temp = dht.getTemperature();
  writedisplaytext("","Init:","DHT OK!",String(temp),"",750);
  Serial.print("Init: DHT OK! Temp=");
  Serial.println(String(temp));
//#else                        //#if (SEND_WX)
//  writedisplaytext("","Init:","no DHT configuration","","",1000);
//  Serial.println("Init: no DHT configuration");
//#endif                       //#if (SEND_WX)

digitalWrite(TXLED, HIGH);
writedisplaytext("","Init:","All DONE OK!",":-D","",1000);
digitalWrite(TXLED, LOW);
Serial.println("Init: ALL DONE OK! :-D");
writedisplaytext("","","","","",0);
}


//   LOOP

void loop()
{

// temp = dht.readTemperature();
temp = dht.getTemperature();
Serial.print("Init: DHT OK! Temp=");
Serial.println(String(temp));

#if DEBUG
  writedisplaytext("","DEBUG",millis(),String(millis()),"",0);
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

writedisplaytext("     "+String(((lastTX+nextTX)-millis())/1000),"LAT: "+String(gps.location.lat(),5),"LON: "+String(gps.location.lng(),5),"SPD: "+String(gps.speed.kmph(),1)+" CRS: "+String(gps.course.deg(),0),"BAT: "+String(analogRead(35)*7.221/4096,2)+" TEMP: "+String(temp),250);
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
  batt_read();
  writedisplaytext("     ((TX))","LAT: "+String(gps.location.lat(),5),"LON: "+String(gps.location.lng(),5),"SPD: "+String(gps.speed.kmph(),1)+" CRS: "+String(gps.course.deg(),0),"BAR: "+String(analogRead(35)*7.221/4096,2)+" TEMP: "+String(temp),250);
  sendpacket();
  //writedisplaytext("","State:","Packet sent!","","",250);
  Serial.println("State: Packet sent!");
  digitalWrite(TXLED, LOW);
} else {
  if ( (lastTX+nextTX*2) <= millis()  )
  {
    digitalWrite(TXLED, HIGH);
    batt_read();
    writedisplaytext("     ((TX))","LAT: "+String(gps.location.lat(),5),"LON: "+String(gps.location.lng(),5),"SPD: "+String(gps.speed.kmph(),1)+" CRS: "+String(gps.course.deg(),0),"BAT: "+String(analogRead(35)*7.221/4096,2)+" TEMP: "+String(temp),250);
    sendpacket();
    //  writedisplaytext("","State:","Packet sent!","","",250);
    Serial.println("State: Packet sent!");
    digitalWrite(TXLED, LOW);
  }
}

  smartDelay(1000);

  #if !(FIXED_POSITION)
    if (millis() > 200000 && gps.charsProcessed() < 10)
    {
      writedisplaytext("","Warning","No GPS Signal!","","",1000);
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
    temp = dht.getTemperature();
//    temp = dht.readTemperature();
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
  float BattRead = analogRead(35)*7.221;
  // Serial.println("Batt: "+String(BattRead));
  //int BattRead = adc1_get_raw(ADC1_CHANNEL_7);
  // delay(250);
  //lora_TXBUFF[1] = (BattRead / 256);                     //MSB of battery volts
  //lora_TXBUFF[0] = (BattRead - (lora_TXBUFF[1] * 256));  //LSB of battery volts

  BattVolts = (BattRead / 4096);
  // BattVolts = BattRead;
  // writedisplaytext(String(BattVolts,2), "", "", "", "", 0);

  //Serial.print("lora_TXBUFF[0]  ");
  //Serial.println(lora_TXBUFF[0]);
  //Serial.print("lora_TXBUFF[1]  ");
  //Serial.println(lora_TXBUFF[1]);
  //Serial.println("Battery ");
  //Serial.print(BattVolts, 2);
  //Serial.println("V");
}

///////////////////////////////////////////////////////////////////////////////////////
void writedisplaytext(String Line1, String Line2, String Line3, String Line4, String Line5, int warten)
{
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0,0);
  display.println(" LoRa-APRS");
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
