// Tracker for LoRA APRS
// from OE1ACM and OE3CJB redesigned by SQ9MDD
// KISS ans Bluetooth by SQ5RWU
// TTGO T-Beam v1.0 only
//
// licensed under CC BY-NC-SA

// Includes
#include <TTGO_T-Beam_LoRa_APRS_config.h> // to config user parameters
#include <Arduino.h>
#include <SPI.h>
#include <BG_RF95.h>         // library from OE1ACM
#include <math.h>
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
#include "taskGPS.h"
#ifdef KISS_PROTOCOL
#include "taskTNC.h"
#endif


// I2C LINES
#define I2C_SDA 21
#define I2C_SCL 22

// DISPLAY address
#define SSD1306_ADDRESS 0x3C

//other global Variables

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

// Pins for LoRa module
const byte lora_PReset = 23; //pin where LoRa device reset line is connected
const byte lora_PNSS = 18;   //pin number where the NSS line for the LoRa device is connected.

// Variables for APRS packaging
String Tcall;                //your Call Sign for normal position reports
String sTable="/";           //Primer
String relay_path;
boolean gps_state = true;
boolean key_up = true;
boolean t_lock = false;

// Variables and Constants
String loraReceivedFrameString = "";     //data on buff is copied to this string
String Outputstring = "";
String outString="";         //The new Output String with GPS Conversion RAW
String LongShown="";
String LatShown="";
String LongFixed="";
String LatFixed="";

#if defined(ENABLE_TNC_SELF_TELEMETRY) && defined(KISS_PROTOCOL)
  time_t nextTelemetryFrame;
#endif

//byte arrays
byte  lora_TXBUFF[BG_RF95_MAX_MESSAGE_LEN];      //buffer for packet to send
byte  lora_RXBUFF[BG_RF95_MAX_MESSAGE_LEN];      //buffer for packet to send

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
ulong max_time_to_nextTX= MAX_TIME_TO_NEXT_TX;
ulong nextTX=60000L;                  // preset time period between TX = 60000ms = 60secs = 1min
ulong time_to_refresh = 0;
ulong next_fixed_beacon = 0;
ulong fix_beacon_interval = FIX_BEACON_INTERVAL;
ulong time_delay = 0;
#define ANGLE 60                      // angle to send packet at smart beaconing
#define ANGLE_AVGS 3                  // angle averaging - x times
float average_course[ANGLE_AVGS];
float avg_c_y, avg_c_x;

static const adc_atten_t atten = ADC_ATTEN_DB_6;
static const adc_unit_t unit = ADC_UNIT_1;

#ifdef T_BEAM_V1_0
AXP20X_Class axp;
#endif

// checkRX
uint8_t loraReceivedLength = sizeof(lora_RXBUFF);

// Singleton instance of the radio driver
BG_RF95 rf95(18, 26);        // TTGO T-Beam has NSS @ Pin 18 and Interrupt IO @ Pin26

// initialize OLED display
#define OLED_RESET 16         // not used
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

// + FUNCTIONS-----------------------------------------------------------+//

char *ax25_base91enc(char *s, uint8_t n, uint32_t v){
  /* Creates a Base-91 representation of the value in v in the string */
  /* pointed to by s, n-characters long. String length should be n+1. */
  for(s += n, *s = '\0'; n; n--)
  {
    *(--s) = v % 91 + 33;
    v /= 91;
  }
  return(s);
}

void prepareAPRSFrame(){
  String Ns, Ew, helper;
  char helper_base91[] = {"0000\0"};
  double Tlat=52.0000, Tlon=20.0000;
  uint32_t aprs_lat, aprs_lon;
  double Tspeed=0, Tcourse=0;
  String Speedx, Coursex;
  int i;
  #ifdef SHOW_ALT
    String Altx;
    int Talt;
  #endif

  Tlat=gps.location.lat();
  Tlon=gps.location.lng();

  Tcourse=gps.course.deg();
  Tspeed=gps.speed.knots();
  if(Tlat<0) { Ns = "S"; } else { Ns = "N"; }
  if(Tlon<0) { Ew = "W"; } else { Ew = "E"; }
  if(Tlat < 0) { Tlat= -Tlat; }

  if(Tlon < 0) { Tlon= -Tlon; }
    aprs_lat = 900000000 - Tlat * 10000000;
    aprs_lat = aprs_lat / 26 - aprs_lat / 2710 + aprs_lat / 15384615;
    aprs_lon = 900000000 + Tlon * 10000000 / 2;
    aprs_lon = aprs_lon / 26 - aprs_lon / 2710 + aprs_lon / 15384615;
  //}

  outString = "";
  outString += Tcall;

  #ifdef DIGI_PATH
    outString += ">APLM0," + relay_path + ":!";
  #elif
    outString += ">APLM0:!";
  #endif

  if(gps_state && gps.location.isValid()){
    outString += APRS_SYMBOL_TABLE;
    ax25_base91enc(helper_base91, 4, aprs_lat);
    for (i=0; i<4; i++) {
      outString += helper_base91[i];
    }
    ax25_base91enc(helper_base91, 4, aprs_lon);
    for (i=0; i<4; i++) {
      outString += helper_base91[i];
    }
    outString += APRS_SYMBOL;
    ax25_base91enc(helper_base91, 1, (uint32_t) Tcourse/4 );
    outString += helper_base91[0];
    ax25_base91enc(helper_base91, 1, (uint32_t) (log1p(Tspeed)/0.07696));
    outString += helper_base91[0];
    outString += "H";
    #ifdef SHOW_ALT
      Talt=gps.altitude.meters() * 3.28d;
      Altx = Talt;
      outString += "/A=";
      for (i = 0; i < (6-Altx.length()); ++i) {
        outString += "0";
      }
      outString += Talt;
    #endif
  }else{
    outString += LATIDUDE_PRESET;
    outString += APRS_SYMBOL_TABLE;
    outString += LONGITUDE_PRESET;
    outString += APRS_SYMBOL;
  }

  outString += MY_COMMENT;

  #ifdef SHOW_BATT            // battery is not frame part move after comment
    outString += " Batt=";
    outString += String(BattVolts, 2);
    outString += ("V");
  #endif

  #ifdef KISS_PROTOCOL
    sendToTNC(outString);
  #else
    Serial.println(outString);
  #endif
}

void sendpacket(){
  batt_read();

  prepareAPRSFrame();
  loraSend(TXdbmW, TXFREQ, outString);  //send the packet, data is in TXbuff from lora_TXStart to lora_TXEnd
}

/**
 * Send message as APRS LoRa packet
 * @param lora_LTXPower
 * @param lora_FREQ
 * @param message
 */
void loraSend(byte lora_LTXPower, float lora_FREQ, const String &message) {
  digitalWrite(TXLED, LOW);
  lastTX = millis();

  int messageSize = min(message.length(), sizeof(lora_TXBUFF) - 1);
  message.toCharArray((char*)lora_TXBUFF, messageSize + 1, 0);
  rf95.setModemConfig(BG_RF95::Bw125Cr45Sf4096);
  rf95.setFrequency(lora_FREQ);
  rf95.setTxPower(lora_LTXPower);
  rf95.sendAPRS(lora_TXBUFF, messageSize);
  rf95.waitPacketSent();
  digitalWrite(TXLED, HIGH);
}

void batt_read(){
#ifdef T_BEAM_V1_0
  BattVolts = axp.getBattVoltage()/1000;
#else
  BattVolts = analogRead(35)*7.221/4096;
#endif
}

void writedisplaytext(String HeaderTxt, String Line1, String Line2, String Line3, String Line4, String Line5, int warten) {
  batt_read();
  if (BattVolts < 3.5){
    #ifdef T_BEAM_V1_0
      axp.setChgLEDMode(AXP20X_LED_BLINK_4HZ);
    #endif
  }

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
  time_to_refresh = millis() + SHOW_RX_TIME;
}


String getSatAndBatInfo() {
  String line5;
  if(gps_state == true){
    line5 = "SAT: " + String(gps.satellites.value()) + "  BAT: " + String(BattVolts, 1) + "V";
  }else{
    line5 = "SAT: X  BAT: " + String(BattVolts, 1) + "V";
  }
  #if defined(ENABLE_BLUETOOTH) && defined(KISS_PROTOCOL)
    if (SerialBT.hasClient()){
      line5 += "BT";
    }
  #endif
  return line5;
}

void displayInvalidGPS() {
  writedisplaytext(" " + Tcall, "(TX) at valid GPS", "LAT: not valid", "LON: not valid", "SPD: ---  CRS: ---", getSatAndBatInfo(), 1);
  #ifdef SHOW_GPS_DATA
    Serial.print("(TX) at valid GPS / LAT: not valid / Lon: not valid / SPD: --- / CRS: ---");
    Serial.print(" / SAT: ");
    Serial.print(String(gps.satellites.value()));
    Serial.print(" / BAT: ");
    Serial.println(String(BattVolts,1));
  #endif
}

#if defined(KISS_PROTOCOL)
/**
 *
 * @param TNC2FormatedFrame
 */
void sendToTNC(const String& TNC2FormatedFrame) {
  if (tncToSendQueue){
    auto *buffer = new String();
    buffer->concat(TNC2FormatedFrame);
    if (xQueueSend(tncReceivedQueue, &buffer, (1000 / portTICK_PERIOD_MS)) != pdPASS){
      delete buffer;
    }
  }
}
#endif

String prepareCallsign(const String& callsign){
  String tmpString = "";
  for (int i=0; i<callsign.length();++i){  // remove unneeded "spaces" from callsign field
    if (callsign.charAt(i) != ' ') {
      tmpString += callsign.charAt(i);
    }
  }
  return tmpString;
}

#if defined(ENABLE_TNC_SELF_TELEMETRY) && defined(KISS_PROTOCOL)
void sendTelemetryFrame() {
  #ifdef T_BEAM_V1_0
    uint8_t b_volt = (axp.getBattVoltage() - 3000) / 5.1;
    uint8_t b_in_c = (axp.getBattChargeCurrent()) / 10;
    uint8_t b_out_c = (axp.getBattDischargeCurrent()) / 10;
    uint8_t ac_volt = (axp.getVbusVoltage() - 3000) / 28;
    uint8_t ac_c = (axp.getVbusCurrent()) / 10;

    String telemetryParamsNames = String(":") + Tcall + ":PARM.B Volt,B In,B Out,AC V,AC C";
    String telemetryUnitNames = String(":") + Tcall + ":UNIT.mV,mA,mA,mV,mA";
    String telemetryEquations = String(":") + Tcall + ":EQNS.0,5.1,3000,0,10,0,0,10,0,0,28,3000,0,10,0";
    String telemetryData = String("T#MIC") + String(b_volt) + ","+ String(b_in_c) + ","+ String(b_out_c) + ","+ String(ac_volt) + ","+ String(ac_c) + ",00000000";
    String telemetryBase = "";
    telemetryBase += Tcall + ">APLM0" + ":";
    sendToTNC(telemetryBase + telemetryParamsNames);
    sendToTNC(telemetryBase + telemetryUnitNames);
    sendToTNC(telemetryBase + telemetryEquations);
    sendToTNC(telemetryBase + telemetryData);
  #else
  #endif
}
#endif

// + SETUP --------------------------------------------------------------+//

void setup(){
  for (int i=0;i<ANGLE_AVGS;i++) {                                        // set average_course to "0"
    average_course[i]=0;
  }

  pinMode(TXLED, OUTPUT);
  pinMode(BUTTON, INPUT);
  digitalWrite(TXLED, LOW);                                               // turn blue LED off
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  #ifdef T_BEAM_V1_0
    if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    }
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);                           // switch on GPS
    axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
    axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    axp.setDCDC1Voltage(3300);
    // Enable ADC to measure battery current, USB voltage etc.
    axp.adc1Enable(0xfe, true);
    axp.adc2Enable(0x80, true);
  #endif

  if(!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_ADDRESS)) {
     for(;;);                                                             // Don't proceed, loop forever
  }

  writedisplaytext("LoRa-APRS","","Init:","Display OK!","","",1000);
  Tcall = prepareCallsign(String(CALLSIGN));
  relay_path = DIGI_PATH;

  if (!rf95.init()) {
    writedisplaytext("LoRa-APRS","","Init:","RF95 FAILED!",":-(","",250);
    for(;;); // Don't proceed, loop forever
  }

  if (max_time_to_nextTX < nextTX){
    max_time_to_nextTX=nextTX;
  }
  writedisplaytext("LoRa-APRS","","Init:","RF95 OK!","","",250);
  writedisplaytext(" "+Tcall,"","Init:","Waiting for GPS","","",250);
  xTaskCreate(taskGPS, "taskGPS", 10000, nullptr, 1, nullptr);
  writedisplaytext(" "+Tcall,"","Init:","GPS Task Created!","","",250);
  #ifndef T_BEAM_V1_0
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_DB_6);
  #endif
  batt_read();
  writedisplaytext("LoRa-APRS","","Init:","ADC OK!","BAT: "+String(BattVolts,1),"",250);
  rf95.setFrequency(433.775);
  rf95.setModemConfig(BG_RF95::Bw125Cr45Sf4096); // hard coded because of double definition
  rf95.setTxPower(TXdbmW);
  delay(250);
  #ifdef KISS_PROTOCOL
    xTaskCreatePinnedToCore(taskTNC, "taskTNC", 10000, nullptr, 1, nullptr, xPortGetCoreID());
  #endif

  #if defined(ENABLE_BLUETOOTH) && defined(KISS_PROTOCOL)
    #ifdef BLUETOOTH_PIN
      SerialBT.setPin(BLUETOOTH_PIN);
    #endif
    SerialBT.begin(String("TTGO LORA APRS ") + CALLSIGN);
    writedisplaytext("LoRa-APRS","","Init:","BT OK!","","",250);
  #endif
  writedisplaytext("LoRa-APRS","","Init:","FINISHED OK!","   =:-)   ","",250);
  writedisplaytext("","","","","","",0);
  time_to_refresh = millis() + SHOW_RX_TIME;
  displayInvalidGPS();
  digitalWrite(TXLED, HIGH);
  #ifdef T_BEAM_V1_0
    axp.setChgLEDMode(AXP20X_LED_OFF);
  #endif
}

// +---------------------------------------------------------------------+//
// + MAINLOOP -----------------------------------------------------------+//
// +---------------------------------------------------------------------+//

void loop() {
  if(digitalRead(BUTTON)==LOW && key_up == true){
    key_up = false;
    delay(50);
    if(digitalRead(BUTTON)==LOW){
      delay(300);
      time_delay = millis() + 1500;
      if(digitalRead(BUTTON)==HIGH){
        if(gps_state == true && gps.location.isValid()){
            writedisplaytext("((MAN TX))","","","","","",1);
            sendpacket();
        }else{
            writedisplaytext("((FIX TX))","","","","","",1);
            sendpacket();
        }
        key_up = true;
      }
    }
  }
    //delay(1500);
    if(digitalRead(BUTTON)==LOW && key_up == false && millis() >= time_delay && t_lock == false){
      t_lock = true;
        if(gps_state == true){
          gps_state = false;
          #ifdef T_BEAM_V1_0
            axp.setPowerOutPut(AXP192_LDO3, AXP202_OFF);                    // GPS OFF
          #endif
          writedisplaytext("((GPSOFF))","","","","","",1);
          next_fixed_beacon = millis() + fix_beacon_interval;

        }else{
          gps_state = true;
          #ifdef T_BEAM_V1_0
            axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
          #endif
          writedisplaytext("((GPS ON))","","","","","",1);                // GPS ON
        }
    }
  
  if(digitalRead(BUTTON)==HIGH && key_up == false){
    key_up = true;
    t_lock = false;
  }

  #ifdef FIXED_BEACON_EN
    if(millis() >= next_fixed_beacon && gps_state == false){
      next_fixed_beacon = millis() + fix_beacon_interval;
      writedisplaytext("((AUT TX))","","","","","",1);
      sendpacket();
    }
  #endif

  #ifdef KISS_PROTOCOL
    String *TNC2DataFrame = nullptr;
    if (tncToSendQueue) {
      if (xQueueReceive(tncToSendQueue, &TNC2DataFrame, (1 / portTICK_PERIOD_MS)) == pdPASS) {
        writedisplaytext("((KISSTX))","","","","","",1);
        time_to_refresh = millis() + SHOW_RX_TIME;
        loraSend(TXdbmW, TXFREQ, *TNC2DataFrame);
        delete TNC2DataFrame;
      }
    }
  #endif


  if (rf95.waitAvailableTimeout(100)) {
    #ifdef T_BEAM_V1_0
      axp.setChgLEDMode(AXP20X_LED_LOW_LEVEL);
    #endif
    #ifdef SHOW_RX_PACKET                                                 // only show RX packets when activitated in config
      loraReceivedLength = sizeof(lora_RXBUFF);                           // reset max length before receiving!
      if (rf95.recvAPRS(lora_RXBUFF, &loraReceivedLength)) {
        loraReceivedFrameString = "";
        for (int i=0 ; i < loraReceivedLength ; i++) {
          loraReceivedFrameString += (char) lora_RXBUFF[i];
        }
        writedisplaytext("  ((RX))", "", loraReceivedFrameString, "", "", "", SHOW_RX_TIME);
        #ifdef KISS_PROTOCOL
          sendToTNC(loraReceivedFrameString);
        #endif
      }
    #endif
    #ifdef T_BEAM_V1_0
      axp.setChgLEDMode(AXP20X_LED_OFF);
    #endif
  }

  LatShown = String(gps.location.lat(),5);
  LongShown = String(gps.location.lng(),5);
  average_speed[point_avg_speed] = gps.speed.kmph();   // calculate smart beaconing
  ++point_avg_speed;
  if (point_avg_speed>4) {
    point_avg_speed=0;
  }
  average_speed_final = (average_speed[0]+average_speed[1]+average_speed[2]+average_speed[3]+average_speed[4])/5;
  nextTX = (max_time_to_nextTX-min_time_to_nextTX)/(max_speed-min_speed)*(max_speed-average_speed_final)+min_time_to_nextTX;

  if (nextTX < min_time_to_nextTX) {nextTX=min_time_to_nextTX;}
  if (nextTX > max_time_to_nextTX) {nextTX=max_time_to_nextTX;}

  average_course[point_avg_course] = gps.course.deg();   // calculate smart beaconing course

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
    if (new_course < 0) {
        new_course=360+new_course;
      }

    if ((old_course < ANGLE) && (new_course > (360-ANGLE))) {
      if (abs(new_course-old_course-360)>=ANGLE) {
        nextTX = 0;
        // lastTX = min_time_to_nextTX
      }
    } else {
      if ((old_course > (360-ANGLE)) && (new_course < ANGLE)) {
        if (abs(new_course-old_course+360)>=ANGLE) {
          nextTX = 0;
        }
      } else {
        if (abs(new_course-old_course)>=ANGLE) {
          nextTX = 0;
        }
      }
    }
    old_course = new_course;
  }

  if ((millis()<max_time_to_nextTX)&&(lastTX == 0)) {
    nextTX = 0;
  }

  if ( (lastTX+nextTX) <= millis()  ) {
    if (gps.location.age() < 2000) {
      writedisplaytext(" ((TX))","","LAT: "+LatShown,"LON: "+LongShown,"SPD: "+String(gps.speed.kmph(),1)+"  CRS: "+String(gps.course.deg(),1),getSatAndBatInfo(),1);
      sendpacket();
      #ifdef SHOW_GPS_DATA
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
        //digitalWrite(TXLED, LOW);
      #endif
    } else {
      if (millis() > time_to_refresh){
        displayInvalidGPS();
      }
    }
  }else{
    if (millis() > time_to_refresh){
      if (gps.location.age() < 2000) {
        writedisplaytext(" "+Tcall,"Time to TX: "+String(((lastTX+nextTX)-millis())/1000)+"sec","LAT: "+LatShown,"LON: "+LongShown,"SPD: "+String(gps.speed.kmph(),1)+"  CRS: "+String(gps.course.deg(),1),getSatAndBatInfo() ,1);
      } else {
        displayInvalidGPS();
      }
    }
  }
  #if defined(ENABLE_TNC_SELF_TELEMETRY) && defined(KISS_PROTOCOL)
    if (nextTelemetryFrame < millis()){
      nextTelemetryFrame = millis() + TNC_SELF_TELEMETRY_INTERVAL;
      sendTelemetryFrame();
    }
  #endif
  #ifdef KISS_PROTOCOL
    #ifdef KISS_DEBUG
      static auto last_debug_send_time = millis();
      if (millis() - last_debug_send_time > 1000*5) {
        last_debug_send_time = millis();
        String debug_message = "";
        #ifdef T_BEAM_V1_0
          debug_message += "Bat V: " + String(axp.getBattVoltage());
          debug_message += ", ";
          debug_message += "Bat IN A: " + String(axp.getBattChargeCurrent());
          debug_message += ", ";
          debug_message += "Bat OUT A: " + String(axp.getBattDischargeCurrent());
          debug_message += ", ";
          debug_message += "USB Plugged: " + String(axp.isVBUSPlug());
          debug_message += ", ";
          debug_message += "USB V: " + String(axp.getVbusVoltage());
          debug_message += ", ";
          debug_message += "USB A: " + String(axp.getVbusCurrent());
          debug_message += ", ";
          debug_message += "Temp C: " + String(axp.getTemp());
        #else
          debug_message += "Bat V: " + String(BattVolts);
        #endif

        Serial.print(encapsulateKISS(debug_message, CMD_HARDWARE));
        #ifdef ENABLE_BLUETOOTH
          SerialBT.print(encapsulateKISS(debug_message, CMD_HARDWARE));
        #endif
      }
    #endif
  #endif
  vTaskDelay(1);
}
// end of main loop
