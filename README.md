First of all I want to thank OE1ACM Bernd for his approval to use his code for my experiments.<br>
He is the author of BG_RF95!<br>
<br>
If you want to discuss with other LoRa APRS interested persons join the growing community at the LoRa-APRS Telegram group<br>
<br>
<b>USER GUIDE of TTGO T-Beam LoRa APRS V1.2:</b><br>
<b>Attention: a new HW Version is available - if you use the old version uncomment "// #define T_BEAM_V0_7" and comment out "#define T_BEAM_V1_0".</b>You can recognize the new Rev1.0-Boards at their three push buttons instead of two at the older boards - both versions are now supported.<br>
As the new board only has two LEDs for the GPS and battery charger I've moved the TX Led to PIN 33 - please use a LED with reasonable resistor of 470R.<br>
<br>
You can now do an immediate TX beacon if you press the key for 2secs - use this for testing purposes.<br>
<br>
<b>Callsign setting:</b><br>
Now two possibilities are implemented:<br>
1st) enter your callsign in the file TTGO_T-Beam_LoRa_APRS_<b>config</b>.h and replace the OE1XYZ-0 with your data with 6 characters plus SSID-<br>
in case of shorter callsign use "SPACES" and extend up to total length of 6 characters<br>
2nd) if you leave the config-file unchanged, you will run into a simple setup routine at the very first boot and you will be asked to enter your callsign and SSID(s).<br>
Once the callsign is programmed you can reenter the programming mode by pressing the BUTTON for 5secs while switching on the tracker.<br>
<br>
The MODE of the tracker can now be changed by pressing the button 10secs!<br>
This can be done now without connected display but with a mounted TX LED.<br>
<b>The modes are</b><br>
TRACKER ...     LED blinks 1x - normal APRS tracker<br>
WX&TRACKER ...  LED blinks 2x - alternate transmission of normal position packet and WX packet (if DHT22 is mounted)<br>
WX-MOVE ...     LED blinks 3x - only WX packets are sent but with position from GPS<br>
WX-FIXED ...    LED blinks 4x - only WX packets are transmitted but with fixed position given in Header-File<br>
<br>
The <b>fixed position</b> is used for a fixed weather station, e.g. without GPS signal.<br>
<br>
Possible symbols are<br>
Weather Station (1x blink), Car (2x blink), Runner (3x blink), Bicyle (4x blink), Motorcycle (5x blink)<br>
The symbol can now be changed without attached display - during normal operation press the key for 3secs and you will enter the setup routine. The first value to set is now the symbol and the currently selected symbol is represented by blinks of the TX LED. Once the one needed blinks, just press the key for short moment. After that you will be asked if you want to continue the setup routine (which only makes sense with a connected display) or if you want to stop and so the new symbol will be stored in the NVS memory.<br>
<b>2x TX LED blinks represent "yes" to leave the setup</b> - press here the key to leave the setup - please do so if you don't have a display attached.<br>
1x TX LED blinks represent "no" to continue with the setup - press here the key to continue the setup - please do so if you have a display attached.<br>
<br>
<b>Temperature Sensor:</b><br>
for DHT22 I used the library from https://github.com/beegee-tokyo/DHTesp, as the standard library gives to many wrong readings<br>
Now the DS18B20 is supported as well - uncomment line 31: // #define DS18B20    // use this if you use DS18B20, default ist DHT22<br>
<br>
<b>show RX packets</b><br>
by uncommenting <b>// #define SHOW_RX_PACKET</b> the tracker shows received LoRa APRS packets in raw format for the time in milliseconds defined in SHOW_RX_TIME - both in ...config.h<br>
<br>
<b>new features:<b><br>
- show RX packets
- DS18B20 support (setable in config.h)
- GPS switched off in WX_FIXED mode (only available with boards with HW-Version >=V1.0)
- immediate TX with short key press
- course changes of >30° will cause a TX beacon
- code optimized and cleaned
- preset of callsign and SSID in file TTGO_T-Beam_LoRa_APRS_<b>config</b>.h --- this is the <b>only</b> you should change - if you are not familiar with programming ;-)<br>
- corrected format of speed, course and height to be shown correctly in aprs.fi and aprsdirect.com
- Smart Beaconing - the maximum period can be set in the config-file - the minimum period is limited to 60sec, the calculation is based on speeds between 0 and 50 km/h, the default is smart beaconing is off with a minimium period setting of 60secs<br>
- usage of shorter callsigns is now also possible - fill up with SPACES up to 6 characters please<br>
- support of new power management chip AXP192<br>
