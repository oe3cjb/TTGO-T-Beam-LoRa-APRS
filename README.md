First of all I want to thank OE1ACM Bernd for his approval to use his code for my experiments.<br>
He is the author of BG_RF95!<br>
<br>
If you want to discuss with other LoRa APRS interested persons join the growing community at the LoRa-APRS Telegram group<br>
<br>
<b>USER GUIDE of TTGO T-Beam LoRa APRS V1.1:</b><br>
<b>Attention: a new HW Version is available - if you use the old version uncomment "// #define T_BEAM_V0_7" and comment out "#define T_BEAM_V1_0".</b>You can recognize the new Rev1.0-Boards at their three push buttons instead of two at the older boards - both versions are now supported.<br>
As the new board only has two LEDs for the GPS and battery charger I've moved the TX Led to PIN 33 - please use a LED with reasonable resistor of 470R.<br>
<br>
<b>Callsign setting:</b><br>
Now two possibilities are implemented:<br>
1st) enter your callsign in the file TTGO_T-Beam_LoRa_APRS_<b>config</b>.h and replace the OE1XYZ-0 with your data with 6 characters plus SSID-<br>
in case of shorter callsign use "SPACES" and extend up to total length of 6 characters<br>
2nd) if you leave the config-file unchanged, you will run into a simple setup routine at the very first boot and you will be asked to enter your callsign and SSID(s).<br>
Once the callsign is programmed you can reenter the programming mode by pressing the BUTTON for 3secs while switching on the tracker.<br>
<br>
The MODE of the tracker can now be changed by pressing the button 10secs!<br>
<b>The modes are</b><br>
TRACKER ...     normal APRS tracker<br>
WX&TRACKER ...  alternate transmission of normal position packet and WX packet (if DHT22 is mounted)<br>
WX-MOVE ...     only WX packets are sent but with position from GPSLED<br>
WX-FIXED ...    only WX packets are transmitted but with fixed position given in Header-File<br>
<br>
The <b>fixed position</b> is used for a fixed weather station, e.g. without GPS signal.<br>
<br>
Possible symbols are<br>
Weather Station, Car, Runner, Bicyle, Motorcycle<br>
<br>
for DHT22 I used the library from https://github.com/beegee-tokyo/DHTesp, as the standard library gives to many wrong readings<br>
<br>
new features:<br>
- preset of callsign and SSID in file TTGO_T-Beam_LoRa_APRS_<b>config</b>.h --- this is the <b>only</b> you should change - if you are not familiar with programming ;-)<br>
- corrected format of speed, course and height to be shown correctly in aprs.fi and aprsdirect.com
- Smart Beaconing - the maximum period can be set in the config-file - the minimum period is limited to 60sec, the calculation is based on speeds between 0 and 50 km/h, the default is smart beaconing is off with a minimium period setting of 60secs<br>
- usage of shorter callsigns is now also possible - fill up with SPACES up to 6 characters please<br>
- support of new power management chip AXP192<br>
