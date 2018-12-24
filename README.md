First of all I want to thank OE1ACM Bernd for his approval to use his code for my experiments.
He is the author of BG_RF95!

<b>USER GUIDE of TTGO T-Beam LoRa APRS:</b><br>
<b>Callsign setting:</b><br>
If you start the FW for the first time you are asked to program your callsign and SSID(s).<br>
Once the callsign is programmed you can reenter the programming mode by pressing the BUTTON (GPIO39) for 3secs while switching on the tracker.<br>

The MODE, fiex position and Symbol of the tracker can now be changed by pressing button at <b>GPIO39 for 10secs</b>!<br>
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


for DHT22 I used the library from https://github.com/beegee-tokyo/DHTesp, as the standard library gives to many wrong readings
