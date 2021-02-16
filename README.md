# LoRa tracker with KISS TNC capability

Tracker can be used on its own. 
You can also connect it via blueetooth with APRSdroid.
After connection with APRX based DIGI it can be used as KISS-TNC

![diagram](https://github.com/SQ9MDD/TTGO-T-Beam-LoRa-APRS/blob/master/img/digi-schemat.png)

## Contributors
* Initial work: OE1ACM, OE3CJB<br>
* Redesigned: SQ9MDD<br>
* KISS TNC Over Seriall or Bluetooth: SQ5RWU<br><br>

## Supported boards
* TTGO T-beam v.0.7
* TTGO T-beam v.1.0 


## User key functions:
### short press:
* if there is a GPS-FIX immediate sending of a frame with the position from the GPS
* if there is no GPS-FIX, immediate sending of the frame with the position saved in the configuration

### long press: on or off the GPS power supply 
* if the "FIXED_BEACON_EN" option is enabled in the configuration, a beacon will be sent every set time interval

## Oled screens information
* ((TX)) - position frame sent automatically
* ((RX)) - informs about the received frame 
* ((GPSOFF)) - GPS power-off information
* ((GPS ON)) - GPS power-on information
* ((MAN TX)) - information about sending a manually initialized GPS position frame
* ((FIX TX)) - information about the forced manual sending of a frame with the position saved in the configuration when GPS is off or no fix
* ((AUT TX)) - information about sending automatic positioning frame when GPS is turned off
* ((KISSTX)) - information about sending the frame sent by KISS
