# LoRA tracker with KISS TNC capability

Tracker can be used on its own. 
You can also connect it via blueetooth with APRSdroid.
After connection with APRX based DIGI it can be used as KISS-TNC

![TTGO scenarios](pics/digi-schemat.png)

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


R.
