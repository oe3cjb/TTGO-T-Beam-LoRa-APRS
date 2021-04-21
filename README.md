# LoRa tracker with KISS TNC capability

Tracker can be used on its own. 
You can also connect it via blueetooth with APRSdroid.
After connection with APRX based DIGI it can be used as KISS-TNC

![diagram](https://github.com/SQ9MDD/TTGO-T-Beam-LoRa-APRS/blob/master/img/digi-schemat.png)

## Contributors
* Initial work: OE1ACM, OE3CJB
* Redesigned: SQ9MDD
* KISS TNC Over Seriall or Bluetooth: SQ5RWU
* Lora32 board support DJ1AN

## Supported boards
* TTGO T-beam v.0.7
* TTGO T-beam v.1.0 
* Lora32 board


## User key functions:
### short press:
* if there is a GPS-FIX immediate sending of a frame with the position from the GPS
* if there is no GPS-FIX, immediate sending of the frame with the position saved in the configuration

### long press On boot 
* reset to factory default

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

## How to binary first flash readme... (thanx SP6VWX)
* Download the appropriate binary file for your board from;
* https://github.com/SQ9MDD/TTGO-T-Beam-LoRa-APRS/releases
* Download current version of the esphome-flasher tool
* https://github.com/esphome/esphome-flasher/releases
* Connect your board to your USB port and open ESPHome Flasher.
* If your board is not showing under Serial Port then you likely need to install the drivers for the CP210X serial chip. In Windows you can check by searching “Device Manager” and ensuring the device is shown under “Ports”.
* In ESPHome Flasher, refresh the serial ports and select your board's serial port.
* Browse to the downloaded firmware and select the correct firmware based on the board type.
* Select/Press Flash ESP.
* Once complete, “Done! Flashing is complete!” will be shown.
* any subsequent updates can be done via WWW
