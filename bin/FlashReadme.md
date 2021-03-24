How to binary flash readme...
Download current version of the esphome-flasher tool
https://github.com/SQ9MDD/TTGO-T-Beam-LoRa-APRS/blob/master/bin/FlashReadme.md
Connect your board to your USB port and open ESPHome Flasher.
If your board is not showing under Serial Port then you likely need to install the drivers for the CP210X serial chip. In Windows you can check by searching “Device Manager” and ensuring the device is shown under “Ports”.
If there is an error, instal the drivers and make sure your board is present in Device Manager as COMxxx port.
In ESPHome Flasher, refresh the serial ports and select your board's serial port.
Browse to the downloaded firmware and select the correct firmware based on the board type.
Select Flash ESP.
Once complete, “Done! Flashing is complete!” will be shown.
