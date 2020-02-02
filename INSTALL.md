<h2>Installation Guide using PlatformIO</h2>
<br>
1. Clone all files of the respository to your local working directory<br>
2. Install the missing libraries<br>
There are two possibilities - either using the Library Manager of PlatformIO or the command line tool:<br>
<h3>1. Built-In Liabrary Manager</h3>
Press the PlatformIO HOME Button to enter the Home Screen and there the Libraries Button to add missing libraries:<br>
<img src="img/img1.jpg" width="480"><br>
Search and install the following libaries:<br>
<ul>
<li>RadioHead</li>
<li>TinyGPSPlus</li>
<li>DHT sensor library for ESPx</li>
<li>Adafruit SSD1306</li>
<li>Adafruit GFX Library</li>
<li>Adafruit Unified Sensor</li>
<li>AXP202X_Library</li>
<li>OneWire</li>
<li>DallasTemperature</li>
</ul>
<br>
<h3>2. Command Line Tool</h3>
use the following commands<br>
platformio lib install "RadioHead"<br>
platformio lib install "TinyGPSPlus"<br>
platformio lib install "DHT sensor library for ESPx"<br>
platformio lib install "Adafruit SSD1306"<br>
platformio lib install "Adafruit GFX Library"<br>
platformio lib install "AXP202X_Library"<br>
platformio lib install "Adafruit Unified Sensor"<br>
platformio lib install "OneWire"<br>
platformio lib install "DallasTemperature"<br>
<br>
Check that the platformio.ini is available as it holds the board type for PlatformIO.<br>
After pressing the check mark the code will be compiled, after pressing the arrow it will be compiled and uploaded to a connected TTGO.<br>
