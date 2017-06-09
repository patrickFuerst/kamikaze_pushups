##Dependencies:

Arduino Libraries:
  * Adafruit Unified Sensor
  * Adafruit BNO055
  * Wifi101

##Sensor Modules :

Orientation Module:
  * Sensor: [Adafruit BNO055][1]
  * Board Adafruit Feather: Install *Adafruit AVR Boards* and *Arduino SAMD Boards* in Arduino IDE
  * If using Feather M0 Wifi ATWINC150 be sure to add the following in *setup()* before calling anything else:
```c
  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8,7,4,2);
```

[1]: https://github.com/adafruit/Adafruit_BNO055


Be sure all Arduino libraries are in a place where Arduino can find it.
Best way is to copy the *libraries*  folder from *sensors/* to  *~/Arduino*

## Serial Sensor Data Specification

Every message starts with a header character, specifying from which module the recieved data is, followed by the data.


Orientation Module:

* Header character: *"o"* : *char*
* Orientation Data: *x,y,z,w* : *double*
* Accelaration Data: *ax,ay,az* : *double*
* System status: *status* : *uint8*
* Is calibrated: *cal* : *bool*
* Battery Voltage: *voltage* : *float*

Data send using serial is currently send as strings. In the end of each data string theres is newline feed and between each field there is a delimiter (",").
