# Video Based Multi Channel Position Sensing Camera
## Introduction
This single camera is capable of replacing up to 80 detectors/sensors on a model railroad along with their extensive wiring.
As it is a cheap device, on large railroads, several cameras can be used for adequate coverage. 
It can detect, and be used to control, train movements without any trackside sensors at all.
Used in conjunction with an i2c bus to a DCC-EX Command Station, the options through X-RAIL automation seem unlimited.
It does not include loco or rolling stock identification.

## Hardware
### Base Processor Module
The Camera is a standard ESP32-CAM with an ESP32-CAM-MB (CH-340G based) to add USB capability and, in it's simplest form, a PCA9515A i2c interface module.
To connect the sensorCAM to i2c, some limited soldering is required to connect the PCA9515A (5 wires) to the USB MB on the CAM.
An advanced proto-daughter board is preferable to the PCA9515, to include long i2c buffering (up to 30metres), an isolated power supply, and indicator LEDs.
A cheap commercial aerial may be useful in marginal WiFi situations.

### Camera Module
The sensorCAM uses the 32bit ESP32-S microcontroller with an OV2640 camera module.  
It is fitted with a 4Mbyte PSRAM for image storage and a built in WiFi aerial.
It also has 3Mb of available program storage, EPROM memory for parameters, and 327kbytes of dynamic memory. 

## Software requirements
The Arduino IDE is recommended for the sensorCAM and, if used, the connected EX-Command Station DCC-EX-CS software with EX-RAIL.
The ESP32-CAM has an Arduino library of software that is needed to enable the camera image manipulation and the WiFi imaging of the railroad.
In addition, the Processing 4 application is highly desired to enable railroad images to be captured and sensor positions to be located and seen, as the Arduino IDE cannot give visual feedback.
* https://processing.org/

The webserver WiFi images are not a substitute for the Processing 4 utility, as sensorCAM cannot "sense" in webserver mode.
Specific SensorCAM files are provided for each of the above apps.

## Documentation
The sensorCAM is a complex device needing some in depth understanding to achieve its full potential. A 30 page manual is provided to assist with this process. 

If you want clarification or wish to discuss the implementation please contact me at bcdaniel2@optusnet.com.au and I will attempt to address your issues. 







  
