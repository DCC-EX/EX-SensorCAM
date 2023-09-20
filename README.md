# VideoWebServer_for_CameraCar
## Introduction
I installed a front view camera in a N-Guage train which can transmit the video image over WiFi. Newly released small ESP32 S3 based module "XIAO ESP32 S3 Sense" made it available in N-Guage train which has has a very limited space to install necessary hardware. 
Users are able to view the front view video image from a running train by Web server which runs on the ESP32 S3 processor over WiFi. The system is powered by a 3.7V battery installed in the car. 
In this prototype, the system was installed in a very common box car which is xxx mm length. The car does not have a motor to run. Thus, the car has to be connected in front of an engine. 

## Hardware
### Base Processor Module
Seeed Studio XIAO ESP32 S3 Sense is used for the base processor module The module has WiFi, OV2540 Camera Interface, Microphone, Battery input with battery charing circuit, USB interface and micro SD Card interface. OV2540 Camera module comes with the product, but was replaced with a longer cable one. Micro SD card interface or Microphone is not used in the Camera Car use at this moment. 

XIAO ESP32 S3 Sense can be obtained form Amazon.
* https://www.amazon.com/gp/product/B0C69FFVHH/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1

Product introduction by the maker is seen here.
* https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html

Please refer the following tutrial page for the general usage of XIAO ESP32S3 Sense Camera.
* https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html
* https://github.com/limengdu/SeeedStudio-XIAO-ESP32S3-Sense-camera

### Camera Module
OV2640 camera module with 120 mm wire is used raplacing the OV2640 camera module attached in XIAO ESP32 S3 Sense. It is to install a camera on the front of the box car and XIAO ESP32 S3 Sense module on the other side of car so that USB-C connector is exposed to the back side of the box car.

The OV2640 module I used was obtained from form Amazon here.
* https://www.amazon.com/gp/product/B08XLWLGG6/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1

### Wiring
The additioinal wirings made in the system were.
* DC input from 3.7V battery
* An external LED connection. The LED works as a headlight of the car.

### Battery
A 3.7V Lithium-Ion battery is used to supply a power to the modules. As XIAO ESP32 S3 Sense hasa charging circuit to the Lithium-Ion battery, it is possible to charge it by USB power input.
A 350 mAh battery can operate the video transmission for about 15 min.

### USB port
USB port with USB-C connector faces to the rear wall of the box car. It is mainly to install a new sketch to the processor, but it can be used to input power to the processor as well as charge the 3.7V battery. 
In future, I want to supply 5V DC power through the USB port from a track power.

## Installation to a Box Car
### Box Car for the installation

### Install XIAO ESP32 S3 Sense

### OV2640 Camera

### WiFi Antenna
The antenna is not embedded to the processor module. The antenna is installed externally via tiny coax cable. To get the better WiFi connectivity, the antenna was installed on top ot the box car.

### External view

Front View
<img src="https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/314a999d-b404-46b6-92d5-5c1bea4c2ba9" width="640">

Read View
<img src="https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/fe9dbde1-e343-4f5d-9b38-5c507936bcf2" width="640">

Side View
<img src="https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/862044c0-b980-42e9-ac90-13991024830d" width="640">


## Software
1. First, please install necessary libraries and set-up Arduino IDE following the Software Preparation section of the following tutrial.
* https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/

2. Download sketch
   Download the sketches from the repository here.
   
4. Change WiFi SSID and Password
   Change WiFi SSID and Password to the ones in your network.

## Operation
### Basic Operation
The Web Server access uses the video streaming feature by SpeedStudio which is introduced in the tutorial page of XIAO ESP32S3 Sense Camera shown in Base Processor Module section.
The operation is easy. Just access to http://<IP address> via an Web Brower and click "Tuen On Video" on the bottom of the menu.
The camera configuration is available through the bottuns and windows in the menue. They are same as the original example VideoWebServer application by SpeedStudio.
One modification is to add Head Light button on top. It turns on/off the head light LED.

### SNMP MIB
The system supports SNMP agent and has following MIB onjects.
The default access password is public for SNMP GET and private for SNMP SET. They are mainly for a trouble shooting in case the video streaming does not come smoothly by checking WiFi signal strength and video frame rate.

### Serial Console Output
There is serial console output for debugging purpose. If the system is faling to connect WiFi, the serial console output reports the status of WiFi connection.


