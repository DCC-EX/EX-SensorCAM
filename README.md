# VideoWebServer_for_CameraCar
## Introduction
I installed a front view camera in a N-Guage train which can transmit the video image over WiFi. Newly released small ESP32 S3 based module "XIAO ESP32 S3 Sense" made it available in N-Guage train which has has a very limited space to install necessary hardware. 
Users are able to view the front view video image from a running train by Web server which runs on the ESP32 S3 processor over WiFi. The system is powered by a 3.7V battery installed in the car. 
In this prototype, the system was installed in a very common box car which is xxx mm length. The car does not have a motor to run. Thus, the car has to be connected in front of an engine. 

## Hardware
### Base Processor Module
Seeed Studio XIAO ESP32 S3 Sense is used for the base processor module. The module has WiFi, OV2540 Camera Interface, Microphone, Battery input with battery charing circuit, USB interface and micro SD Card interface. OV2540 Camera module comes with the product, but was replaced with a longer cable one. Micro SD card interface or Microphone is not used in the Camera Car use at this moment. 

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

### Battery
A 3.7V Lithium-Ion battery is used to supply a power to the modules. As XIAO ESP32 S3 Sense hasa charging circuit to the Lithium-Ion battery, it is possible to charge it by USB power input.
A 350 mAh battery can operate the video transmission for about 15 min.

### USB port
USB port with USB-C connector faces to the rear wall of the box car. It is mainly to install a new sketch to the processor, but it can be used to input power to the processor as well as charge the 3.7V battery. 
In future, I want to supply 5V DC power through the USB port from a track power.

## Installation to a Box Car
### Box Car for the installation
The system was installed in a box car with 98mm length x 19mm wide x 31mm height (including wheels) which is commonly sold.

<img src="https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/3e2e2176-a74a-49ce-851e-d10133da78b1" width="480">

### Wiring
The additioinal wirings made in the system were.
* DC input from 3.7V battery. Wires from battery case soldered to BAT+ and BAT- terinal.
* An external LED connection. Connected to D1 and GND. The LED works as a headlight of the car.

<img src="https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/1a57552a-cf30-4f62-b478-99aca77f1405" width="640">

### OV2640 Camera
Camera itself is installed on the front panel of the box car. The flat cables goes to inside of the box car via an open hole. 
The flat calbes is connected to camera module connector of the XIAO ESPS3 Sense.

### WiFi Antenna
The antenna is not embedded to the processor module. The antenna is installed externally via tiny coax cable. To get the better WiFi connectivity, the antenna was installed on top ot the box car.

### Install XIAO ESP32 S3 Sense
After the necessqry wired were conencted, XIAO ESP32 Sense module placed at the rear side of the box car. The box car is not enough wide to put the module inside. 

<img src="https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/1c16ae6b-a12f-43a2-8eb3-b04fdd8ac742" width="640">

<img src="https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/cd68beb5-a480-4a34-a9d6-671703eff4a2" width="640">

### External view
Side View
To cover the cut on the side of box car, I made a panel which was made from a beer can, and put it on the side of box car body.

<img src="https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/862044c0-b980-42e9-ac90-13991024830d" width="640">

Front View

<img src="https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/314a999d-b404-46b6-92d5-5c1bea4c2ba9" width="640">

Read View

<img src="https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/fe9dbde1-e343-4f5d-9b38-5c507936bcf2" width="640">



## Software
1. First, please install necessary libraries and set-up Arduino IDE following the Software Preparation section of the following tutrial.
* https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/

2. Download sketch
   Download the sketches from the repository here.
   
4. Change WiFi SSID and Password
   
   Change WiFi SSID and Password to the ones in your network.
   
   Two sets of SSID and Password can be configured. They are for the case the camera car is used on own layout or a layout in other place such as model train club.
   The softare switches the SSID automatically from one to another if the connectios is not establishsed in 50 tries.

   If only one SSID configuration is enough, please use same SSID name and passowrd for both.

## Operation
### Basic Operation
The Web Server access uses the video streaming feature by SpeedStudio which is introduced in the tutorial page of XIAO ESP32S3 Sense Camera shown in Base Processor Module section.
The operation is easy. Just access to http://<IP address> via an Web Brower and click "Tuen On Video" on the bottom of the menu.
The camera configuration is available through the bottuns and windows in the menue. They are same as the original example VideoWebServer application by SpeedStudio.
One modification is to add Head Light button on top. It turns on/off the head light LED.

<img width="589" alt="20230920_Model_Train1" src="https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/da16e990-76c9-441a-be3c-2d047deadf7b">

Captured Video Image (Click the video thumbnail below)

The layout itself is still under building.

[![Video on WebServer_for_CameraCar](http://img.youtube.com/vi/2yf1QeSd02I/0.jpg)](https://www.youtube.com/watch?v=2yf1QeSd02I)

### SNMP MIB
The system supports SNMP agent and has following MIB onjects.
The default access password is public for SNMP GET and private for SNMP SET. They are mainly for a trouble shooting in case the video streaming does not come smoothly by checking WiFi signal strength and video frame rate.
OID = .1.3.6.1.4.1.4998.3.1.1, ReadWrire, It reports SSID ID which is in use.

OID = .1.3.6.1.4.1.4998.3.1.2, ReadOnly, It reports SSID Name which is in use. 

OID = .1.3.6.1.4.1.4998.3.1.3, ReadOnly, It reports WiFi signal stlength. 

OID = .1.3.6.1.4.1.4998.3.1.4, ReadOnly, It reports video frame rate in 1000th of the rate. 10000 representes 10 frames per sec.

OID = .1.3.6.1.4.1.4998.3.1.5, ReadWrire, It reports the status of head light LED, true (on) or false (off).


Eaxmple output.

~ % snmpwalk -c public -v 2c 192.168.1.38 1.3.6.1.4.1.4998.3.1

SNMPv2-SMI::enterprises.4998.3.1.1 = INTEGER: 0

SNMPv2-SMI::enterprises.4998.3.1.2 = STRING: "ssid1"

SNMPv2-SMI::enterprises.4998.3.1.3 = INTEGER: -55

SNMPv2-SMI::enterprises.4998.3.1.4 = INTEGER: 10416

SNMPv2-SMI::enterprises.4998.3.1.5 = INTEGER: 0

SNMPv2-SMI::enterprises.4998.3.1.5 = No more variables left in this MIB View (It is past the end of the MIB tree)


### Serial Console Output
There is serial console output for debugging purpose. If the system is faling to connect WiFi, the serial console output reports the status of WiFi connection.


