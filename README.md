# VideoWebServer_for_CameraCar
## Introduction
I installed a front view camera in a N-Guage train which can transmit the video image over WiFi. Newly released ESP32 S3 based module made it available in N-Guage train which has has a very limited space to install necessary hardware. 
Users are able to view the front view video image from a running train by Web server which runs on the ESP32 S3 processor over WiFi. The system is powered by a 3.7V battery installed in the car. 
In this prototype, the system was installed in a very common box car which is xxx mm length. The car does not have a motor to run. Thus, the car has to be connected in front of an engine. 

## Hardware
### Base Processor Module
Seeed Studio XIAO ESP32 S3 Sense is used for the base processor module The module has WiFi, OV2540 Camera Interface, Microphone, Battery input with battery charing circuit, USB-C interface and SD Card interface. OV2540 Camera module comes with the product, but was replaced with a longer cable one. SD card interface or Microphone is not used in the Camera Car use at this moment. 

XIAO ESP32 S3 Sense can be obtained form Amazon.
https://www.amazon.com/gp/product/B0C69FFVHH/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1

Please refer the following URL for the general usage of XIAO ESP32S3 Sense Camera.
https://github.com/limengdu/SeeedStudio-XIAO-ESP32S3-Sense-camera

### Camera Module
OV2640 camera module with 120 mm wire is used raplacing the OV2640 camera module attached in XIAO ESP32 S3 Sense. It is to install a camera on front of the box car and XIAO ESP32 S3 Sense　module on the other side of car.

The OV2640 module I used was obtained from form Amazon here.
https://www.amazon.com/gp/product/B08XLWLGG6/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1

### Wiring

## Installation to a Box Car
### Box Car for the installation

### Install XIAO ESP32 S3 Sense

### OV2640 Camera

### WiFi Antenna
The antenna is not embedded to the processor module. The antenna is installed externally via tiny coax cable. To get the better WiFi connectivity, the antenna was installed on top ot the box car.

### External view

![20230618_Moodel_Train9](https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/2313e41b-c91b-4342-9147-577760250168)

<img src="https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/2313e41b-c91b-4342-9147-577760250168" width="480">

![20230618_Moodel_Train7](https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/862044c0-b980-42e9-ac90-13991024830d)

<img src="https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/862044c0-b980-42e9-ac90-13991024830d" width="480">

![20230618_Moodel_Train10](https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/26b64cba-4164-4e61-a883-defcb8490ccc)

<img src="https://github.com/ktomoma/VideoWebServer_for_CameraCar/assets/131932595/26b64cba-4164-4e61-a883-defcb8490ccc" width="480">


## Software

