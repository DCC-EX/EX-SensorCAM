#if __has_include ( "configCAM.h")
  #include "configCAM.h"
#else
  #warning configCAM.h not found. Using defaults from configCAM.example.h
  #include "configCAM.example.h"
#endif
#define NUMdigPins   80   //CS can create fewer to save memory.
#define NUManalogPins 0   //must be 4 for UNO emulation
//#define SUPPLY   10     //see configCAM.h - set period to 10 half-cycles of mains (50Hz) (use 25 for 60Hz)
#define CYCLETIME 100000  //100mSec cycle time syncs with 5cycles of 50Hz and 6 cycles at 60 Hz
#ifndef BRIGHTSF
#define BRIGHTSF 3      //increases sensitivity.  If 1, 20% change adds 3*SF to diff. 5% is ignored
#endif
#ifndef SEN_SIZE
#define SEN_SIZE 0      //sensor expansion - add SEN_SIZE rows and columns + through 4x4 sensor. 
#endif
#ifndef STARTUP_DELAY
#define STARTUP_DELAY 2000 
#endif

#define EXIOINIT 0xE0   // CS: CAM config request
#define EXIORDY  0xE1     //response OK & ready for next ioexpander cmd
#define EXIODPUP 0xE2   // CS: pullup settings (ack only)
#define EXIOVER  0xE3   // CS: CAM version request
#define EXIORDAN 0xE4     // CS: requests analogue data array (ignored or send digital array)
#define EXIOWRD  0xE5   // CS: CAM enable(a) or refresh(r) specified sensor (deprecate?)
#define EXIORDD  0xE6   // CS sensor status request
#define EXIOENAN 0xE7   // CS: CAM analogue pin enable (ack only)
#define EXIOINITA 0xE8  // CS: requests analog pin info (ignored)
#define EXIOPINS 0xE9     //response code
#define EXIOWRAN 0xEA   //CS: can use to create a sensor position (r,x)
#define EXIOERR  0xEF     //response - last command erroneous
#define CAMERR   0xFE     //CAM cmd error header
#include "esp_camera.h"   //Define i2c interface parameters--- Any difference between “Wire.h” and <Wire.h> ??

#include <Arduino.h> 
#include <WiFi.h>         //compiler still finding: Multiple libraries were found for "WiFi.h"
            //Used: C:\Users\Barry\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.3-RC1\libraries\WiFi
            //Not used: C:\Program Files (x86)\Arduino\libraries\WiFi
#include <Wire.h>         //esp32 i2c Wire library 
TwoWire MyWire = TwoWire(0);   //Create second i2c interface (don't use Wire1 = ) 

#define I2C_FREQ 100000   //Master sets i2c clock frequency   MEGA uses default clock (100kBits/sec)
                        //Define GPIO ports for I/O use 

//define pins for CAMERA_MODEL
 //set options for ESP32_WROVER_KIT

#ifdef CAMERA_MODEL_WROVER_KIT
#warning WROVER defined
#endif

#ifdef CAMERA_MODEL_WROOM_S3
#warning WROOM_S3 defined
#endif

#ifdef CAMERA_MODEL_AI_THINKER
#warning AI_THINKER defined
#endif

#ifdef CAMERA_MODEL_WROVER_KIT  // define WROVER_KIT or WROOM-S3 in configCAM.h if appropriate 
#define CAMERA_MODEL 2
#define I2C_SDA2 15         //for MyWire (twowire) i2c data GPIO pin 15
#define I2C_SCL2 13         //for MyWire (twowire) i2c clock pin
#define WEBPIN   14         // if grounded on Reset, this pin invokes web server mode - uses default WiFi(1) ssid . 
#define GPIO0     0         // Used as CSI MCLK output, so best to not ground while running UNLESS in reset mode.                    
#define FLASHLED  2         //GPIO2, when 2 HIGH turns on Red LED.  Use ~25mSec pulse to indicate new fb image.
#define BLK0LED  33         //tried GPIO12 - DANGEERUS 12 MUST BE LOW on reset to correctly reset intenals for RAM 
#define BLK1LED  33         // N.B. tried:  GPIO12 and stuffed a CAM -GPIO12 used to set flash programming voltage! 
#define BLK2LED  33         //to use 33 (on-board LED) need to rework CAM to repurpose P_OUT pin(P1-4)(see Youtube)
#define BLK3LED  33         //consider using 4 if can disable flash (remove resistor from CAM?)(but flash useful!)
#define PLED     14         //assign a LED to be programmable. i.e. 'n' cmd assigns a bank status to the PLED
#define QLED     32         //assign a LED to configurable bank NLED output
    
#elif defined( CAMERA_MODEL_WROOM_S3 )     //set camera pin options
#define CAMERA_MODEL 3  
#define I2C_SDA2 47         //for MyWire (twowire) i2c data GPIO pin 47 
#define I2C_SCL2 21         //for MyWire (twowire) i2c clock pin
#define WEBPIN   14         // if grounded on Reset, this pin invokes web server mode - uses default WiFi(1) ssid. 
#define GPIO0     0         // Used as CSI MCLK output, so best to not ground while running UNLESS in reset mode.                    
#define FLASHLED  2         // GPIO2, when 2 HIGH turns on Red LED.  Use ~25mSec pulse to indicate new fb image.
#define BLK0LED  38         // 48 R? need code change to benefit from RGB neopixel on pin 48
#define BLK1LED  38         // 48 Y?   WARNING: pin 38 - MUST NOT HAVE AN SD-CARD IN CAM !
#define BLK2LED  38         // 48 G?
#define BLK3LED  38         // 48 B? Could consider external (9?) neopixels :)
#define PLED     14         //assign a LED to be programmable. i.e. 'n' cmd assigns a bank status to the PLED
#define QLED      1         //assign a LED to configurable bank NLED output 
      // SPARE   38         //38 incompatible with SDcard !
                       
#else   //define CAMERA_MODEL_AI_THINKER as default
#ifndef CAMERA_MODEL_AI_THINKER
#define CAMERA_MODEL_AI_THINKER   // Select camera model for ESP32-CAM (deault: has OV2640 Camera module)
#endif
#define CAMERA_MODEL 1                             
#define I2C_SDA2 15       //for MyWire (twowire) i2c data GPIO pin 15
#define I2C_SCL2 13       //for MyWire (twowire) i2c clock pin
#define WEBPIN  14        // if grounded on Reset, this pin invokes web server mode - uses default WiFi(1) ssid . 
#define GPIO0    0        // Used as CSI MCLK output, so best to not ground while running UNLESS in reset mode.                    
#define FLASHLED 4        //GPIO4, when 4 HIGH turns on Flash LED.  Use ~25uSec pulse to indicate new fb image.
#define BLK0LED 33        //tried GPIO12 - DANGEERUS 12 MUST BE LOW on reset to correctly reset intenals for RAM 
#define BLK1LED 33        // N.B. tried:  GPIO12 and stuffed a CAM -GPIO12 used to set flash programming voltage! 
#define BLK2LED 33        //to use 33 (on-board LED) need to rework CAM to repurpose P_OUT pin(P1-4)(see Youtube)
#define BLK3LED 33        //consider using 4 if can disable flash (remove resistor from CAM?)(but flash useful!)
#define PLED    14        //assign a LED to be programmable. i.e. 'n' cmd assigns a bank status to the PLED
#define QLED    2         //configurable bank NLED output
       //GPIO14 multipurposed - LED pulls GPI14 high. Need to Gnd GPIO14 to select web server.
       //GPIO0 unavailable. Need to Gnd GPIO0 to program then needed for CAM CSI_MCLK
       //GPIO1 & GPIO3 reserved for TX/RX USB comms.
       //GPIO12 best left untouched. Used to set internal voltages for Embedded Flash? 
       //    (tried to use 12, but rendered FLASH RAM unprogrammable!(stuffed #one CAM)
#endif

#include "camera_pins.h"  //for thinker, includes #define XCLK_GPIO_NUM 0

#include <EEPROM.h>
#define EEPROM_SIZE 320+8+80+80+80 //long pointers to image +reboot flag(int)(+threshold,nLED,min2flip,maxSensors?)
#define EPvFlag      EEPROM_SIZE-80-80-80-8 //vFlag stored as (1byte)integer, others as byte
//#define   also stepr/x_SF??
#define EPminSensors EEPROM_SIZE-80-80-80-5 //beware of 0xFF
#define EPnLED       EEPROM_SIZE-80-80-80-4 //NOTE: these cells have garbage(FF?) in unprogrammed CAM - needing init.
#define EPthreshold  EEPROM_SIZE-80-80-80-3
#define EPmin2flip   EEPROM_SIZE-80-80-80-2
#define EPmaxSensors EEPROM_SIZE-80-80-80-1 //use to limit printout of sensor states (if > 40, loop time may suffer)
#define EPSensorTwin EEPROM_SIZE-80-80-80  //save SensorTwin array
#define EPpvtThreshold  EEPROM_SIZE-80-80 //Individual thresholds (use default threshold if =255) also lineardr
#define EPlineardX      EEPROM_SIZE-80   //save xsize for linear sensors
                                        //maxSensors may also be used to avoid other wasted processing time.
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality

#define RGB565p  2                 // bytes per pixel in RGB565 image
#define FBPITCH  (320*RGB565p)    // bytes per row of FB (QVGA=320 pixels)

#define IFsc  if(scroll==true) //output progressive scroll data
#define Printf Serial.printf   //esp32S3 required global change of printf() to implement fast native USB connection
#define Spr  Serial.print     //shortens long repetitive print statement lines.
#define IF0  if(DEBUG[0])     //Higher priority debug
#define IFT  if(DEBUG[1])     //Debug control of timing output
#define IFD  if(DEBUG[2])     //Lower priority Debug control
#define IFI2C if(DEBUG[3])    //output generated i2c packets 
#define IFN  if(DEBUG[4])     //output relating to pixel noise measurements
#define IF5  if(DEBUG[5])     //if bsn>=010 use 4pixel (IR) brightness only //show parz output
#define IF6  if(DEBUG[6])     //grab_ref message printed on full execution of S666init_SensorRefs(prefill) 
#define IF7  if(DEBUG[7])     //stop/wait on a trip of bank # or sensor 16. only if LAST 'h' was h7 or h7#
#define IF8  if(DEBUG[8])     //write EX-CS cmd to USB e.g. EXIODPUP gives "E2", undefined cmds give "#EF" error
#define S666_pitch 48         //length of one sensor rgb666 data of 16 pixels  4x4x3bytes saved in Sensor666[]
#define S666_row  12          //length of row for one sensor (_pitch= s666_row x 4(rows) (for QVGA resolution) 4x3
#define AVCOUNT 32            //Nominate the number of frames to average for 'r' reference image
#define NUM2AVERAGE 32        //variables for auto references averaging & updating for unoccupied enabled sensors
            
#ifndef NLED                  //define NLED in config.h to select a BLK for QLED indicator pin 
#define NLED 0                //default 0 maintains consistency with earlier versions
#endif
#include "esp_partition.h"