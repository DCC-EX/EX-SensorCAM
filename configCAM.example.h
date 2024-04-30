/*

 *  © 2023 Barry Daniel
 *  
 *  This file is part of CommandStation-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

/**********************************************************************

The configuration file for ESP32-CAM based sensorCAM 

**********************************************************************/

/////////////////////////////////////////////////////////////////////////////////////
// WIFI_SSID is the network name IF you want to use your existing home network.
// Do NOT change this if you want to use the WiFi in Access Point (AP) mode. 
//
// If you do NOT set the WIFI_SSID and do NOT set the WIFI_PASSWORD,
// then the WiFi chip will first try to connect to the previously
// configured network and if that fails fall back to Access Point mode.
// The SSID of the AP will be automatically set to DCCEX_*.
// If you DO set the WIFI_SSID then the WiFi chip will try to connect
// to that (home) network in station (client) mode. If a WIFI_PASSWORD
// is set (recommended), that password will be used for AP mode.
// The AP mode password must be at least 8 characters long.
//
// Your SSID may not contain ``"'' (double quote, ASCII 0x22).
//#define WIFI_SSID "Your network name"
#define WIFI_SSID    "Your network name"
//
// WIFI_PASSWORD is the network password for your home network or if
// you want to change the password from default AP mode password
// to the AP password you want. 
// Your password may not contain ``"'' (double quote, ASCII 0x22).
//#define WIFI_PWD "Your network passwd"
#define WIFI_PWD "Your network passwd"
//
// For ALTERNATIVE WiFi network set up SSID and password if required
//#define ALTWIFI_SSID     "Your Alt.net name"
//#define ALTWIFI_PWD  "Your Alt.netPassword"
#define ALTWIFI_SSID     "Optus_7CAD47"
#define ALTWIFI_PWD  "stagsyentaXPPjX"
//
// To invoke TWOIMAGE Averaging feature on more/less virtual Sensors, adjust this parameter
// this average may reduce sensitivity to noise BUT may increase response time by 100mSec to minimal level changes 
// use 2 image pixel level average for sensor to filter noise below this bsNo.
//#define TWOIMAGE_MAXBS 030
#define TWOIMAGE_MAXBS 030
//                                                                                
// Set the I2C device address 
// If using a DCCEX CS master, this address must be below the ESP32CAP set in the Command Station "driver EXSensorCAM.h"
// the default CAP is 0x12
//#define I2C_DEV_ADDR 0x11  // default
#define I2C_DEV_ADDR 0x11	 
// 
// As a mains frequency driven lighting system can introduce "flicker" at double the frequency (100/120Hz)
// setting this parameter may reduce the resulting "noise" on sensors
// The software attempts to synchronise the sampling with an integral number of half cycles, i.e the "flicker" frequency
// use 10(mSec)(1 cycle) for 50Hz or 25 (3 cycles) for 60Hz systems.
// #define SUPPLY 10    
#define SUPPLY 10  
//
// Set baud rate for USB port communications
// Very slow response will be observed for serial image downloads at less than 115200 baud 
// however if long unbuffered USB cable is used a slower rate may be unavoudable.
//#define BAUD 115200      // Limits full QVGA Serial image transfer time to 13seconds.      
#define BAUD 115200          
//
// Set Sensor size at 0 for default 4x4 pixel size (16 usable pixels)
// For larger footprint set to # (1-9) for size (4+#)x(4+#)
//#define SEN_SIZE 2       //2 gives 6x6 pixel sensor size (16 usable corner pixels)
#define SEN_SIZE 0         //0 gives standard 4x4 pixels
//
/////////////////////////////////////////////////////////////////////////////////////
