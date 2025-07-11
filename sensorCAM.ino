//sensorCAM Alpha release                                                                                 limit >|
#define BCDver 318   
/*  //318  now also Accepts CS style sensorCAM cmds (<N  >), and more spaces in commands, from CAM USB monitor
    //v317c set minSensors to skip scroll status & (working) sensors below minSensors, unless set to default 0
    //v317 block LED handling re. latched & shared sensors ; convert EPvFlag to byte; enhanced EPminSensors
    //v316 added ESP32_WROVER_CAM option (integral USB); made QLED configurable via NLED in config.h; fix <NM>
    //v315 fix to '&' parzing, make 'v 0' from CS equivalent to '^'
    //v314 fix 'v1' command bug introduced v310
    //v313 corrected a line bug (dx) and suppressed some debug output
    //v312 introduce int8_t deltaX[80] to allow finer line angles. dx -64 to 63.  Saves to EPROM. Remved _SFs
    //v311 improved setup for line sensor '\%%'  Add linear markers to image 'Y' & correct f565to666() ptr
    //v310 added parser and modified processCmd() Remove some redundancies
    //v308 add tcounter for 't' argument of 2-30. Linear STEPX_SF & STEPR_SF. Only 'a%%,r,x' clears pvtThreshold
    //v307 getting line sensors cmd '/' and '\' working optimally.  'v' for version, 'v1' or 'v2' for webcam
    //v306 pvtThreshold for S00 & linear banks. 'a' now clears pvtThreshold. Create linear '/' & '\' cmds. 
    //v305 more minor code neatening & array initialisation. Correct Linear stepR calculation
    //v304 move globals to static functions where possible
    //v303 start localising variables. check r00 malfunction & adjust "i2c cmd:" display
    //v302 add '+' alignment, & experiments with wire.write() strategies 
    //v301 adjust i2c 'p' & 'q' cmd. added Prof's startup delay(2000)
    //v300 working mods to deal with new IO_EXSensorCAM.h driver(v300)
*/    
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

     // define appropriate board (only AI_THINKER tested so far
#ifndef CAMERA_MODEL_WROVER_KIT   // define in configCAM.h if appropriate 
#define CAMERA_MODEL_AI_THINKER   // Select camera model for ESP32-CAM (deault: has OV2640 Camera module)
#endif
#include "camera_pins.h"          //for thinker, includes #define XCLK_GPIO_NUM 0

#define I2C_FREQ 100000   //Master sets i2c clock frequency   MEGA uses default clock (100kBits/sec)
                        //Define GPIO ports for I/O use 
#ifdef  CAMERA_MODEL_AI_THINKER                             
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
       //    (tried to use, but rendered FLASH RAM unprogrammable!(stuffed one CAM)
#else  //set options for ESP32_WROVER_KIT
#define I2C_SDA2 15       //for MyWire (twowire) i2c data GPIO pin 15
#define I2C_SCL2 13       //for MyWire (twowire) i2c clock pin
#define WEBPIN  14        // if grounded on Reset, this pin invokes web server mode - uses default WiFi(1) ssid . 
#define GPIO0    0        // Used as CSI MCLK output, so best to not ground while running UNLESS in reset mode.                    
#define FLASHLED 2        //GPIO2, when 2 HIGH turns on Red LED.  Use ~25mSec pulse to indicate new fb image.
#define BLK0LED 33        //tried GPIO12 - DANGEERUS 12 MUST BE LOW on reset to correctly reset intenals for RAM 
#define BLK1LED 33        // N.B. tried:  GPIO12 and stuffed a CAM -GPIO12 used to set flash programming voltage! 
#define BLK2LED 33        //to use 33 (on-board LED) need to rework CAM to repurpose P_OUT pin(P1-4)(see Youtube)
#define BLK3LED 33        //consider using 4 if can disable flash (remove resistor from CAM?)(but flash useful!)
#define PLED    14        //assign a LED to be programmable. i.e. 'n' cmd assigns a bank status to the PLED
#define QLED    32        //assign a LED to configurable bank NLED output
#endif
#include <EEPROM.h>
#define EEPROM_SIZE 320+8+80+80+80 //long pointers to image +reboot flag(int)(+threshold,nLED,min2flip,maxSensors?)
#define EPvFlag   EEPROM_SIZE-80-80-8   //VFlag stored as (1byte)integer, others as byte
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

#define RGB565p  2                // bytes per pixel in RGB565 image
#define FBPITCH  (320*RGB565p)    // bytes per row of FB (QVGA=320 pixels)

bool    DEBUG[11]={false,false,false,false,false,false,false,false,false,false}; //show detail debug to USB output

#define IFsc  if(scroll==true) //output progressive scroll data
#define Spr  Serial.print     //shortens long print statements.
#define IF0  if(DEBUG[0])     //Higher priority debug
#define IFT  if(DEBUG[1])     //Debug control of timing output
#define IFD  if(DEBUG[2])     //Lower priority Debug control
#define IFI2C if(DEBUG[3])    //output generated i2c packets 
#define IFN  if(DEBUG[4])     //output relating to pixel noise measurements
#define IF5  if(DEBUG[5])     //show parz output
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


/*  sensorCAM commands:
a%%   Activate/enAble sensor[%%] refresh Sensor_ref[%%], cRatios etc. from S[%%] image (48 bytes) in latest frame.
a%%,rrr,xxx  *Set coordinates of Sensor[%%] to row/col: rrr/xxx AND activate and auto ref.   Verify with p$ cmd.    
b%[#]   bank % sensors. Show which sensors OCCUPIED(1) (in bits 7-0) [brightSF=# ('h'shows #)] <Request $+1 bytes>  
c$$$$  *reCalibrate camera CCD occasionally and grab new references for all enabled sensors. (Beware of doing this
        while any sensors are occupied!  Obstructed sensors will later need an r%%   Check all bank LEDS are off 
        AND check all sensors are unoccupied before recalibrate.  Can set BRI CON SAT AWB through c$$$$ e.g. c0120
        Can change default setting for AWBg,AEC,AECd,AEL,AGC,AGg with six extras: c$$$$1111119. c$$$$ resets them!
d%%[#] *Difference score in colour & brightness between Ref  & actual image. Show # grabs.  <Request 4 data bytes>
e    *EPROM save of any new Sensor offset positions & 4 parameters.    Warning: May only work once per reset
f%%  *Frame buffer sample display. Prints latest binary image of SensorRef[%%] & Sensor[%%]  <Requests 4x28 bytes>
g    *Get Camera Status. Displays most current settings available in webcam window.  (also works in video mode)
h$    Help(debug)output -  h to turn OFF, h0 turns ON detailed USB output. h1:more; h2:timing; h3:i2c; h4:Noise
i%%  *Individual sensor %% Information. Optional i%%,$$ sets Twin sensor[$$] for S%%.  <i2c Request 10 data bytes>
j$#   adJust camera setting $ to value # and display most settings (as for ‘g’) ‘j’ does NOT get new refs. - use r  
k%%,rrr,xxx    *Set coordinates of Sensor[%%] to row: rrr &  xVal: xxx. Follow with r%%. Verify values with p$ cmd 
l%%   (Lima) force sensor %% to ON (1= occupied (LED lit) & also set SensorActive[%%] false to inactivate updating
m$[,%%] *Min. no.($) of sequential frames to trigger Occupied threshold for detection (def.=2)[%% sets maxSensors]    
n$[,%%] *Number of bank($) assigned to the programmable nLED to show bank$ occupancy status.  [%% sets minSensors]
o%%   (Oscar)force sensor %% off (0=UN-occupied (LED off) & also set SensorActive[%%] false to inactivate updating
p$   *Position Pointer table for banks 0 to $ giving DEFINED sensor r/x positions.  p%% shorter.  <32 data bytes>
q$   *Query bank $, to show which sensors enabled (in bits 7-0). 1=enabled. q9 gives ALL    <Req. $+2 data bytes>
r%%[*]  Refresh average Sensor_Ref[%%] (Iff defined), enable & calc. cRatios etc. [r%%* refreshes bank up to S%%] 
r00   Renew Average Refs etc. for ALL defined sensors.  Ignores active[].  Sensor 00 reserved for brightness ref.
s%%  *Scan for new location for sensor %% (00-97). If found, records location in Sensor[%%]. Further setup needed. 
        Scan looks for a bright LED on a dimmer background.  The LED should be placed on the desired sensor pos'n.  
        If satisfied with the scan, the user should REMOVE the LED, set lighting, and do an r%% to set enabled AND 
        record a new reference image (also computes colour ratios & brightness) The location must be unoccupied!
t##[,%%] Threshold level being used. show/set for Sensors (## sets 32-98)[for S%%] t1 toggle scrolling  <32 bytes>
u%%  *Un-define/remove sensor %% by setting INACTIVE & Sensor[%%]=0. u99 erases ALL. Needs ‘e’ to erase from EPROM
v[1|2] *Video mode. Reboot CAM in webserver mode. [v2” will connect to 2nd (alt.) router ssid] ('v' gives version)
w    *Wait for new command line (\n) before resuming loop().   (handy to stop display data scrolling)
x###  Presets column for start of Processing image transfer (0-318)
y###  Suspend imaging. Proceeds to write header & Zlength row### pixels to USB port for Processing4. 'yy' resumes
yy    Used exclusively to end a series of 'y' commands (that suspended imaging) and returns CAM to run mode.
z###  Presets Zlength for line length (even number of pixels) in image transfer (2-320) (see 'y')
F|R  *Reset commands – will Reset CAM and initiate the Sensor mode.  Both will Finish the WebServer (v) mode.
\%%,$,$ *Line sensor S%% uses a 4 step line with $,$(r,x) steps AND extend using rest of bank sensors.('/' for -x)
&    *Diagnostic output of statistics. Gives histogram of No. of frames of ("noise-tripped") occupations.
@##   Changes 'Occupied' indicator from 35('#') to any nominated ASCII char. from 1 to 127. @12 for BOLD 'spade'
+#,$ *Add to position - camera alignment adjustment. Move ALL sensors #(0-3) pixels in $ direction(0-7)(North-NW)  
     *  These commands typically for diagnostic/setup use only.  They wait for a line feed or command to resume.
*/
unsigned long int starttime=0;          //counter giving each start time in uSec.
int E6counter=200;  //E6 EXIOexpander cmd occurs at 100/sec.  stop E4 debug output after #

byte analoguePinMap[] = {12,13,14,15,16,17};       //default pin map for uno & nano  vpin=firstVpin+Map[i]?
byte EXCScmd0=0;      //last EXCS cmd
bool EXCScmd=false;   //true if last i2c cmd from EX
int firstVpin=0;      //to be set by EX-CS EXIOINIT function
int Bri=0, Con=1, Sat=2, AWB=1, AWBg=1, AEC=1, AECd=1, AEL=1, AGC=1, AGg=9;   //initial settings of 'c' variables
int AWB9=1, AGC9=1;                         //after prefillRef delay (9sec) change AWB and AGC if desired
//   arguments for 'c' command; Bri Con Sat AWB AWBg AEC AECd AEL AGC AGg (NO SPACES!)
int dbug=10;
char OCc = '#';       //OCc character to indicate OCCUPIED in scrolling status line. @12 good for Arduino IDE

int emptyStateCtr[80];        //used to count No. of loops a sensor NOISE caused "OCCUPIED" status  
int rbsn = 1;                 //bsn for current averaging If unoccupied, new 3.2sec ref regularly(~EVERY MIN.?)
bool autoRefSuspended = true; //use to stop auto referencing during conflicting commands e.g. 'r' and 'c' ("SUS")
int  force_refs=0;              //used to force block refs

// byte RingBuff[2*80*16*3];     //ring buffer to save grabs to average before trip
bool avOddFrame=true;         //for av2Frames()
bool boxSensors=false;
int  SensorHiCount[80];       //for '&' noise histogram command
int  HistoLoops=0;
int  SensorHisto[80*5];
//uint32_t i2cCtr = 0;        //prepare for i2c       
//bool CmdI2C=false;
int  nLED=2;                  //Blk No assigned to programmable LED (PLED)- initially bank 2
 
int  maxSensors=050;          //use to limit USB PRINTOUT time & line length 
unsigned int  minSensors=000;           //a lower limit for sensor data output
    //used if using more sensors than can otherwise be processed in 100mSec.(42-> max bsNo=51 (5*8+1) ). It limits
    //monitor output (if more than 40, sensors loop time may suffer because of printout time to monitor)
unsigned long Sensor[80];     //from EEPROM array of xy coordinates for 10 banks of 8 sensors.   
    //Sensor[] Offsets into FULL image buffer!  xy stored in 4 bytes EPROM
byte Sensor666[80*16*3];      //buffer to hold decoded latest 4x4 sensor images from frame fb (pitch=S666_row=12)
byte Sensor_ref[80*4*4*3];    //array of 80 reference grabs (QVGA: 4x4x3RGB =48 bytes/sensor (total 3840 bytes 
unsigned int SensorRefRatio[80*12];        //array of 3 x 4 quadrant colour ratios (r/g g/b b/r)x4
int  Sen_Brightness_Ref[80];  //sum of pixels in all quadrant colours combined. max=3024 (4*4*3*63) for 666/pixel 
byte SensorBlockStat[11]={0,0,0,0,0,0,0,0,0,0,0};//array of bits(8/bank) each high if corresponding sensor tripped
          // If SensorBlockStat[b] ==0 then NO sensors in that block set are occupied (sensor[b0] is LSB in byte)
unsigned int quad[5]={0,0,0,0,0};    //a temporary place to hold current sensor image quadrant brightness values 
unsigned int bright;       //  (4xquad[0-3] & total in quad[4] )
int  brightSF=BRIGHTSF;    //a scale factor to weight brightness variation when computing diff + bright*brightSF
bool SensorStat[80];       //state occupied/unoccupied true/false  (can set using l%% or o%%.  They set inactive)
bool SensorActive[80];     //if false (inactive) then don't update SensorStat[]  (can set active with a%%)
byte SensorActiveBlk[10];  //each holds 8 sensor bits(8 bits/block) e.g. SensorActive[07-00] in SensorActiveBlk[0]
byte SensorTwin[80];       //if sensors in pairs use to identify secondary bsn.  (should save into eprom!)
byte pvtThreshold[80];
int8_t lineardX[80];    //xlength of a single linear Sensor[%%] pixels (+/-63 max)

int  min2flip = 2;         //number of exceptional frames required before transitioning output tripped/untripped.
byte mask[8]= {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
int bsNo;               //parameter from text commands
int bsn;                //general use
int bsnUpdated=-1;
int dbsNo=0;            //bsNo chosen with d%% cmd.
int i;                  //local index counter
int j;                  //local index counter
int b=0;                //local (block) variable
int h7block=1;          //h7 stop on block # trip
int aRefCtr=-1;         //for 'a' cmd
int absn=0; //char absNo1='0', absNo2='0';  //save 'a' bsn for 'r'

char i2cCmd[64]={'0'};      //string from Master (onReceive)
byte dataPkt[128]={CAMERR};   // prepared packet(s) for i2c (max packet=32 bytes)
bool newi2cCmd=false;    // flags the presence of a new i2c serial command string for processing  
                                                                                           
//int  wifi=1;                //identify default wifi router (1 or 2)
bool MyWebServer=true;
char Shedssid[]=WIFI_SSID;
char ShedWIFIpwd[]=WIFI_PWD;
char Altssid[]=ALTWIFI_SSID;
char AltWIFIpwd[]=ALTWIFI_PWD;
char* ssid = Shedssid;       //must match local WiFi !
char* password=ShedWIFIpwd;  //must match local WiFi !
char* ssid2 = Altssid;                 //alternate ssid use with "v2" cmd
char* password2=AltWIFIpwd;

int  fbheight = 240;    //number of rows in fb default QVGA 320x240
int  pitch=640*3;       //image pixels/row (RGB888 VGA 640x480)
char cmdString[64];     // holds a whole command line from input source
char cmdChar;
bool Yhold=false;       //halt refreshing frames so multiple 'y's can see same fb
int  Xcolumn=0;         //default value for 'x' command
int  Yrow=0;            //default for 'y' cmd
int  Zlength=320;       //default full line for 'z' command
char Yheader[10]="y_x_z_ck:";

bool SendYpacket=false;
char i2cData[64];       //array of sensor status data to send on 't' request
bool scroll=true;       //if false suppress scrolling output (use 't0')
int  tcounter=0;        //down counter for delayed scroll "off"
int  i2cDatai=0;        //array index

int  threshold=42;      //difference (max Xratio+brightnessRatio) threshold for occupancy
int  maxDiff=0;         //exact Xdiff match produces minimum maxDiff of 32 
int  dFlag=0;           //flag set by d%% to print out (while>0) diff and sample image for bsNo.
int  dMaxDiff=0;        //saved maxDiff for 'd' cmd.
int  dBright=0;         //saved bright for 'd' cmd.

unsigned int releaseTime=0;  //counter for loop cycle timing control (target 10Hz?)
unsigned int timer;          //*code subsection time measurement ESP32() returns uint32_t NOT long(64)
unsigned int timerLoop=0;    //*loop time measurement
unsigned int cFlag=0;        //*flag to set off full Sensor_ref[] refresh of all active sensors (triggers c##
unsigned int cDelay=10000;  //*cDelay mSec after c## before updating active ref's

int sScan=-1;           //flag set by s%% to scan for, and set, new Sensor[sScan] position 
int sScanCount=3;       //get several frames before doing a scan to clear pipeline
int refBrightness=0;    //reference brightness sum of 16 pixles of Sensor_ref[00]
int refActual=0;        //actual Sensor[00] brightness on current frame
int prefillRef=90;      //after first 9sec of fb_gets() (plus seconds from cFlag?) take 80 Sensor reference grabs
byte *imagePtr;         //full image buffer pointer
byte *imageFB;          //ditto used by 'y' processing
byte rgb666[3]={0,0,0}; //destination for decodeRGB565() (one pixel)
String cStr=" ";        //string to hold latest auto adjust settings (AWB AEC AGC CBar)

int  averageRbsn=-1;          //a flag for main loop to do averaging
int  averageRcounter = -1;    //down counter for frame averaging (see 'r');
unsigned int stdCtimer=3600*1000*24;  //*24hours minimum - use with 'c' comand  (int=32 bit)

void startCameraServer();      //cameraInit() seems to initiate TwoWire(1) for CAM internal i2c

  // *********************************************************************

void setup() {   
    Serial.begin(BAUD);   //downloads with 115200+. Note: Mega still using 9600 for more reliable visual basic!
     Serial.setDebugOutput(true);
    Serial.println();
     printf("\nHello\nBCD version No %d\n",BCDver);          //my software version
     printf("Total heap: %d\n", ESP.getHeapSize());   //356488  //BCD: added from esp32-how-to-use-psram  
     printf("Free heap: %d\n", ESP.getFreeHeap());    //331728  //set Arduino IDE Core Debug Level: "Verbose"
     printf("Total PSRAM: %d\n", ESP.getPsramSize()); //4192139 //should output to SerialMonitor
     printf("Free PSRAM: %d\n", ESP.getFreePsram());  //4.192139 for 4MB PSRAM

  for (i=0;i<(80*16*3);i++){Sensor666[i]=10; Sensor_ref[i]=10;} //ensure none remain at 0 or risk /0 reboot
  for (i=0;i<(80*12);i++)   SensorRefRatio[i]=1;                      
  camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;   //AIthinker No
    config.pin_d0 = Y2_GPIO_NUM;        //5
    config.pin_d1 = Y3_GPIO_NUM;        //18
    config.pin_d2 = Y4_GPIO_NUM;        //19
    config.pin_d3 = Y5_GPIO_NUM;        //21
    config.pin_d4 = Y6_GPIO_NUM;        //36
    config.pin_d5 = Y7_GPIO_NUM;        //39
    config.pin_d6 = Y8_GPIO_NUM;        //34
    config.pin_d7 = Y9_GPIO_NUM;        //35
    config.pin_xclk = XCLK_GPIO_NUM;    // 0 *****!
    config.pin_pclk = PCLK_GPIO_NUM;    //22
    config.pin_vsync = VSYNC_GPIO_NUM;  //25
    config.pin_href = HREF_GPIO_NUM;    //23
    config.pin_sccb_sda = SIOD_GPIO_NUM;//26
    config.pin_sccb_scl = SIOC_GPIO_NUM;//27
    config.pin_pwdn = PWDN_GPIO_NUM;    //32
    config.pin_reset = RESET_GPIO_NUM;  //-1
    config.xclk_freq_hz = 20000000;     //20MHz
 
    pinMode(WEBPIN, INPUT_PULLUP);              //startup() uses GPIO(14?) (LOW) to trigger WebServer (jpg) mode

    EEPROM.begin(EEPROM_SIZE);  //will need to load 80 (long)Sensor[] pointers from EPROM
    delay(500);                 //just copying an EPROM example (why?)
    i=int(EEPROM.read(EPvFlag));   //if 1-2 initiate web server via wifi 
    if(i==2){ssid=ssid2; password=password2;}
    printf("Video flag read as %d; Webpin %d\n",i,(1-digitalRead(WEBPIN)));
    if(i==0){    //EPROM contains data so use stored parameters - restore threshold,nLED,min2flip,maxSensors
      nLED=int(EEPROM.read(EPnLED));
      if (nLED>9) nLED=2;                        //set a default if new EEPROM
      minSensors=int(EEPROM.read(EPminSensors));      //use S00-S97(79) default: 0
      if(minSensors>79) minSensors=0;      
      threshold=int(EEPROM.read(EPthreshold));      //use 32-254 default: 45?
      if(threshold==255) threshold=45;
      min2flip=int(EEPROM.read(EPmin2flip));        //default: 2
      if(min2flip>9 || min2flip==0) min2flip=2;     //checks for unprogrammed EPROM
      maxSensors=int(EEPROM.read(EPmaxSensors));    //debug print limit
      printf("EEPROM set threshold= %d; nLED= %d; min2flip= %d; maxSensors= 0%o\n",\
              threshold,nLED,min2flip,maxSensors);
    }
    for (j=0;j<80;j++) SensorTwin[j]=0;     //initialise Twins, if none in eprom.
    
    if((i==0) && (digitalRead(WEBPIN)==1)){   //if HIGH startup in RGB565 mode (else MyWebServer mode)
      MyWebServer=false;                       //do not start MyWebServer                            
      config.pixel_format = PIXFORMAT_RGB565;   // YUV422|GRAYSCALE|RGB565|JPEG (see esp_camera.h)  
        // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
        //                      for larger pre-allocated frame buffer.
      if(psramFound()){
        config.frame_size = FRAMESIZE_QVGA;   //QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
        config.jpeg_quality = 10;           //0-63 0 for highest quality?
        config.fb_count = 2;
        IFD Spr("fb_count set to 2 as PSRAM found\n"); 
      } else {                         //setup webserver mode        
        config.frame_size = FRAMESIZE_QVGA; //select optimum size 640 x 480 for Sensor (speed v resolution)
        config.jpeg_quality = 12;           //4 better quality BUT slows any conversion to BMP by ~10%
        config.fb_count = 1;                //investigate speed gain using 327kB fast dynamic RAM. for a 154k fb  
      }
      pinMode(FLASHLED,OUTPUT);digitalWrite(FLASHLED,LOW);  //turn off flash LED
      //pinMode(FLASHLED,OUTPUT);                           //to stop cycle flash set FLASHLED to another pin (33?)
      pinMode(BLK0LED, OUTPUT);digitalWrite(BLK0LED,HIGH);  //normal LED HIGH=off BUT (GPIO12 inverted. AVOID 12!)
      pinMode(BLK1LED, OUTPUT);digitalWrite(BLK1LED,HIGH);  //set up bank occupied indicators (GPIO 2,14,4,33)
      pinMode(BLK2LED, OUTPUT);digitalWrite(BLK2LED,HIGH);
      pinMode(BLK3LED, OUTPUT);digitalWrite(BLK3LED,HIGH);  //consider logic reversal for all ext. LEDS
      pinMode(PLED, OUTPUT);digitalWrite(PLED,HIGH); 
      pinMode(QLED, OUTPUT);digitalWrite(QLED,HIGH);          

    }else{ MyWebServer=true;                  //run CAM in webserver mode (no track Sensors)
        pinMode(FLASHLED,OUTPUT); 
        flashLed(25);         //pulse flash to show webserver mode initiation
        
         //clear EEPROM Web flag and proceed to webserver
        EEPROM.write(EPvFlag,0);  //clear flag at end of EEPROM
        EEPROM.commit();
        EEPROM.end();   //burn eeprom   
    
      config.pixel_format = PIXFORMAT_JPEG;
   
      if(psramFound()){
        config.frame_size = FRAMESIZE_UXGA;
        config.jpeg_quality = 10;
        config.fb_count = 2;
      } else {                                          //could print error as must have psram to operate fully
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
      }
    }  //end of else{ MyWebServer=true; 

      printf("Camera init attempt...\n");
      esp_err_t err = esp_camera_init(&config);    // camera init
      if (err != ESP_OK) {
        printf("Camera init failed with error 0x%x", err);
        delay(10000);
        ESP.restart();
      }
      delay(2000);                  //Prof's mod to help startup
      MyWire.onReceive(i2cReceive);                //set up response to receive & request
      MyWire.onRequest(i2cRequest);                //links to void i2cRequest()
bool  MyWireOK=MyWire.begin((uint8_t)I2C_DEV_ADDR,I2C_SDA2,I2C_SCL2,0);  //Slave needs no FREQ - Master's choice
      if(MyWireOK) printf("MyWire.begin()INITIAISED OK\n");
      else printf("MyWire.begin()FAILED\n");

#if CONFIG_IDF_TARGET_ESP32                           //ESP32 needs a call to slaveWrite for Arduino compatibility
      uint32_t i2cCtr = 0;          //prepare for i2c 
      IFT printf("Doing slaveWrite() for ESP32\n");   //ESP32-S2 and ESP32-C3 don't need slaveWrite()
      char message[64];                               
      snprintf(message, 64, "E%u Packets.", i2cCtr++);//snprintf() puts the text "1 Packets" into char message[64]
      message[0]=CAMERR;
      MyWire.slaveWrite((uint8_t *)message, strlen(message)); //preload onRequest buffer
      IFT printf("Done slaveWrite\n");
#endif                                                        
      delay(4000);
 
    Spr("Camera initialised ");Serial.println(esp_err_to_name(err));
  
    sensor_t * s = esp_camera_sensor_get();

      // drop down frame size for higher initial frame rate
    s->set_framesize(s, FRAMESIZE_QVGA);   //THIS IS THE FINAL SETTING 

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#endif

/*                 //using CAMERA_FB_IN_DRAM got figures below CLEARLY ASSIGNING FB TO DRAM!
//  printf("AFTER good camera_init and parameter setting & before WiFi.begin\n"); 
//  printf("Total heap: %d\n", ESP.getHeapSize()); //356488->356212 BCD: added from esp32-how-to-use-psram  
//  printf("Free heap: %d\n", ESP.getFreeHeap());  //331492->258348 set Arduino IDE Core Debug Level: "Verbose"
//  printf("Total PSRAM: %d\n", ESP.getPsramSize());   //4192139 unchanged for DRAM. -> 4192127
//  printf("Free PSRAM: %d\n", ESP.getFreePsram());    //4.192139 MB!(DRAM) &  4153727 (PSRAM)(38400 for QQVGA)
*/      
    if(MyWebServer){        //set up WiFi connection                   
          //if WEBPIN held LOW at startup, MyWebServer=true so startup in MyWebServer mode
          //(if GPIO0 held low at startup then automatically goes into flash program mode)          
      ConnectWifi(ssid, password);            //WiFibegin()
      if (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("WiFi failed to connect.  Try alt-wifi (v2)?");
        ESP.restart();
      }
      Serial.println("");
      Serial.println("WiFi connected");

      startCameraServer();

      Spr("Camera Ready! To connect, use 'http://");
      Serial.println(WiFi.localIP());
 
      flashLed(250);       //long pulse flash to show MyWebServer mode established
    }
    else{                  //PROCEED TO SETUP SENSORS using RGB565 & QVGA     
      
      for (i=0;i<80;i++){  //set all to UNDEFINED, Brightness Sensor(0/0) defaults to offset 000 if not from EPROM
        Sensor[i]=0L;             //in absence of EPROM data set all sensors to offset 0 (i.e. Sensor UNDEFINED)
        SensorActive[i]=false;    //can set undefined sensors active as ALL sensors scanned at present anyway
        SensorActiveBlk[i>>3]=0;  //clear all Blk bytes also
      }
      for (i=0;i<80;i++){        //will need to load 80 (long)Sensor[] pointers from EPROM and 80 byte Twin values
        Sensor[i]=EEPROM.readLong(i*4);       //get a long (4byte) pointer from EEPROM. 
        SensorTwin[i]=EEPROM.read(EPSensorTwin+i);
        pvtThreshold[i]=EEPROM.read(EPpvtThreshold+i);   //if =255 then use default threshold
        lineardX[i]=EEPROM.read(EPlineardX+i);
		if (Sensor[i]>=153600L) Sensor[i]=0;   //eliminates wild values (all 1's) from a pristine EPROM
        if (Sensor[0]==0)         //153600L = size of image
          Sensor[0]=(320*120+160)*2;           //create first S00 as k00,120,160 (~centre)	  
        if (SensorTwin[i]>79)  SensorTwin[i]=0;          // 153600L = size of image  
																										
													  
        if (Sensor[i]!=0){  IFT printf("define Sensor[0x%x]= %lu \n",i,Sensor[i]);        
          SensorActive[i]=true;               //initially activate all sensors defined (i.e. >0) in EPROM
          SensorActiveBlk[i>>3]=SensorActiveBlk[i>>3] | mask[i&0x07]; 
        }    //Never obstruct Sensor[00].  If brightness SensorStat[00] ever goes true, we have a lighting problem
      }      //Note: if sensor[0]=0 (top left cell undefined) cmd 'r' will ignore it!
																												  
      delay(5000);                           //give the cam settings time to settle then renew
      Spr("Turn AWB & AGC OFF(0) after 5 sec. Leave auto adjustments (AWBg,AEC,AECd,AEL) ON(1)\n");
        //set AWB,AEC,AGC OFF,  or just AWB & CB OFF?? NEEDS MORE EXPERIMENTING

      new_camera_settings(Bri,Con,Sat,AWB,AWBg,AEC,AECd,AEL,AGC,AGg);    //set Sat,AWB,AEC,AGC,.. to defaults
      AWB=AWB9; AGC=AGC9;    //Ready for Turn Off AFTER prefillRef delay, say 9000mSec.
        //  new_camera_settings(0,1,2,0,1,1,1,1,0,9);  after prefill delay 
      printf("\n*** WILL WAIT >10 SECONDS THEN AUTOMATICALLY LOAD ACTIVE SENSOR IMAGE REFERENCES\n");
      printf("        ASSUMING ALL SENSORS UNOCCUPIED ***\n");

      cFlag=millis();     //record end of setup() time - used to determine when prefillRef happens
    }			   
}       //end setup()

  // *************************************************************************************************************

void loop() {
 //execte once per 100mSeconds (sample rate)
  static int  frameNo=0;         //variables for NOISE measurement code
  static int  Sensor666Av[48];
  static int  rollAvFrameR[64]={0}, rollAvFrameG[64]={0}, rollAvFrameB[64]={0}; //rolling Av. values of Sensor[00]
  static int  rollNsFrameR[64]={0}, rollNsFrameG[64]={0}, rollNsFrameB[64]={0}; //rolling Noise values
  static int  rollAvR=0, rollAvG=0, rollAvB=0;   //RGB components
  static int  r00Av=0;
  static int  rollNsR=0, rollNsG=0, rollNsB=0;   //RGB components
  static int  r00NsR, r00NsG, r00NsB;
  int  cmdCount = 0;       // number of characters in serial cmdString[]
  int  Noise=0;            //difference between frame pixel and ref pixel
  int  nParam=0;           //number of parameters parz() found in command line
  int16_t p[5]={-1,-1,-1,-1,-1};       //repository for parz() command parameters 
  
/***/  if(prefillRef>1) {Serial.print(prefillRef);Serial.print(" ");}
// ****IF MYWEBSERVER MODE, JUST MONITOR SERIAL PORTS FOR RESET (R or F) 
  while(MyWebServer) {          //monitor for character to trigger reboot back to sensor mode
    if (Serial.available() > 0)  {   //if MyWebServer mode, do nothing but serve web and monitor for Reset cmd.    
      cmdChar=Serial.read();
      if((cmdChar=='R') || (cmdChar=='F')) { //'F' gives Bluetooth ability to "Finish with webserver"
        Serial.println("Restarting in sensor mode\n");        
        WiFi.disconnect(true, true); 
        delay(3000);          
        ESP.restart();          //give up on wifi and return to sensor mode
      }else Spr(cmdChar);
      if((cmdChar=='g') || (i2cCmd[0]=='g')){          //get camera status
        i2cCmd[0]='.';
        getCAMstatus();
      }
    }         //end if (Serial.available())
    if ((i2cCmd[0]=='R') || (i2cCmd[0]=='F')) ESP.restart();        //exit back to sensor mode
  }       //end MyWebserver code
  
// ***CALCULATE AND PRINT LOOP TIME - & IF CALLED FOR, UPDATE NEW CAMERA SETTINGS   
/***/   IFT Spr("\n******** ");
      int loopTime=millis()-timerLoop; 
      if(loopTime==99) loopTime=100;	//add 1 just to stop display jitter with 99
      timerLoop=millis();
      if(loopTime<100)Spr(' ');
        IFsc if(tcounter>0){tcounter--; if(tcounter==1) scroll=false;}
        IFsc{ Spr(loopTime); Spr("mS; "); }   
      HistoLoops++;                 //increment loop counter
      if (stdCtimer < millis()) {   //time to invoke new camera settings after 'c' std delay
        stdCtimer=1000*3600*24*10;      //set timer too large to ever trip again. Long 2^31 > 2,147,000,000,000
        new_camera_settings(Bri,Con,Sat,AWB,AWBg,AEC,AECd,AEL,AGC,AGg);  //set Sat,AWB,AEC,AGC,..   select AWB OFF
        cFlag=millis(); //Time stamp Flag for loop() to reload all references after cDelay sec of flushing frames  
      }

// ***TAKE A FULL FRAME INTO fb IN RGB565 FORMAT 
/***/ IFD Serial.println(" Try a get_picture ");  // Get Picture from Camera.  New image starts when fb released.  
/***/ IFT  timer=micros();                     // Images are "pipelined" so images are 2 "gets" old. (160mSec)
    camera_fb_t * fb = NULL;                  //transfer rgb565 image #1 into fb (taken  
    fb = esp_camera_fb_get();                 //get most recent frame into fb (and return pointer?)
    if(!fb) { Serial.println("Camera capture #1 failed\n"); return;}   //no pic fb ? - restart program loop    
/***/ IFT{ Spr(timer);timer=micros()-timer;//does camera_fb_get return error if .jpg not ready, or does it wait
/***/    Spr(" fb_get time:"); Serial.println(timer);   //zStr); 
/***/ } 
    fbheight=fb->height;
/***/ IF0 printf(" got RGB565 frame. fbheight %d fb->len %d\n",fbheight,fb->len); 
 
// ****DECODE RGB565 FRAME INTO COMPACT SENSOR FRAME OF RGB666 FORMAT
/***/ IFT  timer=micros();    
      int pitch=RGB565p*fb->width;        //set pitch for RGB565 image format
      imagePtr=fb->buf;  
      bool converted = f565to666(imagePtr,pitch,Sensor,Sensor666,SensorActive);//decode rgb565 to rgb666 in Sensor666
/***/ IFT{ timer=micros()-timer; Spr("Decode (uS):"); Serial.println(timer);}
     
      if (!converted) {Spr("Conversion to rgb666 failed\n"); return;}
/***/ IFD printf("should now be rgb666 in Sensor666[]\n");      

// ****SEE IF IMAGE DUMP (TO PROCESSING4)CALLED FOR & DUMP AS REQUESTED BEFORE NEW FRAME INITIATED
      if(Yhold) cmdCount=0;      //reset command string index & prepare for a new command following recent y#
      while(Yhold){   //do not proceed with loop() until Yhold cleared by "yy\n" command
        if(SendYpacket){
          imageFB = fb->buf;     //set a ptr to start of image buf
          if(boxSensors){        //put a box around active sensors in image   
            for (int sen=0; sen<maxSensors; sen++) {        //limit to first maxSensors
              if (SensorActive[sen]) boxIt(Sensor[sen],sen);    //box 4x4 sensor at Sensor[] location              
            }
            boxSensors=false;    //don't do again - only on first 'y' command of series
          }     
          j = Xcolumn*2 + Yrow*320*2;                   //starting point in data   //then send an ASCII header
/***/     Spr('\n');Spr('y');Spr(Yrow);Spr('x');Spr(Xcolumn);Spr('z');Spr(Zlength);Spr(";\n"); //printf() too slow
          Yheader[1]=byte(Yrow); Yheader[3]=byte(Xcolumn/2); Yheader[5]=byte(Zlength/2);
uint32_t  Ycksum=0;              //Calculate checksum
          for (i=0;i<6;i++) Ycksum += int(Yheader[i]);  //does not include chsum or ':' in summation
          for (i=j;i<(j+Zlength*2);i+=2) {
            if(!imageFB[i])  imageFB[i] = 0x20;        //tweak to data so no NUL bytes by adding LSB to even green
            if(!imageFB[i+1]) imageFB[i+1]= 0x08;       //ditto red  (Note: 0x08 = BS backspace)          
            Ycksum += imageFB[i]; Ycksum += imageFB[i+1];
          }
          Yheader[6]=byte(Ycksum & 0x00FF); Yheader[7]=byte(Ycksum >>8);
          i=Serial.write(Yheader,9);                    //write 9 header bytes
          if(i != 9) printf("Y header write error %d\n",i);
          i=Serial.write(&imageFB[j],Zlength*2);        //write data
          if(i != Zlength*2) printf("Y data write error %d\n",i);
          SendYpacket=false;    
        }

        while (Serial.available() > 0) {   //each command is followed by a fresh loop to "get_image"
          cmdChar=Serial.read();
          cmdString[cmdCount]=cmdChar;               //add it to cmd string
          cmdCount+=1;
          cmdCount=constrain(cmdCount,0,20);            //increment & limit string length
          if (cmdChar == '\n'){                   //don't process cmdString until get LF (\n)
 
            nParam=parz(cmdString,p);
/***/  //       printf(("parz %d parameters; p[0] %c %d %d %d %d \n"),nParam, byte(p[0]),p[1],p[2],p[3],p[4]);            
            processCmd(imagePtr,pitch,Sensor,nParam,p);  //process command line            
            cmdCount=0;      //reset command string index 
          }     
        }
      }    //end of while(Yhold)


// ****BEFORE DISCARDING RGB565 fb FRAME, CHECK IF NEEDED TO PROCESS A sCAN or cALIBRATE COMMAND REQUEST
      long bright_spot=0;     //pointer to the brightest spot in BMP/RGB666 image
      if (sScan>=0){      // scan full frame for a bright new Sensor[sScan] offset position 
        sScanCount -=1;   // decrement frame count to flush pipeline
        if(sScanCount==0){ 
          bright_spot = scan( imagePtr, pitch,fb->width, fbheight, RGB565p); //check imageData for a bright spot
          Sensor[sScan] = bright_spot - long(2 * RGB565p + 2 * pitch);      //point to first byte top left corner             
          printf("Scan: New Sensor %d/%d(%d) bright pixel position: row=%d column(x)=%d offset %lu\n",
                             sScan>>3,sScan&7,sScan,int(bright_spot/pitch),int(bright_spot%pitch)/2,bright_spot);
          printf(" YOU MUST DO an r%d%d TO RECORD NEW IMAGE REFERENCE DATA *AFTER* removing brightspot and "
                                                                     "restoring light level.\n",sScan/8,sScan%8);
          printf(" Press Enter for fresh frames (DO NOT DO r%%%% here)\n");
          sScan=-1; // clear request.  //NOTE: printed pixel pos'n is the bright_spot NOT the Sensor corner pos'n 
          wait();                      //  (sensor corner reduced by 2*pitch+4)
        }
      }
      
// ****DO A STEP TOWARDS AVERAGING Sensor[bsNo] IF COUNTER SET.
      refRefresh(autoRefSuspended);    //try to auto refresh a bsNo reference (if not tripped)
      if (averageRcounter>0) averageRcalculation(averageRbsn,AVCOUNT);    //update average ref[bsn] process ('r')
      
// ****CALCULATE AVERAGE FOR Sensor[00] AND AVERAGE NOISE (& PEAK NOISE?) 						 
     frameNo += 1; frameNo &= 0x3F;       //inc. frame counter reset to 0 after 0x3F(63)
     if(frameNo==1) {  //every 64th frame update Sensor_ref[00] & print 4x4 new av & current red(noisiest?) values    
/***/   IFN Spr("av&cur Red ref[0] ");   //NOTE THIS DOES NOT UPDATE BRIGHT OR COLOUR RATIOS!
        for (i=0;i<48;i++) {                 //48 = 4x4pixels*3colours
          Sensor_ref[i]=Sensor666Av[i]>>6;  //update Sensor_ref[0/0] every 6.4 seconds (64 frames) (av = sum/64)
/***/     IFN{ Spr('&');Spr(Sensor_ref[i]); }        //print whole ref (DEC)   
/***/     IFN if(i%3==0) {Spr(" [");Spr(i);Spr("]:");Spr(Sensor_ref[i]);Spr(' ');\
/***/           Spr(Sensor666[i]);} //print (DEC) one (red) colour of new sensor_ref[00] & actual[00]                
/***/     Sensor666Av[i]=0;                 //initialise for next 64 frames
/***/   }  
/***/   IFN Spr("\n.......");              //try to keep next line aligned.      
     }
  //compute new cratios etc for Sensor[00] latest av. reference UPDATE sensor_ref(00) every 6.4sec to new average
     if(frameNo==2) {                       //grab_ref() includes calculation of new Cratios & brightness
       grab_ref(0,Sensor_ref,S666_pitch,0L,&Sensor_ref[00],&Sen_Brightness_Ref[00],&SensorRefRatio[00]);  
       Sen_Brightness_Ref[00]=r00Av; //Sum of (grab_ref)rounded averages is < Average of sum, use r00Av instead.
/***/   IFN {Spr("adjusted _Brightness_ref[00] to Average "); Serial.println(Sen_Brightness_Ref[00]);}
     }
int  sumR=0; int sumG=0; int sumB=0;        //calculate new bright(00) ( <= 48*63 (3024max)) 
     for (i=0;i<48;i+=3){                   //calculate colour averages (NB. not colour ratios!)
        sumR=sumR+Sensor666[i]; sumG=sumG+Sensor666[i+1]; sumB=sumB+Sensor666[i+2];   //calculate rolling averages
        Sensor666Av[i]  += Sensor666[i];    //calculate individual pixel colour averages for _ref 
        Sensor666Av[i+1]+= Sensor666[i+1];
        Sensor666Av[i+2]+= Sensor666[i+2];        
     }                                    
           //calculate whole sensor rolling averages (sumR already contains sum of R in current frame)
     rollAvR=rollAvR+sumR-rollAvFrameR[frameNo]; rollAvFrameR[frameNo]=sumR;//r00AvR=rollAvR>>6;//av.of Sensor[00]
     rollAvG=rollAvG+sumG-rollAvFrameG[frameNo]; rollAvFrameG[frameNo]=sumG; //r00AvG=rollAvG>>6;   
     rollAvB=rollAvB+sumB-rollAvFrameB[frameNo]; rollAvFrameB[frameNo]=sumB; //r00AvB=rollAvB>>6;
     r00Av = (rollAvR+rollAvG+rollAvB)>>6; //div by 64 frames to get 0-3024 (48*0x3F) av. s00 frame sum (bright) 
       
     sumR=0; sumG=0; sumB=0;
     for (i=0;i<48;i+=3){          //calculate av noise
               //calculate sum of noise squared for CURRENT 4x4 frameNo.  aiming for RMS equivalent
        Noise = Sensor666[i]   - Sensor_ref[i];   sumR=sumR+Noise*Noise;    
        Noise = Sensor666[i+1] - Sensor_ref[i+1]; sumG=sumG+Noise*Noise;
        Noise = Sensor666[i+2] - Sensor_ref[i+2]; sumB=sumB+Noise*Noise;
     }    //sumR is now sum from 16 pixels in one frame 
     rollNsR=rollNsR+sumR-rollNsFrameR[frameNo]; rollNsFrameR[frameNo]=sumR; r00NsR=rollNsR>>6;  //calculate rolling average noise on Sensor[00]
     rollNsG=rollNsG+sumG-rollNsFrameG[frameNo]; rollNsFrameG[frameNo]=sumG; r00NsG=rollNsG>>6;  //divide by 64.
     rollNsB=rollNsB+sumB-rollNsFrameB[frameNo]; rollNsFrameB[frameNo]=sumB; r00NsB=rollNsB>>6;  //16 for av/pixel
     IFN printf("s00 roll'n brAv=%d, noisAv=%d %d %d s",r00Av,r00NsR,r00NsG,r00NsB);  //SHOULD THESE BE /16 as 16 pixels in every r00Ns%

// ****FOR FLUORESCENT LIGHTING TRY TO SYNCHRONISE release/start image WITH 50Hz MAINS USING INTERNAL CLOCK
// ****DELAY TO REDUCE FRAMES PER SECOND FROM one/80mS to one/100mSec FOR PSRAM 25% IDLE (DE-STRESS) TIME. 
      while ((micros()-starttime)>CYCLETIME) {starttime += CYCLETIME;} //bring it back into sync if loop is late
      while ((micros()-starttime)<CYCLETIME) {;}  //kill any remaining spare time before 50/60Hz sync.
      starttime += CYCLETIME;      
    // above should sync with mains just before releasing fb to be refilled.
    // this is based on the premise that the mains frequency is very accurate, stable & doesn't drift noticeably! 
    // also assumes ov2640 is restrained to run at 10Hz (This may not be true - suspect free runs at ~13Hz!) 

 /***/ IFT {Spr(starttime);Spr("uSec ");}
 /***/ IFT timer=micros();  
 
      esp_camera_fb_return(fb);         //release jpg camera image frame buffer - starts new frame capture? 
         //sensor images now in 3 byte format in array Sensor666[] 	 
/***/ IFT{ Spr("50Hz fb return(uS) "); Serial.println(timer);}
     
// ****IF CALLED FOR BY c####, OR STARTUP(), DO A FULL REFERENCE UPDATE FOR ALL ACTIVE SENSORS
       //if cFlag not 0 then all defined sensors should be prefilled from good stable unoccupied image, i.e.AFTER
       // CAM has run for > prefillRef frames. 
      if (cFlag!=0) prefillRef = S666init_SensorRefs(prefillRef);  //Sensor_refs[],related brightness,cRatios etc.
            
/***/ //Print fb data sample         //NOTE this is AFTER release of fb so use saved decoded Sensor666[] images!!!
/***/ IFD write_img_sample(Sensor666,S666_row,0x08*48,24);//write current image of sensor[00] AND Sensor[bsn(1/0)] 

// ****NOW WANT TO CLEAR CAMERA PIPELINE & START NEW IMAGE CAPTURE FOR NEXT LOOP  
/***/IFT timer=micros();    
//   WITH FAST rgb565 decode above, it isn't worthwhile trying to clear out pipeline AS WAS NEEDED WITH jpg TO bmp                                     

// ****DO COMPARE OF ALL SENSORS WITH THEIR REF'S, DECIDE IF OCCUPIED & IF SO SET STATUS.
/***/IFT timer=micros();   
    //at start of program preload sensor_ref[] automatically after (prefillRef) images
    // LOOP UNTIL prefillRef frames acquired at which time S666init_SensorRefs(prefillRef) will load ref. images.
    if(prefillRef>0){           //at start of program there is time to kill before ref.images are auto grabbed 
/***/   if (prefillRef>1) {     //during initial countdown, output some debug images to see changes (if obvious)       
/***/     IF0 for (i=0;i<3;i++){ printf("Sensor_ref[]:");                     //print out first 3 SensorRefs
/***/         for (int j=0;j<48;j++) printf(" %x",Sensor_ref[i*48+j]); printf("\n");
/***/     } 
/***/     IF0 printf("Quad: %u %u %u %u quad[4]:%u\n",quad[0],quad[1],quad[2],quad[3],quad[4]);                                             
/***/   }    
/***/   IFD for (int i=0;i<3;i++){     //print Cratios for first 3 sensors
/***/        printf("\nbsNo %d SensorRatios: ",i); for (int j=0;j<4;j++)printf(" %3d %3d %3d :",\
/***/                           SensorRefRatio[i*12+j*3],SensorRefRatio[i*12+j*3+1],SensorRefRatio[i*12+j*3+2] );
/***/   } printf("\n");    
    }else{                           // DO NORMAL LOOP COMPARISON
      i2cDatai=1;                    //index for i2c message data entry - leave [0] for 't'
      avOddFrame=!avOddFrame;        //flip flag for av2frames() execution
      for(bsn=0;bsn<80;bsn++){ if(bsn==1)timer=micros(); //leave bsn==0 out of timing to see effect.
        if (SensorActive[bsn]){     //skip compare if Sensor NOT active! (inactive sensors can't  
                                    //                    auto update/track brightness drift (& saves time))
           // ****USE 2 image average for compare if bsNo < TWOIMAGE_MAXBS as a test for improved algorithm  
          if (bsn < TWOIMAGE_MAXBS) av2frames(bsn);    //do 2xframe av. if active & bsNo < 2/0   & compare current S666 image
          maxDiff= compare( Sensor666, S666_row, long(bsn*S666_pitch), &SensorRefRatio[bsn*12], bsn, RGB565p); 
          processDiff(bsn);    //make a decision on whether to set occupied or not and set status[bsn] accordingly
              //processDiff recomputes bright & sets/resets all sensor status flags
          if (bsn==dbsNo) {dMaxDiff=maxDiff; dBright=bright*brightSF;} //save for delivery on dflag or i2c request
        }                        
        if(bsn==0){         //special treatment for brightness reference sensor[00]
/***/     IFD {Spr("bsNo 0/0 maxDiff: ");Serial.println(maxDiff);}          
          refBrightness=Sen_Brightness_Ref[0];   //a refBrightness from last time ALL references updated (startup, 'r00' or 'c' only)             
          refActual=quad[4];   //save refActual in case use later
/***/     IFT { Spr(refBrightness);Spr(" refBrightness > actual ");Serial.println(refActual);}  //sensor[00]
          else IFsc if(minSensors==0){ Spr(" T");Spr(threshold);Spr(" N");Spr(nLED);Spr(" R");
            Spr(refBrightness);Spr(" A");Spr(refActual);
            Spr(" M");Spr(min2flip);Spr(" B");Spr(brightSF);Spr("\t");   //short & no \n
          }
   //       bsn=minSensors;     //then skips active/enabled sensors up to minSensors!
        }   //end if(bsn==0)
      }     //end for(bsn=0->80) 
      i2cData[1]=byte(threshold);  //replace first i2c byte (bsn=0?) with threshold (don't want NULL bsn at start!
      i2cData[i2cDatai]=0x50;      //put block "80." to flag end of i2cdata[] (max valid bsn=79.)
      i2cData[i2cDatai+1]=0x00;    //put a NULL on end of data message to end transmission
      
      timer=micros()-timer;
/***/ IFT {Spr("Compare(uS): "); Spr(timer,DEC);}
      if(bsnUpdated>=0) { IFsc{ Spr("Ref 0");Spr(bsnUpdated,OCT);} bsnUpdated=-1;} 
      IFsc Serial.println("");              //was consistently getting same No. from (timer,DEC) & (zStr) (until neg nos)

// ****CHECK DFLAG AND OUTPUT REQUESTED INFO.  (BUG - doesn't look like it prints requested dbsNo DIFFERENCES only LAST active sensor leftover?)
        //if dFlag==0 just output diffs. if dFlag >0 output image table and repeat dFlag times
      if(dFlag>=0) {
        printf("Diff: bsNo %d/%d diff %d bright %d diff+bright %d\n",dbsNo>>3,dbsNo&7,dMaxDiff,dBright,dMaxDiff+dBright);
         // also need to send something to i2c - do this in i2cRequest() if last i2cReceive was 'd' 
        dFlag-=1;          //decrement towards -1 
        if(dFlag>=0){      //d%%n sets dFlag so can output debug sample images for bsNo '%%' on each of 'n' loops     
          write_img_sample(Sensor666,S666_row,long(dbsNo*48),24); //write for visual check what is being compared.
          wait();          //give user chance to read          
        }
      }     //end if(dFlag) code
    }       //end (normal loop comparisons) (prefillRef==0)
   
// ****FLASH A LED ON EACH LOOP.  Use FLASHLED pin
    if (FLASHLED==4) flashLed(20);            //flash White led on IO4 briefly (~25uSec for ESP32-CAM)
    else { digitalWrite(FLASHLED, HIGH); delay(5); digitalWrite(FLASHLED, LOW); }  //wastes 5% of loop time!*****
    setLED(nLED,true);    // set the PLED and QLED block status indicators also
 
// ****CHECK FOR USB COMMAND INPUT -  PROCESS ANY COMMAND
/***/ IF7 if (SensorBlockStat[h7block] != 0){Spr("debug-trip in Block:");Serial.println(h7block);wait();} 
                                                                                //wait if Block # occupied
    if(aRefCtr>0){  aRefCtr--;       //do we need to do a ref for new sensor yet?
      if(aRefCtr==1){     
        nParam=2;
        p[0]='r'; //byte('r');
        p[1]= absn;       
        processCmd(imagePtr,pitch,Sensor,nParam,p);  //process 'r' command
        cmdCount=0;      //reset command string index  	 
      }
    }  
    if(newi2cCmd==true){     
/***/ IFI2C {Serial.print("I2C:");Spr(char(i2cCmd[0]));}
      for (i=0;i<64;i++)cmdString[i] = i2cCmd[i];     //copy i2cCmd to cmdString
/***/ IFT {Spr("****************i2cCmd: ");Spr(i2cCmd);}
      if(cmdString[0]=='w') newi2cCmd=false;  //this ensures 'w' actually waits - other i2c commands DON'T WAIT
      nParam=parz(cmdString,p);     // parse ascii cmd
/***/ // printf(("parz - %d parameters; p[0-4]: %c %d %d %d %d bsN(p[1]): 0%o \n"),nParam, p[0],p[1],p[2],p[3],p[4],bsN(p[1]) );      
      processCmd(imagePtr,pitch,Sensor,nParam,p);      //process command line
      if(cmdString[0]!='w') newi2cCmd=false;  //if() lets newi2cCmd cancel 'w' without being discarded itself.
    }
    else while (Serial.available() > 0) {         //each command is followed by a fresh loop to "get_image"
      cmdChar=Serial.read();
      cmdString[cmdCount]=cmdChar;               //add it to cmd string
      Spr(cmdChar);                      //immediate echo back to monitor
      cmdCount+=1;
      cmdCount=constrain(cmdCount,0,20);            //increment & limit string length
      if (cmdChar == '\n'){                   //don't process cmdString until get LF (\n)
        Serial.println("**************"); // return a new line to monitor
        
        nParam=parz(cmdString,p);     // parse ascii cmd
/****/  IF5 printf(("parz: %d parameters; p[0-4]: %c %d %d %d %d bsN(p[1]): 0%o \n"),nParam,p[0],p[1],p[2],p[3],p[4],bsN(p[1]));
                   
        processCmd(imagePtr,pitch,Sensor,nParam,p);  //process command line 
        cmdCount=0;      //reset command string index 
      }     
    }
/***///output reference images sensor_Ref[00] & sensor_Ref[02] for diagnostics purposes captured Sensor666 array  
/***/ IF0 disp_sample_pixels( Sensor666, S666_row, 4 * (01) ,  1*16,   24); //dispX/16 will move multiples of bsNo
 
/***/ IF0 delay(1000); //delay a second for serial.print debug to catch up?
}               //end of main loop
/////////////////////////////////////////////////////////////////////////////////////////////////////////

//  PROCESS A COMMAND STRING FROM USER    
void processCmd(byte *imagePtr,int pitch,unsigned long *Sensor,int nParam,int16_t *p){    //process parz command in p[]                                        
    //imagePtr: is a pointer to the location of image Data  NOTE no longer used by processCmd !
    //pitch: is the number of bytes in a full line of fb imagePtr
    //Sensor: is a pointer to the Sensor array of pointers into the image Data
    //nParam: the number of parameters in p[]
    //p: is pointer to array of command parameters

int b=0;        //local (bank) variable
int c=0;
int i, j;       //j: sensor index
int param2;     // second cmd argument
int bsn=0;
 
    if(nParam > 0) 
     switch(p[0]) { 
      case 'a':{      //a%%;  enAble Sensor[bsn] and get fresh reference (only do if UNOCCUPIED!)
        absn = p[1];  //save decimal straight from parz() for later use
        bsn = bsN(p[1]);    // (can't produce out of range bsn - defaults to 00)       
        if(nParam==1) { printf("(a%%%%[,rrr,xxx]) enAble Sensor(%%%%)[& sets new coordinates for it]\n"); break; }
        if(nParam==4) {
          int rowVal=p[2];
          if(rowVal<0 || rowVal>235) {printf("invalid rowValue\n"); break;} 
          int xVal=p[3];          
          if(xVal<0 || xVal>319) {printf("invalid xValue\n"); break;}
          pvtThreshold[bsn]=255;   //new coordinates so cancel pvtThreshold. ('k' doesn't clear it)
          Sensor[bsn]=pitch*rowVal + 2*xVal; 
          printf("(a%%%%[,rrr,xxx]) new coordinates. Sensor[%d/%d] to r:%d x:%d\n",bsn>>3,bsn&7,rowVal,xVal);                    
          aRefCtr=4;        //get # fresh frames then take ref for sensor[aRef]
          wait();
        }      
        if (bsn>=0) { printf("(a%%%%) enAble: sensor %d/%d with fresh reference - bsNo 0%o\n",bsn>>3,bsn&7,bsn);         
          SensorActive[bsn]=true;              //set active and grab new ref
          SensorActiveBlk[bsn>>3] |= mask[bsn&0x07];
          grab_ref(bsn,Sensor666,S666_pitch,long(bsn*S666_pitch),&Sensor_ref[bsn*S666_pitch],
                                                                &Sen_Brightness_Ref[bsn],&SensorRefRatio[bsn*12]);
                                   //includes new Cratios & brightness                                                                
           //NOTE WELL: without new frames, any new reference won't be much use with new coordinates!
/***/     printf("updated Sensor_ref[] \n");  //HEX bytes(0-5)(BGRBGR): %X %X %X %X %X %X \n",
        } break;
      }     
      case 'b':{ //b$; Block status byte output to host on USB (& i2c)  //b$# use # to change brightSF from 1 to 4     
        if (nParam<2){
          Serial.print("(b$#) Bank/BrightSF: parameters not found - brightSF(#): "); Serial.println(brightSF);
        } 
         else {
            if(nParam==3){ brightSF=p[2];
              Spr("(b$#) changing brightSF sensitivity to "); Serial.println(brightSF); 
            }  
          b=p[1]%10;                //ignore any further text after b$
          if(!newi2cCmd){
            Spr("(b$#)bank: ");Spr(b);Spr(" occupancy status byte: 0x");
            Spr(SensorBlockStat[b],HEX);Spr(" (sensors 7-0) ");
            for (i=7;i>=0;i--) Spr((SensorBlockStat[b]>>i) & 0x01); Serial.println(' ');
          }         
          else IFI2C {for (i=7;i>=0;i--) Spr((SensorBlockStat[b]>>i) & 0x01); Serial.println(' '); }        
        } 
        break;  
      }            //c$$$$#####; set $=1 to leave AWB AEC AGC & CB respectively. 
      case 'c':{   //c$$$$; Calibrate: turn on/off Auto adjustments for 10seconds and reload all references!
                   //6x % parameters are 1/0 for AWBg AEC AECd AEL AGC AGg respectively ( default to off(0) ) 
                   //full list: arguments for 'c' command; Bri Con Sat AWB  AWBg AEC AECd AEL AGC AGg (NO SPACES!)
        printf("(c$$$####$##) dangerous cam cmd! - USE CMD 'r00' after 15ses or just use 'j' ( $= _|-|0|1|2 )\n");  
                 
        Bri=0; Con=1; Sat=2; AWB=1; AWBg=1; AEC=0; AECd=0; AEL=1; AGC=1; AGg=9;    //reset defaults

        if(cmdString[1]<0x20){ 
         // printf("CALIBRATE CAM - syntax: c AWB AEC AGC CBar Bri Con Sat AGCgain AWBg  e.g. reset defaults: c01200129\n"); 
          printf("Calibrate CAM syntax: c Bri Con Sat AWB AWBg AEC AECd AEL AGC AGgain defaults: c0121100119\n");
          wait();
          break;
        }
        autoRefSuspended = true;                //Suspend auto refRefresh() during 'c'           
        cStr=" ";
        if (isDigit(cmdString[1])) Bri=cmdString[1]-0x30;          //options: use 0,1,2,'-' for -1 and '_' for -2  
        else {if (cmdString[1]=='-') Bri=-1; if (cmdString[1]=='_') Bri=-2;} 
        Spr(" Bri=");Spr(Bri); cStr=cStr+cmdString[1];                  
        if (isDigit(cmdString[2])) Con=cmdString[2]-0x30;
        else {if (cmdString[2]=='-') Con=-1; if (cmdString[2]=='_') Con=-2;}
        Spr(" Con=");Spr(Con); cStr=cStr+cmdString[2];
        if (isDigit(cmdString[3])) Sat=cmdString[3]-0x30; 
        else {if (cmdString[3]=='-') Sat=-1; if (cmdString[3]=='_') Sat=-2;}
        Spr(" Sat=");Spr(Sat); cStr=cStr+cmdString[3];       
        if (cmdString[4]=='1') {AWB=1; Spr(" AWB "); cStr=cStr+" AWB ";} else AWB=0;
        if (cmdString[5]!='\n') {
          if(cmdString[11]!='\n'){Spr(" missing arguments\n"); break;}
          if (isDigit(cmdString[5])){AWBg=cmdString[5]-0x30; Spr(" AWBg=");Spr(AWBg); cStr=cStr+cmdString[5];}
          if (cmdString[6]=='1') {AEC =1; Spr(" AEC "); cStr+=" AEC ";}else {AEC=0; cStr+=" ";}
          if (cmdString[7]=='1') {AECd=1; Spr(" AECd ");cStr+=" AECd ";}else AECd=0;                      
          if (isDigit(cmdString[8])) AEL=cmdString[8]-0x30;         //options: use 0,1,2,'-' for -1 and '_' for -2                 
          else {if (cmdString[8]=='-') AEL=-1; if (cmdString[8]>'_') AEL=-2;} 
          Spr(" AEL=");Spr(AEL); cStr=cStr+' '+cmdString[8];                                   
          if (cmdString[9]=='1') {AGC =1; Spr(" AGC "); cStr+=" AGC ";}else {AGC=0; cStr+=" ";}
          if (isDigit(cmdString[10])) {AGg=cmdString[10]-0x30;Spr(" AGCgain=");Spr(AGg); cStr=cStr+cmdString[10];}     
        }
        Serial.println(" "); 
               
        new_camera_settings(Bri,Con,Sat,AWB,AWBg,AEC,AECd,AEL,AGC,AGg);      //set AWB,AEC,AGC    select Auto ON initially 
     //   loop will do following call after 10 seconds and delay a further 10 before doing ref updates                 
     //   new_camera_settings(Bri,Con,Sat,AWB,AWBg,AEC,AECd,AEL,AGC,AGg);    //set AWB,AEC,AGC,ColourBar    select AWB OFF      
        Spr("Calibrate: run 10 seconds for auto calibrate before turning selected 'Auto' options off ");Serial.println(cStr);
        prefillRef=1;       //set to 1 should suppress normal loop data stream until end of stdCtimer
        getCAMstatus();     //also print in 'g' format
        Serial.println(" NOTE Calibration operation 'c' has NOT FULLY FOLLOWED THROUGH - assumes SensorStat[] & SensorBlockStat[] will automatically reset");
        printf("After a new line, calibrate will take ~20 seconds to execute and refresh references\n");
        wait();
        stdCtimer=millis()+10000;        //start a timer - wait 10sec for adjustments to settle automatically then turn off AGC say 
        break;              //leaves flags set for main loop() to calibrate from fresh images.        
      }
      case 'd':{      //d%%#; Differences between Sensor_ref[bsNo] and imagePtr. (optional n=No. of repetitions of image) (& i2c)         
        if(nParam==1) {   
          printf("(d%%%%#) Diff score for S%%%%. Optional repeat # times including frame samples \n ");
          wait();
          break;
        }
        dFlag=0; //set a (+ve) flag for loop() to print out data after it gets a fresh image repeating if desired.
        if(nParam==3) dFlag=p[2];           
        Spr("(d%%#) Diff: ");
        dbsNo = bsN(p[1]);   
        if (dbsNo>=0) {              
          printf(" difference score for Sensor[0%o], use %d consecutive images (remember pipelining!)\n",dbsNo,dFlag);
          printf("will print data AFTER new line - follow with 'w' command to pause to read\n");
        } else dFlag=-1;
        wait();      
        break;        //leaves flag for loop() to complete print output.       
      } 
      case 'e':{     //e;  EPROM burn any changes to Sensor[] & some other parameters
        printf("(e)EPROM: Save latest threshold(%d), min2flip(%d), nLED(%d), minSensors(0%o), maxSensors(0%o), Sensor[] SensorTwin[] & pvtThreshold[] to EPROM\n"\
               ,threshold,min2flip,nLED,minSensors,maxSensors);
        EEPROM.write(EPminSensors,byte(minSensors));
        EEPROM.write(EPnLED,byte(nLED));
        EEPROM.write(EPthreshold,byte(threshold));
        EEPROM.write(EPmin2flip,byte(min2flip));
        EEPROM.write(EPmaxSensors,byte(maxSensors));
        printf("EPROM: Saving latest sensor pointers to EPROM with thresholds, nLED, min2flip, min & maxSensors\n");
        for (i=0;i<80;i++) {            //put changed values for SensorTwin[] & Sensor[] in EEPROM
          EEPROM.writeLong(i*4,Sensor[i]);            
          EEPROM.write(EPSensorTwin+i,SensorTwin[i]);
          EEPROM.write(EPpvtThreshold+i,pvtThreshold[i]); 
          EEPROM.write(EPlineardX+i,lineardX[i]); 
        }
        EEPROM.commit();                //ESP32 method
        EEPROM.end();                   //burn eeprom   
        delay(500);   
        EEPROM.begin(EEPROM_SIZE);      //restart EEPROM for any further EEPROM commands (e.g. e or v).
        wait();
        break;            
      }  
      case 'f':{      //f%%;  Frame: image of bsNo sensor output to USB (if sensor[bsNo] undefined, will get no data)
        Spr("(f%%) ");
        bsn = bsN(p[1]);    
        printf("Frame: Print fb ref & newest sample for S%o\n",bsn);
        if (bsn>=0) write_img_sample(Sensor666,S666_row,bsn*48,24);  //write current image of sensor[bsn] ref & new data         
        wait();
        break;
      }
      case 'g':{        //get OV2640 camera status and print it to USB
        getCAMstatus();
        wait();
        break; 
      }
      case 'j':{        //j$#;  Set one camera parameter($) directly to digit(#)  (5=-1,6=-2)
        Serial.print("(j$#) adJust camera setting $ to # (NO auto new Ref's)\n");
        if(p[2]<10) p[2] += '0';   //convert int(0-2) back to char
        if(nParam==1) p[1]=012;    //LF to trigger 'j' menu
        new_camera_set( (char)p[1],(char)p[2]);           //if invalid $, print list of valid parameters 
        getCAMstatus();   // display new settings, as for (g) cmd
        wait();
        break;        
      }
      case 'h':{      //h$;  Help sets DEBUG flags to true (or false).  ALSO "h$%" set maxSensors to %x8 (010-090)                                       
        if (nParam>=2) {
          if (p[1]<9) {     //(h$) turns on debug of type $ 
            dbug=p[1];      //get level
            DEBUG[dbug]=true;  
            if (dbug==8) E6counter=200;  
            if ((dbug==7) && (nParam==3))  h7block=p[2];  
          }
          if(p[1]==9) {      //(h9) de-activates all debug
               dbug=9;                  //deselect any dbug (0-9)
               for(c=0;c<9;c++) DEBUG[c]=false;
               printf("turn off Help: DEBUG0(detail) off; DB1(timing) off; DB2(gen) off; DB3(i2c) off; DB4(noise) off; DB5 off; \n"); 
          }
          if((p[1]>9) && (p[1]<99)){   //set a maxSensors bsn limit on sensor state printout from 4min to 36max(044) (digit[2]x4)         
                maxSensors=bsN(p[1]);  
                printf("(h%% Setting maxSensors to 0%o\n",maxSensors);
          }  
        }else{    //just print info.
              printf("(h$) set debug $ level ON.  (h9) turns off levels 0 to 8.  (h%%%%) sets maxSensors (010-097)\n");            
              printf(" options: h0(detail) h1(timing) h2(gen) h3(i2c) h4(noise) h5(parz) h7,#(wait on bank # trip) h8(CS cmds)\n"); 
              printf("current help: ");     //if not h# or h9, print out current settings
              if(DEBUG[0]==1)printf("DE0(detail) ON; ");
              if(DEBUG[1]==1)printf("DB1(timing) ON; ");
              if(DEBUG[2]==1)printf("DB2(gen) ON; ");
              if(DEBUG[3]==1)printf("DB3(i2c) ON; ");
              if(DEBUG[4]==1)printf("DB4(noise) ON; ");
              if(DEBUG[5]==1)printf("DB5(parz) ON; \n");
              if(DEBUG[7]==1)printf("DBt ON; halt scroll on trip of bank %d \n",h7block);
              printf(" dbug=%d, maxSensors: 0%o (brightSF:%d)\n",dbug,maxSensors,brightSF); 
              wait();                 //stops looping indefinitely until input a newline (\n)       
        }
        wait();
        break; 
      }
      case 'i':{      //i%%;  Individual sensor status byte (1/0) sent to USB (& i2c)             
        if (nParam>1) bsn = bsN(p[1]); 
        else { printf("(i%%%%) Info: No Individual sensor specified\n"); break;
        } 
        if (nParam==3) {
          j=bsN(p[2]);
          SensorTwin[bsn]=j;         //i%%,$$
          printf("Setting SensorTwin[0%o",bsn);printf("] to sensor[0%o}\n",j);
        }
        printf("(i%%%%,$$) Info: Sensor %d/%d(%d) enabled:%d status:%d ",bsn>>3,bsn&7,bsn, SensorActive[bsn],SensorStat[bsn]);
        if (SensorStat[bsn])printf("true/OCCUPIED    ");             
          else              printf("false/unoccupied ");
        printf(" r=%3d x=%3d ",int(Sensor[bsn]/pitch),int((Sensor[bsn]%pitch)/2));
        if (SensorTwin[bsn]!=0)printf("Twin: S%o ",SensorTwin[bsn]);
        if (pvtThreshold[bsn]!=255){ printf("pvtThreshold= %d ",pvtThreshold[bsn]);
          if(pvtThreshold[bsn]>=128){
            int lstepR= pvtThreshold[bsn]&0x7F;   //linear "vertical"  length (4 spots) 0-63
            int lstepX = lineardX[bsn];          //linear "horizontal" length (-64 to 63 pixels)
          //  if (lstepX>7) lstepX = -(lstepX&7); //negative step -> 2's comp.
            printf("LINEAR steps: dR=%d dX=%d pixels",lstepR,lstepX);
          }
        }
        b=0;
        for(j=0;j<48;j++) b += Sensor666[bsn*48+j];
        printf(" brightness: A%d\n",b);
        wait();                     
        break;          
      }
     
      case 'k':{      //k%%,rrr,xxx  set coordinates for Sensor[%%] = pitch*r+2*x (for RGB565)
        bsn = bsN(p[1]);  
        if(nParam!=4) { printf("(k%%%%,rrr,xxx)Koordinates: setup for Sensor%%%%\n"); break; }      //invalid bsNo
        int rowVal=p[2];  
        if( rowVal<0 || rowVal>239) printf("invalid rowValue\n");
        else {
          int xVal=p[3];     
          if(xVal<0 || xVal>319) printf("invalid xValue\n");
          else{ 
            Sensor[bsn]=pitch*rowVal + 2*xVal; 
            printf("(k%%%%,rrr,xxx)setting new coordinates for Sensor %d/%d(%d) to r=%d x=%d\n",bsn>>3,bsn&7,bsn,rowVal,xVal);                    
          }
        }
        wait();
        break;       
      }     
      case 'l':{      //l%%;  (lima) set1: set bsNo OCCUPIED(1) & INACTIVE
        bsn = bsN(p[1]);     
        if(bsn>=0){ printf("(l%%%%)(Lima) set1: Sensor %d/%d (bsn/pin %d) set=1, OCCUPIED & disabled\n",bsn>>3,bsn&7,bsn);
          SensorStat[bsn]=true; 
          SensorBlockStat[bsn>>3] |=  mask[bsn&7]; 
          SensorActive[bsn]=false;
          SensorActiveBlk[bsn>>3] &= ~mask[bsn&0x07];           
        }else printf("(l%%%%)(Lima) set1: Sensor %%%% latch state = 1, OCCUPIED & disabled\n");
        break;
      }
      case 'o':{      //o%%;  (oscar) set0: bsNo set to UN-OCCUPIED(0) & INACTIVE(0)
        bsn = bsN(p[1]);      
        if(bsn>=0){ printf("(o%%%%)(Oscar)setO: Sensor %d/%d (bsn/pin %d) set=0, UN-OCCUPIED & disabled\n",bsn>>3,bsn&7,bsn);
          SensorStat[bsn]=false; 
          SensorBlockStat[bsn>>3] &= ~mask[bsn&0x07];
          SensorActive[bsn]=false;
          SensorActiveBlk[bsn>>3] &= ~mask[bsn&0x07]; 
        }else  printf("(o%%%%)(Oscar)set0: Sensor[%%%%] latch state = 0, UN-OCCUPIED & disabled\n");
        break;             
      }
      case 'm':{      //m$;    Minimum: sequential frames to trigger Occupied status (default 2)
                      //m$,%%; Optional set maxSensors to ## (decimal)
        if(nParam>1){   
          if(p[1]>0 && p[1]<9) min2flip = p[1];       //only allow 1-8        
        }
        if (nParam > 2){  
          if(p[2]>=100) minSensors=bsN(p[2]%100);   //fudge to let (m$,1%%) set minSensors - but can also use n$,%%
          else if(bsN(p[2])>minSensors) maxSensors=bsN(p[2]);
        }
        printf("(m$[,%%%%]) Min/Max: $ frames min2flip (trip) %d, maxSensors 0%o, minSensors 0%o, nLED %d, NLED %d, "
        "threshold %d, TWOIMAGE_MAXBS 0%o \n", min2flip, maxSensors, minSensors, nLED, NLED, threshold, TWOIMAGE_MAXBS);  
        wait();
        break;
      }
      case 'n':{      //n;  Number of block to assign to the programmable LED
        if(nParam>1){   //caution: setting nLED to the number used as NLED overrides/disables QLED in favour of PLED
          if(p[1]<10) nLED = p[1];
          digitalWrite(PLED,HIGH);          //initial setting (LED off) in case it never gets updated elsewhere.
        }
        if (nParam > 2){
          if((p[2]%100)<maxSensors) minSensors=bsN(p[2]%100); 
        }  
        printf("(n$[,%%%%]) bank Number:%d assigned to programmable status LED (minSensors=0%o)\n",nLED,minSensors); 
        wait();             
        break;  
      }
      case 'p':{      //p#;  Print table of Sensor[] Position Pointers (banks 0 to # only)
        i=0; c=0; 
        if((p[1]<10) && (p[1]>=0)) b=p[1]; else b=3;  //default value       
        printf("(p$) Position Pointers: TL corners for DEFINED Sensors to bank %d\n",b);
        for (i=i;i<=b;i++) {
          printf("Bank %d;", i);
          for (int j=c;j<=7;j++)
            if (Sensor[i*8+j]>0)printf(" s[%d]: r=%3d x=%3d",j,int(Sensor[i*8+j]/pitch),int((Sensor[i*8+j]%pitch)/2));              
          printf("\n");
        } 
        wait();
        break;
      }
      case 'q':{      //q$;  Query: block $ sensors. Show which are enabled - byte of 8 sensors (7-0) to USB & i2c 
        if((p[1]<=9) && (p[1]>=0)) b=p[1]; else b=1;  //default value 1       
        j=0;
        if(b==9) j=9;      //if "q9" send all blocks in reverse order (i.e. 9 first) else just do requested block       
        Spr("(q$) Query: bank ");Spr(b);Spr(" sensors (7-0) in enabled state: ");
        for (j=j;j>=0;j--){
            for (i=7;i>=0;i--)
              if(!SensorActive[b*8+i]) Spr('0');
              else Spr('1');                         
            Spr(" ");
            b=b-1;
        } 
        Serial.println(' '); 
        wait();
        break; 
      }
      case 'r':{      //(r%%[,0];  Refresh bsNo Reference in Sensor_ref[bsNo], Activate and may display img data
        printf("(r%%%%[,0]) Refresh Reference: for sensor[%%%%]. (default r = r00 for all) (MUST BE UNOCCUPIED!)\n");
        if(nParam==1) p[1]=0;       //r defaults to r00;
        bsn = bsN(p[1]);  
        if(p[2]==0) {               //requesting new ref's for block up to bsn
          force_refs=bsn;           //set the "flag" for refreshes without occupancy check          
          refRefresh(true);         //true suspend aborts current averaging
          rbsn=(force_refs & 0170); //move to start of requested block
          if(rbsn==0) rbsn=1;       //don't touch reserved S00        
          refRefresh(autoRefSuspended);      //false suspend starts new av of first sensor of block
        }else{                      //do normal 'r%%' cmd.
          if(Sensor[bsn]==0) { Spr("No action as Sensor(bsNo) undefined 0");Serial.println(bsn,OCT);break;}
          autoRefSuspended = true;                //Suspend auto RefReferesh() during 'r'
          if((bsn>0) && (Sensor[bsn]>0) ){ //if not 00 & defined, set enAble & refresh reference for 1 bsNo alone          
            SensorActive[bsn]=true;//grab_ref() gets image from Sensor666[] and computes new cRatios & brightness
            SensorActiveBlk[bsn>>3] |= mask[bsn&7];
            grab_ref(bsn,Sensor666,S666_pitch,long(bsn*S666_pitch),&Sensor_ref[bsn*S666_pitch],&Sen_Brightness_Ref[bsn],&SensorRefRatio[bsn*12]);  //includes new Cratios & brightness
            printf("References: enable new Sensor_ref[%d/%d]; 1st 2 HEX rgb pixels[0-5]: %X %X %X  %X %X %X\n",bsn>>3,bsn&7,Sensor_ref[bsn*S666_pitch],Sensor_ref[bsn*S666_pitch+1],Sensor_ref[bsn*S666_pitch+2],Sensor_ref[bsn*S666_pitch+3],Sensor_ref[bsn*S666_pitch+4],Sensor_ref[bsn*S666_pitch+5]);
            printf("full new ref[bsNo] from current frame only - average ref still to be calculated\n");
/***/       IFN write_img_sample(&Sensor666[bsn*48], S666_row, 0L, 12 );//print out ref & new img[bsNo] (1 frame)
            averageRbsn=bsn;        //set a flag for main loop to compute a multi-second (AVCOUNT=32) average Reference and update _ref[bsn]
            averageRcounter=AVCOUNT;//initiate down counter.  (10 per second)  If not 0, main loop will call averageRcalculation(averageRbsn,AVCOUNT);      
          }else{  //r00             //r00: refresh ALL defined sensors but does NOT modify current "active" state
            DOr00();
          }
        } 
        break;
      } 
      case 's':{      //s%%;  Scan for bright spot and set new xy for Sensor[bsNo]
        if(nParam>=2) bsn = bsN(p[1]);      //s0# or s%#; accepted as Sensor number (bsn=8*%+#). Second digit <=7
        if (nParam==1) { Spr("(s%%)Scan: place Sensor S%% at brightest point\n"); break; }   //do nothing if bad bsNo.      
        sScan=bsn;             //tell "loop()" to scan with fresh image for the bsNo in sScan. (0-79)
        sScanCount=3;          //request 3 fresh frames first to flush pipeline
        break;
      }     
      case 't':{      //t_:  Set threshold 31 to 250 for sensor "trip", send value to USB (& i2c)
        if(nParam==1) {printf("(t##[,%%%%]) Threshold: trip point set to %d \n", threshold); break;}
        int param1 = p[1];
        if(param1==00 && nParam==3) param1=99;   //t00,%% and t99,%% clears pvtThreshold[%%]
        if(param1 > 30) {                 //OK threshold
          bsn=00;
          if(nParam==3){ bsn=bsN(p[2]);  
            if(bsn >= 00) pvtThreshold[bsn]=byte(param1);   
            if(param1 == 99) {pvtThreshold[bsn]=255; param1=00;}      // clear pvtThreshold
            printf("(t##,%%%%) setting pvtThreshold to %d for sensor S%o \n",param1,bsn);
          }                    
          else threshold=param1;  // will be permanent only use 'e' command (along with sensor changes and nLED)
        }
        else {      //p[1] 1-30
          if(param1==1) {
              scroll=!scroll;  //use to hide scrolling status data.
              if(scroll) printf("scroll ON\n"); else printf("scroll OFF\n");
          }else {tcounter=param1+2;scroll=true; break;}     //start limited scroll looping
        }
        if(tcounter<2) printf("(t##) Threshold: trip point set to %d \n", threshold);
        wait();        
        break;
      }
      case 'u':{      //u%%; Undefine Sensor[%%] by setting pointer = 0
        if((nParam==1)||(p[1]>98))  printf("(u%%%%) undefine(erase) sensor[%%%%]\n");      
        else{
          bsn=bsN(p[1]);    
            printf("(u%%%%) Undefine: Undefining %d/%d  (bsn %d)\n",bsn/8,bsn&0x07,bsn);
            Sensor[bsn]=0L;
            SensorActive[bsn]=false;  
            SensorActiveBlk[bsn>>3] &= ~mask[bsn&7];    
            SensorTwin[bsn]=0;                   //erase any twin
        }
        if(p[1]==99){       //u99 undefines ALL sensors
          for (int i=0;i<80;i++) {Sensor[i]=0;SensorActive[i]=0;SensorActiveBlk[i>>3]=0;SensorTwin[bsn]=0;}
          printf("To clear Sensor pointers in EEPROM also, user should follow 'u99' with an 'e' cmd\n");
        }
        wait();            
        break;
      }
      case 'F':       //F;  Finish = Reset.  Added so Bluetooth phone & CS can send a reset ('R' not on remote)
      case 'R': {     //R;  Reset command reboots the CAM in Sensor mode and restores parameters from EPROM
        if(p[0]!='F'){                          //'F' forces Restart immediately (for EX-CS)
          Serial.print("(R)Reset: Press 'Enter' to confirm reset in Sensor Mode - 'R' or 'aw' will abort Reset & wait");
          while (!Serial.available() && !newi2cCmd) delay(10); //stops looping indefinitely until get a newline \n
          cmdChar=Serial.read();       
          if(cmdChar != '\n'){Serial.println("\nAborting Reset"); break;}  //any character can abort except LF
        }
        ESP.restart();  //or ESP.reboot()?
        break;
      }
      case '^': {
        printf("(^) CAM software version (BCDver): %d\n",BCDver); 
        wait(); 
        break;
      }
      case 'v':{      //v#;  reboot & select video webserver mode
        if(FLASHLED==2)printf("CAMERA_MODEL_WROVER_KIT  ");
        printf("CAM:0x%x\n",I2C_DEV_ADDR); //"CAM:0x%x command: v p[1]: %d\n",I2C_DEV_ADDR,p[1]);               
        if((p[1]<1)||(p[1]>2)) {printf("(v#) CAM software version (BCDver): %d\n",BCDver); wait(); break;}
        int wifi=p[1];     //identify requested wifi router (1 or 2)
        printf("(v[1|2])Video: About to restart CAM (in video(jpeg) mode for webserver via wifi %d.\n",wifi);
        printf("Waiting - press 'Enter' to trigger reset\n");
        while (!Serial.available() && !newi2cCmd) delay(10); //stops looping indefinitely until input a newline NL
          //need to set EPROM flag in eprom
          // should we do another EEPROM.begin in case EEPROM.end already done??
        EEPROM.write(EPvFlag,wifi);   //write a flag to force webserver after reboot. (default wifi=1)
        EEPROM.write(EPnLED,byte(nLED)); //saves n, t, m parameters also. WARNING will loose other unsaved updates
        EEPROM.write(EPthreshold,byte(threshold));
        EEPROM.write(EPmin2flip,byte(min2flip));
        EEPROM.write(EPmaxSensors,byte(maxSensors));    
        EEPROM.commit();   //burn eeprom
        EEPROM.end();     //Note: If 'e' has been used since last reboot, this will fail so may have to repeat 'v' 
        delay(1000);     //Time to burn?                
        ESP.restart();  //or ESP.reboot()?
                       //need to reset flag immediately webserver mode starts, or will lock into webserver mode!
        break;
      } 
      case 'w':{      //w;  Wait: stops looping until a new line is entered (so can stop screen scrolling)
        wait();
        break;        
      }
      case 'x': {      //set a starting column in image for USB transmission to monitor
        printf("(x###) Xcolumn: current= %d ", Xcolumn);
        if(nParam==2) Xcolumn=p[1] & -2;       //make even 
        if(Xcolumn > 319){Xcolumn=0;printf("New val exceeds 320 -> default = 000\n");}
        if(Xcolumn+Zlength>320) {
          Zlength=320-Xcolumn; 
          printf("reset Zlength=%d ",Zlength);
        }
        printf("new Xcolumn for sample image= %d \n",Xcolumn);
        
        wait();
        break;
      }
      case 'y': {        //select an image row and send packet to USB     
        if(nParam==1) {SendYpacket=true; break;} //'y' alone should repeat last y e.g. for crc error retry
        if(cmdString[1]=='y'){Yhold=false; break;} //'yy' ends image transfer. N.B. CAN'T USE p[1] ! 'y121'
        Yrow=p[1];       //'y?\n' would default to Yrow = 0 if not digit
        if(Yrow>239){Spr("ERROR row exceeds limit\n");break;}
        if(!Yhold) {                            //first 'y' command
          Yheader[1]=byte(Yrow);                //prepare header
          Yheader[3]=byte(Xcolumn/2);            
          Yheader[5]=byte(Zlength/2);           //Num. bytes sent = 4 x Zlength/2 + header
          Serial.println("frame stays frozen until cleared with 'yy' command :::\n"); 
          boxSensors=true;                      //need to add fresh sensor boxes to image
        }
        SendYpacket=true;     //send data after next frame
        Yhold=true;          //freeze frame until next "yy" cmd    
        break;               //send immediately 
      }
      case 'z': {      //set a packet length for image transmission
        printf("(z###) z: length of old packet = %d pixels. ", Zlength);
        if(nParam==2) Zlength=p[1] & -2;         //make even
        if((nParam==2)&&(Zlength<2)) Zlength=320;
        if(Zlength > 320){
          Zlength=80;
          printf(" z### exceeds 320 -> default = 80\n");
        }
        if((Zlength+Xcolumn)>320) { 
          Zlength=320-Xcolumn;
          printf("reset Zlength=%d ",Zlength);
        }
        printf("New Zlength for USB image: %d pixels\n",Zlength);
        wait();
        break;
      }
      case '\\':       //create a linear sensor bank sloping down right.    
      case '/':{       //create a linear sensor bank sloping down left 
        int signX=1; int signR=1;  //step size of 2x and r pixels
        int bsn2; int rows=0; int dx=0; int show=1;
        bool newSen=true;
        if(nParam==1) {printf("(\\%%%%[,dR,dX[,#])create linear sensor(s)"); break;}
        if(nParam==2) newSen=false;  
        if(nParam==3) {  //convert to 5 format
          if(p[0]=='/') p[0]='\\';      // '/' would reverse slope
          bsn=bsN(p[1]);                //get %% -Needs better check on range
          bsn2=bsN(p[2]);
          if (bsn2<=bsn){ printf("second b/s must exceed first. 0%o",p[2]); break; }
          signR = updn( bsn, bsn2, &rows, &dx);    //calculate dr and dx
/***/ //    printf("sign:%d updn bsn:%d bsn2:%d rows:%d dx:%d",signR,bsn,bsn2,rows,dx );
          p[2] = rows/(bsn2-bsn);       //max -ve rows of -63 acceptable
          p[3] = dx/(bsn2-bsn);      
          p[4] = (bsn2-bsn+1); 
          nParam=5;  
          printf("\\ %d %d %d %d Sen: %ld %ld rows: %d\n",p[1],p[2],p[3],p[4],Sensor[bsn2],Sensor[bsn],rows);
        }      
        if(nParam==5) {} //limit steps    
        if(nParam>=4)if((p[2]>63) || (p[3])>63) {printf("step out of range %d,%d \n",p[2],p[3]);return;} 
        if (p[0] == '/')signX = -1;    //reverse signX 
        printf("(\\%%%%[,#,$]) linear sensor creation starting with S%%%%. use increments: #rows,$dx pixels\n");                                                                   
        bsn=bsN(p[1]);                //if p[1]>99, get 00       
             
        for(i=0;i<(8-bsn%8);i++){     //do i sensors to end of bank 
          if(!newSen) {      //don't create sensors, calculate steps instead
            if(Sensor[bsn+i+1]==0) {printf("Missing/undefined Sensor in bank. S%o\n",bsn+i+1); return;}
            signR = updn( bsn+i, bsn+i+1, &rows, &dx); 
            if(signR<0)  {printf("sensor line not descending S%o\n",bsn+i); return;}
            if(rows > 31){printf("row gap too big S%o\n",bsn+i); return;}
            p[2] = rows;       //max -ve step is -63
            p[3] = dx;                 
/***/       printf("(0%o) nParam:%d %c %d %d %d\n",bsn+i,nParam,p[0],(char)p[1],p[2],p[3]);      //  i=bsn%8;              
          }
          if(i==p[4]-1) newSen=false;     //stop when p[4] reached.
          if(!makeLinear(bsn+i,p[2],signX*p[3],newSen,signR)) return;     //abort on error
          show=bsn|7;
          if(i==p[4]-1) {show=bsn+i; break;}       //truncate line 
          if(!newSen) break;
        }  
        if(nParam==2) show=bsn; 
        for(i=bsn;i<=show;i++){    //print out results for verification
          printf("sensor S%o ",i);
          printf(" r=%3d x=%3d ",int(Sensor[i]/pitch),int((Sensor[i]%pitch)/2));
          if (SensorTwin[i]!=0)printf(" Twin = 0%o ",SensorTwin[i]);
          if (pvtThreshold[i]!=255){ printf(" pvtThreshold= 128+%d deltaX= %d",pvtThreshold[i]&0x7F,lineardX[i]);		 
          printf(" \n");
          }
        } 
        printf("completed linear sensor\n");
        wait();
        break;
      }
      case '@':{      //@##;  set OCc character to decimal ASCII value
        OCc=char(p[1] & 0x7F);   // DO NOT USE char(12) with PROCESSING4
        if(nParam==1) OCc='#';
        printf("(@##) 'Occupied' indicator in tabulation set to ASCII 0x%x\n",OCc);
        break; 
      }
      case '&':{      //statistics
		    int hsum=0;
        printf("(&) print Histogram of Hi noise trips since last '&'.  i.e. in %d loops.\nbsNo\t",HistoLoops);
        for (bsn=0;bsn<maxSensors;bsn++) if(SensorActive[bsn]) printf("%o\t",bsn); printf("SUM");
        for (i=1;i<5;i++) { 
          printf("\n%d Hi\t",i); 
          for(bsn=0;bsn<maxSensors;bsn++) 
            if(SensorActive[bsn]) {hsum+=SensorHisto[bsn*5+i]; printf("%d\t",SensorHisto[bsn*5+i]);} 
          printf("=%d",hsum);
          hsum=0;
        }
        printf("\nTRIP\t"); 
        for(bsn=0; bsn<maxSensors;bsn++) 
          if(SensorActive[bsn]) {hsum+=SensorHisto[bsn*5+0];printf("%d\t",SensorHisto[bsn*5+0]);} 
        printf("=%d\n",hsum);
        for (bsn=0; bsn<maxSensors*5+5;bsn++) SensorHisto[bsn]=0;   //reset all (5*80) counters
        for (bsn=0; bsn<80;bsn++) SensorHiCount[bsn]=0;       
        HistoLoops=0;
        wait(); 
        break;
      }	    
      case '+':{      //+#,$  Add/move all sensor alignments +/-# pixels in direction $(0-7)*45deg.
        if((nParam==1) || (p[1]>'3')) { Serial.print("(+#,$) invalid realign cmd\n"); break; } 
        if( nParam==3 && p[2]<8) {
          param2=p[2];  
          char dirn='0' + p[2]; 
          if (strchr("123",dirn) != nullptr) c=p[1];  //column
          if (strchr("345",dirn) != nullptr) b=p[1];  //row
          if (strchr("567",dirn) != nullptr) c=-p[1];
          if (strchr("701",dirn) != nullptr) b=-p[1];
          for (bsn=0;bsn<80;bsn++){
            if(Sensor[bsn]!=0) {    //bsn defined
              Sensor[bsn] = Sensor[bsn]+b*pitch + c*2;
              if(bsn==0) printf("(+#,$) move all alignment +/-%d pixels (direction %d). New S00 at r=%d x=%d\n", \
                            p[1],param2,int(Sensor[bsn]/pitch),int((Sensor[bsn]%pitch)/2));  
            }             
          }
          printf("Use 'r' to re-reference.  To make permanent, use 'e' to write to EPROM\n");    
        }else  Serial.print("(+#,$) invalid realign cmd\n");
        wait();
        break; 
      }  
    }       //end switch()
    newi2cCmd=false;        // be aware that this could clobber a fast follow up i2c command already received. 
/***/IFI2C Spr(" i2cCF ");   
}           //end processCmd()
   // *****************************

// FUNCTION TO CALCULATE LINE LENGTHS
int updn(int bsn1,int bsn2, int *abs_dr, int *dx) {
   // bsn1: first sensor bsn
   // bsn2: another sensor bsn
   // abs_dr: no of rows between (positive)
   // dx: number of pixels offset along x axis. (+/-)
    int dr=(int(Sensor[bsn2])/FBPITCH - int(Sensor[bsn1])/FBPITCH);         
    *abs_dr=abs(dr);
    *dx = ((int(Sensor[bsn2])-int(Sensor[bsn1])) - dr*FBPITCH)/2;
/***/ //   printf("bsn1:%d bsn2:%d dr:%d abs_dr:%d dx:%d \n",bsn1,bsn2,dr,*abs_dr,*dx);
    if(dr>=0)   return +1;
    else return -1;
}    // *****************************
   
// FUNCTION TO SET/CHANGE ONE CAMERA SETTING    
void new_camera_set(char C1,char C2){  //Bri,Con,Sat,AWB,AWBg,AEC,AECd,AEL,AGC,AGg
    sensor_t * s = esp_camera_sensor_get();
    if(C1<0x20) {
      printf("j$# SET OV2640 options for parameter $:\n B: BRIGHTNESS (_,-,0,1,2)\n C: CONTRAST   (_,-,0,1,2)\n");   
      printf(" S: SATURATION (_,-,0,1,2)   \n W: AWB      (0,1) 1 = enable\n w: awb_gain (0,1) 1 = enable\n");      
      printf(" E: AEC      (0,1) 1 = enable\n e: aec_level  (_,-,0,1,2)   \n F: aecd     (0,1) 1 = enable\n");       
      printf(" G: AGC      (0,1) 1 = enable\n g: AG_gain  (0-9)           \n b: bpc      (0,1) 1 = enable\n");
      printf(" P: wpc      (0,1) 1 = enable\n R: aw_gma   (0,1) 1 = enable\n L: Lenc     (0,1) 1 = enable\n");    
    }else{     
      int n=0;                          //default 
      if (isDigit(C2)) n=C2-0x30;   //valid: 1 to 9, -(-1), _ (-2)
      switch (C1) {     
        case 'g':               //agc_gain (0-9)
            s->set_agc_gain(s, n);      // 0 to 30 (0-9)(AGg)
            n=10;                       // flag as done OK
            break;
        default:                //fudge to enable -2 to +2 values if needed
            if (C2=='-') n=-1;  
            if (C2=='_') n=-2; 
      }  
      switch (C1) {
        case 'B':               //BRIGHTNESS
            s->set_brightness(s, n);    // -2 to 2
            n=10; break;
        case 'C':               //CONTRAST
            s->set_contrast(s, n);      // -2 to 2
            n=10; break;
        case 'S':               //SATURATION
            s->set_saturation(s, n);    // -2 to 2
            n=10; break;
        case 'e':               //aec_level
            s->set_ae_level(s, n);      // -2 to 2
            n=10; break;
        default:                //clear fudged negative values 
            if (n<0) n=0;       //invalid values default to 0
      }
      if(n<2)                     // only accept 0 & 1 for these parameters
        switch (C1) {
          case 'W':               //AWB
            s->set_whitebal(s, n);break;       //0 = disable , 1 = enable 
          case 'w':               //awb_gain
            s->set_awb_gain(s, n);break;      // 0 = disable , 1 = enable
          case 'b':               //BPC
            s->set_bpc(s, n);break;           // 0 = disable , 1 = enable
          case 'E':               //AEC 
            s->set_exposure_ctrl(s, n);break; // 0 = disable , 1 = enable
          case 'F':               //aec2
            s->set_aec2(s, n); break;         // 0 = disable , 1 = enable
          case 'G':               //AGC
            s->set_gain_ctrl(s, n); break;    // 0 = disable , 1 = enable;        
          case 'L':               //LENC
            s->set_lenc(s, n); break;         // 0 = disable , 1 = enable
          case 'R':               //raw_gma
            s->set_raw_gma(s, n);break;       // 0 = disable , 1 = enable
          case 'P':               //WPC
            s->set_wpc(s, n);break;           // 0 = disable , 1 = enable   
          default:
            Spr("Unrecognised parameter\n");          
        }
      else if(n<10)Spr("invalid 'j' parameters\n");
    }
 }      // ****************************** 
   
// FUNCTION TO SET/CHANGE MULTIPLE CAMERA SETTINGS      new_camera_settings(0,1,2,AWB AWBg AEC AECd AEL AGC AGg); 
void new_camera_settings(int Bri,int Con,int Sat,int AWB,int AWBg,int AEC,int AECd,int AEL,int AGC,int Agg){
  int CB=0;                    //colour bar
  printf("\nNew_camera_settings() invoked - allow 10sec to stabilise\n");
  sensor_t * s = esp_camera_sensor_get();
  if(abs(Bri>2)) Bri=0;        // reset defaults!
  if(abs(Con>2)) Con=1;
  if(abs(Sat>2)) Sat=2; 
//adjust where necessary
  s->set_brightness(s, Bri);    // -2 to 2
  s->set_contrast(s, Con);      // -2 to 2
  s->set_saturation(s, Sat);    // -2 to 2
  s->set_special_effect(s, 0);  // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, AWB);      // 0 = disable , 1 = enable
  s->set_awb_gain(s, AWBg);     // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);         // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, AEC); // 0 = disable , 1 = enable
  s->set_aec2(s, AECd);         // 0 = disable , 1 = enable
  s->set_ae_level(s, AEL);      // -2 to 2
  s->set_aec_value(s, 300);     // 0 to 1200
  s->set_gain_ctrl(s, AGC);     // 0 = disable , 1 = enable
  s->set_agc_gain(s, Agg);      // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 0);             // 0 = disable , 1 = enable
  s->set_wpc(s, 1);             // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);         // 0 = disable , 1 = enable
  s->set_lenc(s, 1);            // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);         // 0 = disable , 1 = enable
  s->set_vflip(s, 0);           // 0 = disable , 1 = enable
  s->set_dcw(s, 1);             // 0 = disable , 1 = enable
  s->set_colorbar(s, CB);       // 0 = disable , 1 = enable    //colour bar may be a good debug option 8 vertical stripes (only works for jpeg???) 
}      // ***************************** 
   
//  DISPLAY A SAMPLE OF BMP FRAME BUFFER 
void disp_sample_pixels(byte *imagePtr,int pitch,int row,int dispX,int nBytes){
  //function can handle selected Sensor image (Sensor666) provided appropriate parameters are used (e.g. as follows)
  //if using Sensor666 use ( Sensor666, S666_row, 4*bsNo,    dispX,    36 ) 36=3*bsNos, dispX/16 will move multiples of bsNo
  //imagePtr points to first byte of image (e.g. top left corner)
  //pitch is "width" in bytes to start of next row
  //row and dispX are desired (r x) integer offsets (in pixel count) to image to print
  //nBytes is number of data bytes per line of output (use multiple of 12)
    long offset=0;
    offset=long(row)*long(pitch)+long(dispX*3);
    write_img_sample(imagePtr,pitch,offset,nBytes );
}    // *****************************

// FUNCTION TO PRINT A 12 x 6 pixel sample from (RGB565) fb (or ref666[] decoded selection)
//   write_img_sample(Sensor666,S666_row,long(dbsNo*48),24);
void write_img_sample(byte *imagePtr,int pitch,long offset,int nBytes ){ 
  //function can handle selected Sensor image (Sensor666) provided appropriate parameters used (e.g. as follows)
  //if using Sensor666 use (Sensor666, S666_row, 48*bsNo, 24 ) 24=2*bsNos, 48*bsNo offset for (bsNo) from sen(00)
  //prints out 4 rows of nBytes of pixels at sensor[0] position AND 
  //the rest at offset position buffer imagePtr from offset 
  byte *k;
  byte *z;
   //  int row = int(offset/pitch);
   //  int xposn=int(offset-long(row*pitch));
   // printf("*(imagePtr+2) %x Sensor[] sample row %d byte %d pitch %d\n",*(imagePtr+2),row,xposn,pitch);
    k=imagePtr;                         //top left pix of S[00]  
    for (int r=0;r<4;r++){              //print out 4 rows for rgb565
      k=&Sensor_ref[00] + offset;       //set left image to the ref[bsn] image
      z=k+long(r*pitch);                //set z to 1st pix for ref S[bsn]
        for(int i=0;i<nBytes;i+=1) {    //print sensor[0] --- then offset sensor
        if (i==12){                     //after bsNo 0 (4pixels wide) skip to offset (dispX?)
          k=imagePtr;                   //set k to actual image 
          z=k+offset+long(r*pitch);     //set z to 1st pix of S(bsn]
          Spr(" offsetBytes ");Spr(int(offset));        //print pixels 0-17 THEN offset pixels
          Spr(" bsNo=");Spr(offset/48/8);Spr("/");Spr((offset/48)%8);Spr(" -> ");
        }
        if(i==36) Spr(".");    //end of offset sensor
        if(*z<0x10)Spr("0");
        Spr(*z,HEX); Spr(" ");
        if((i%3)==2)Spr(" ");  //add space between pixels
        z=z+1;                      //set pointer z to next pixel to print
      }
      Serial.println(""); 
    }
}    // ***************************** 
 
// FUNCTION TO FIND x & y FOR NEW BRIGHT LED SENSOR POSITION
long scan( byte *imageData, int pitch, int width, int height, int fmt) {
  //assumes fmt bytes per pixel and first pixel is xy=0,0    x is position along row/line & y is row number
int  xMax = 0;                 //short (16 bit) global variable
int  yMax = 0;                 //short (16 bit) global variable
long z=0;                      //offset to brightest pixel (in bytes)
byte pixel[3];
char zStr[12];
int  x=0; 
int  y=0;
short int brightness = 0;
short int brightnessMax = 0;

  for (y = 0; y<height; y++) {  
    for (x = 0; x < width ; x += 3) {  
      decode565(imageData,x*2, pixel);
       brightness = pixel[0] + pixel[1] + pixel[2];   //6 bit values give max <189 (63*3)
        if (brightness > brightnessMax) {
          brightnessMax = brightness;
          xMax = x;
          yMax = y;
        }
    }
    imageData += pitch;           //Shift pointer forward a row
  }
  z=long(xMax) * long(fmt) + long(yMax) * long(pitch);
  ltoa(z,zStr,10);
  printf("yMax %d xMax %d index %lu brightnessMax %d \n",yMax,xMax,z,brightnessMax);
  return z;                         //returns offset(bytes). z=0 if fmt not 2 or 3.
}   // *****************************

//  FUNCTION TO SAVE SENSOR REFERENCE IMAGE AND COMPUTE REF BRIGHTNESS AND CRATIOS
/*   grab_ref(bsn,Sensor666,S666_pitch,long(bsNo*S666_pitch),&Sensor_ref[bsNo*S666_pitch],&Sen_Brightness_Ref[bsNo],&SensorRefRatio[bsNo*12]); */ 
void grab_ref(int bsn, byte *iD, int pitch, long SensorPtr, byte *Ref, int *bright, unsigned int *Cratios) {     //includes new Cratios 
  //iD parameter must be buffer of new image data to grab from. e.g. &Sensor[bsn*S666_pitch]
  //pitch is number of image bytes to grab for one Sensor e.g. S666_pitch
  //SensorPtr is offset bytes to required start of data (relative to iD[0]
  //Ref destination must be &Sensor_ref[bsNo*S666_pitch] similarly &bright[bsNo]
  //Cratios is array of (4*3) colour ratios (r/g g/b b/r) e.g. &SensorRefRatio[bsn*12]            
      bright[0]=0;
      iD=iD+SensorPtr;  //sens points to first byte of sensor (first colour of top left pixel)
      for (int i=0;i<pitch;i++) {  
/***/   IFN {Spr("+"); Spr(int(iD[i]));} 
        Ref[i]=iD[i];          // for RGB565, just need to copy sensor from Sensor666 to Sensor_Ref      
        bright[0] += iD[i];    // calculate new brightness value (sum of all sensor pixel colours)
      } 
/***/ IFN {Spr(" bright:");Serial.println(bright[0]);}
      Compute_CRatios(iD, S666_row, 0L, Cratios, 2);   //iD has been advanced with iD=iD+SensorPtr; above
      bsnUpdated=bsn;
/***/ IFN  {Spr("bsn:");Spr(bsn,OCT);Spr(" Computed new Cratios, and brightness of ");Serial.println(bright[0]);}
/***/ IFN   Spr(".......");
}   // ***************************** 
  
// FUNCTION TO COMPARE SENSOR REF WITH LIVE IMAGE DATA
/*e.g. compare( Sensor666, S666_row, long(bsn*S666_pitch), &SensorRefRatio[bsn*12], bsn, RGB565p);  //compare current S666 frame*/
int compare(byte *iD, int pitch, long SensorPtr, unsigned int *Ref, int bsNo, int sum){
  // iD is pointer to base of imageData (code only handles RGB565 format)
  // SensorPtr is the OFFSET (e.g. from Sensor[bsNo]) into the iD array
  // Ref is pointer to 80 RefRatio Cratio sets for Sensor reference image
  // bsNo only used for debug printout not used by compare()
  // sum  (not used)
  int maxDiff=0;
  int i;
  int j;
  unsigned int Cratio[12]={0,0,0,0,0,0,0,0,0,0,0,0};    //128 x colour ratios (Equal if =128) 
  //(quad average Red/av. Blue etc.)
  //compute colour ratios for 3 colours in 4 quadrant from image and put in Cratio[0-11]
      Compute_CRatios(iD,pitch,SensorPtr,&Cratio[0],RGB565p);   //ratios for current image placed in Cratio[12]
/***/ IF0 if(bsNo<13){
/***/   Spr("\nRefRatio(");Spr(bsNo,OCT);Spr("):");for (i=0;i<12;i++){Spr(Ref[i]);Spr(" ");} Serial.println(' ');
/***/   Spr("Cratio(");Spr(bsNo,OCT);Spr("):  ");for(i=0;i<12;i++){Spr(Cratio[i]);Spr(" ");}Serial.println(' ');
/***/ }
      for (i=0;i<3;i++){   //convert Cratios to Xratios - do for each colour //max XRatio[] is 2016 & min is 32
        for (j=0;j<12;j+=3){  //do for each quadrant (have 3 colour ratios per quadrant)
          if(Cratio[i+j]>Ref[i+j]) Cratio[i+j]=(Cratio[i+j]<<5)/(Ref[i+j]);//range 32 to (4*63*32/4) (2016 max)
          else                     Cratio[i+j]=(Ref[i+j]<<5)/(Cratio[i+j]);//max cross ratio = 252*32/4 32-2016 
		   
        }
      }
      maxDiff=0;              //find max Xratios in Cratio
      for (i=1;i<12;i++) if(Cratio[i]>maxDiff) maxDiff=Cratio[i];   //range 32 to 18360/4 (4590 max)       
          //cross ratios should range from 32 to 2016
/***/ IF0 {Spr("Xratios(");Spr(bsNo,OCT);Spr("): ");for (int i=0;i<12;i++){Spr(Cratio[i]);Spr(" ");}
/***/                                        if(maxDiff>threshold)Spr("Hi maxDiff ");Serial.println(maxDiff);}
      return maxDiff;       //from 32 to 2016 //>>3; could scale back to give 4 to 255 range for max(Cratio/32)
          // end of rgb565 colour ratio comparison       
}    // *****************************    
      
/*  Compute_CRatios(Sensor666, S666_row, long(bsn*S666_pitch), &SensorRefRatio[bsn*12], RGB565p);  */     
//COMPUTE COLOUR RATIOS AV. RED TO GREEN, G TO B & B to R & PUT IN CRatio[] AND brightness sums in quad[4]
void Compute_CRatios(byte *iD,int pitch, long SensorPtr,unsigned int *Cratio, int bpp){
     //iD is the pointer to start of image array
   
unsigned int sum[12]={0,0,0,0,0,0,0,0,0,0,0,0};   //sum of R G & B for 4 quadrants (max value = 4 x 63 =252)
int sm;                                           // (for VGA get max value = 9*255*3 =6885)
int i;
int j;  
    iD=iD+SensorPtr;        //sets points to first byte of sensor (top left pixel)
    if(bpp==2){
/***/ IF0  printf("Comp_Cratios: SensorPtr: %lu 1st colour = %x %x %x %x\n",SensorPtr,iD[0],iD[3],iD[12],iD[15]);
      for (int q=0;q<4;q+=2){   //do twice for two pairs of quadrants
        quad[q]=0;quad[q+1]=0;     
        j=q*3;  //do sums for 3 colours x 2 quadrants  //get sum of 9 pixlets in 2 quadrants ->max 63*4=252/quad
         //FOR 1ST(left) QUADRANT CALCULATE COLOUR SUMS using 8 decoded rgb565 pixels in iD[0-23] (two quadrants)      
        for (int c=0;c<3;c++){                          //do for 3 colours
          sm = iD[c]+iD[c+3]+iD[c+12]+iD[c+15];         //sum one colour in 4 pixels spread over 2 rows    
          sum[j+c] = sm;        //colour sum for quadrant
          quad[q] += sm;        //brightness sum for quad
/***/     IFD  if(SensorPtr<(48*4)) printf("q=%d c=%d pix: %d %d %d %d Sensor sum= %d quad= %d\n",
/***/                               q,c,iD[c],iD[c+3],iD[c+12],iD[c+15],sum[j+c],quad[q]);       //ie bsNo=0->2 
        }
                //REPEAT FOR 2ND (right) QUADRANT                
        for (int c=0;c<3;c++){
            sm = iD[c+6]+iD[c+9]+iD[c+18]+iD[c+21];
            sum[j+c+3] = sm;     //colour sum for RH quad
            quad[q+1] += sm;     //brightness sum for second quad   //max=63*4*3=756
        }        
        iD=iD+2*pitch;           //sets points to next quadrnt rows of sensor
      }
      for (i=0;i<12;i++) if (sum[i]<4) sum[i]=4;    //ensure never divide by zero (or less than 4 Cratio fits 
	                                                // within (signed)integer maxDiff)
      for (i=0;i<12;i++){       //for 3 colours and 4 quadrants compute ratios of R/G G/B & B/R -> 12 +ve ratios
        j=i+1;
        if((j%3)==0) j=j-3;        //j=next RGB colour R->G, G->B, B->R   
        if (sum[i]>sum[j]) Cratio[i]=(sum[i]<<5)/(sum[j]); //divide largest by smallest to get positive ratio (32-2016)
        else      Cratio[i]=(sum[j]<<5)/(sum[i]);    // multipy by 32 and integer divide by (refsum) ref for ratio new/old
      }       //Cratio ranges from 32 to 4x63*2^5=252*2^5 / 4 (ie 65280/32 = 2016 max down to 32 if equal)
      quad[4]=quad[0]+quad[1]+quad[2]+quad[3];
/***/ IFD printf("quad[0->3]: %d %d %d %d %d\n",quad[0],quad[1],quad[2],quad[3],quad[4]); 
    } 
}      // ******************************

// GET bsNo FROM cmdString[] CHECKING VALIDITY
int get_bsNo(char *cmdString){        
        //cmdString looks like "c##xxx\n" - only ## used
        //returns integer = # * 8 + #     //(treats %% like octal)
int bsn;
    if(isDigit(cmdString[1])) {bsn=int(cmdString[1]-0x30);     //return -1 if not valid 2 digit no. 
      if(isDigit(cmdString[2])&&(cmdString[2]<=0x37)) {      //check cmdString[2] <= 7
        bsn=bsn*8+int(cmdString[2])-0x30;                  //2nd digit <8
        return bsn;
      }
    }
    Serial.print(" invalid bsNo\n");
    return -1;
}      // *****************************
  
// GET DECIMAL NUMBER FROM cmdString[].  Max 99999
long get_number(char *){
        //cmdString looks like "c####xx\n" - only 1-5 #'s used
     int number=0;   
     for (int i=1;i<6;i++){           //accept up to 99999
       if( isDigit(cmdString[i]) )
         number=number*10+int(cmdString[i])-0x30;
       else return number;
     }
     return number;
}      // *****************************
  
//  DECODE RGB565 INTO 3 BYTES RGB666(@6BIT) FROM 2 BYTES
void decode565(byte *iD,long offset, byte *rgb666){
        //iD contains 2 byte pixel of 5+6+5 bytes which is separated and placed in 3 (6bit) bytes in rgb666[0-2]
     iD=iD+offset;  //sens points to first byte of sensor (top left pixel)
     rgb666[2]=(iD[1]<<1)&0x3E;                    //Blue
     rgb666[1]=((iD[1]>>5)|(iD[0]<<3))&0x3F;       //Green
     rgb666[0]=(iD[0]>>2)&0x3E;                    //Red  
}       // ***************************** 

//  DECODE RGB565 FRAME INTO COMPACT SENSOR FRAME OF RGB666 FORM
bool f565to666( byte *imageData,int pitch, unsigned long *sensor, byte *f666, bool *SensorActive){
      //imageData is pointer to full RGB565 frame at 2 bytes/pixel (fb)
      //pitch is 2*width of imageData (QVGA 2*320)
      //sensor[] contains 80 pointers into RGB565 frame
      //f666 is the destination (rgb666 3 byte format)) for extracted sensor images (80x4x4x3 bytes)
      //SensorActive[] is true/false (only needed if intend to skip inactive sensors)
byte *iPtr0;     //image point of sensor in frame buffer image (Top Left pixel)
byte *iPtr;      //start of an image row (LHS)
int sn;    //sensor number being extracted
int row;     //row count in std. sensor
int quad;   //sensor quadrant (0-3)
int i;     //pixel count 
int j;     //destination offset 
   for(sn=0;sn<80;sn++){    //decode all 80 Sensor images  
     if(SensorActive[sn]){ //for speed, only active sensors decoded. This function only takes <400uSec for 80
       iPtr0=imageData+Sensor[sn];   //point to start of sensor
       if((pvtThreshold[sn]<0x80)||(pvtThreshold[sn]==255)){  //not linear sensor
         for (row=0;row<4;row++) {           //do for four consecutive image lines/rows
           if(row<2) iPtr=iPtr0+row*pitch;         //point to LHS of sensor
           else    iPtr=iPtr0+(row+SEN_SIZE)*pitch;
           j=sn*S666_pitch+row*S666_row; //destination offset (e.g. sn*48+row*12)
           for (i=0;i<4;i++){          //decode 4 pixels per sensor row
             if(i<2) decode565(iPtr,long(i*2),&f666[j+i*3]);
             else    decode565(iPtr,long((i+SEN_SIZE)*2),&f666[j+i*3]);
           }
         }
       }else {   //treat as linear sensor
         int lstepR =pvtThreshold[sn]&0x7F; //linear row Pixels/step
         int lstepX =lineardX[sn];
       //  if ((pvtThreshold[sn] & 0x08)!=0) lstepX = -lstepX; //negative step 
         for (quad=0;quad<4;quad++) {            //do for four consecutive image (2x2) blobs
           row=(quad & 0x02);                    //equivalent row of std. sensor (0|2)
     //      iPtr=iPtr0+((quad*lstepR)>>2)*pitch+(quad*lstepX>>2)*2);(  //LHS of start of quadrant image
             iPtr=iPtr0+((quad*lstepR)>>2)*pitch+((quad*lstepX)>>2)*2;  //round down pixel count
           j = sn*S666_pitch + row*S666_row +(quad & 1)*6;             //destination row & L/R quad
           for (i=0;i<2;i++) {                   //do for 2 rows of one quad 
             decode565(iPtr,long(pitch*i) , &f666[j+i*S666_row]);     //do for L pixel
             decode565(iPtr,long(pitch*i+2),&f666[j+i*S666_row+3]);   //do for R pixel      
           }
         }
       }    
     }
   }
   return true;            //return converted (or number of converted active sensors?)       
}       // ***************************** 

//  DECIDE TIME TO LOAD ALL INITIAL SensorRefs, IMAGE AND BRIGHTNESS
int S666init_SensorRefs(int prefill){
      //initialise ALL active Sensors with image, brightness and colour ratios,  BUT not until CAM auto focus/exposure etc. has had time to settle.
      //prefill: function only acts after <prefill> number of calls (prefillRef) (normally called once per new fb frame loop()
      //decrement prefill each call.  when ===1, wait longer if <cDelay> milliseconds have not passed (a further 10sec delay?)
    if (prefill==2) { 
      DOr00(); 
      new_camera_settings(Bri,Con,Sat,AWB,AWBg,AEC,AECd,AEL,AGC,AGg);    //set Sat,AWB,AEC,AGC,..   select AWB OFF
    }
    if (prefill==1)                       //prefill increments down - consider acting when gets to 1.
      if (cFlag!=0) {                     //wait further time.  <cFlag> holds millis() time stamp at start of timeout
        if (cFlag+cDelay < millis()) {    //update ALL references only AFTER <cDelay> milliSeconds of active camera imaging to flush poor images
/***/   IF6 printf("dbug6 - grab_ref for all enabled sensors\n");
          for (int bsn=0;bsn<80;bsn++)    //grab sensorRef[] images for all active bsNo's
            if(SensorActive[bsn]){        //call grab_ref() for Active sensors only (includes new Cratios & brightness calculations) 
              grab_ref(bsn,Sensor666,S666_pitch,long(bsn*S666_pitch), &Sensor_ref[bsn*S666_pitch], &Sen_Brightness_Ref[bsn], &SensorRefRatio[bsn*12]);  
            }
          prefill=0;            //terminate prefill countdown
          cFlag=0;              // clear request - set cFlag to 0 so it won't invoke another full grab 
        }  
      }         //end of if(cFlag!=0)
    if (prefill>1) prefill-=1;  //decrement prefill count if not <=1
    return prefill;                //return decremented loop countdown    
}     // ***************************** 

//  FUNCTION TO EVALUATE MAXDIFF & OTHER MEASUREMENTS TO SET SENSOR STATE TRUE/FALSE OCCUPIED/UN-OCCUPIED 
bool processDiff(int bsn){
      // bsn Block sensor number for status evaluation
    static int  SensorFilter[80];  //slows dropout by a cycle or two - must be unoccupied for more than 100mSec
    int SenHiCnt;
    int Threshold;
        emptyStateCtr[bsn]++;                   //increment sensor "un-tripped" counter (clear later if tripped 
        bright=quad[0]+quad[1]+quad[2]+quad[3];
/***/   IF0 printf("bsn: 0x%x brightness quads: %d %d %d %d sum: %d _Ref %d\n",bsn,quad[0],quad[1],quad[2],quad[3],bright,Sen_Brightness_Ref[bsn]); 
        if (bright>Sen_Brightness_Ref[bsn])bright= bright*16/Sen_Brightness_Ref[bsn]-16;    //rescale bright for addition to maxDiff
        else bright=Sen_Brightness_Ref[bsn]*16/bright-16;  //produces a ratio of 0 up. (20% diff. gives bright=3)
        int b=bsn>>3; 
        int bpd=brightSF*bright+maxDiff;            //bpd = Brightness Plus colour Diff 
        if (bsn<maxSensors) {                   //save bsn & bpd in i2c message
          i2cData[i2cDatai]=byte(bsn);
          if(bpd<128)i2cData[i2cDatai+1]=byte(bpd); else i2cData[i2cDatai+1]=127;     //cap bpd value at 127 to fit in i2c byte.
          if (i2cDatai<56)i2cDatai += 2;        //increment provided within buffer limit(58) (may corrupt last bsn)   
        }
        Threshold = pvtThreshold[bsn];          //if == 255 NO pvtThreshold
        if (Threshold==255) Threshold=threshold;//no pvtThreshold
        else if(Threshold >= 128) {             //linear sensor
           Threshold = pvtThreshold[bsn | 0x07];       //get from sensor b7 
           if (Threshold >= 128) Threshold=threshold;  //use std if b7 also linear
        }

        if (bpd < Threshold) {    //low diff thinks unoccupied    
          if(SensorHiCount[bsn]>0){          //If had some recent hi's, update histogram
            SenHiCnt=SensorHiCount[bsn];     
            if(SenHiCnt>4) SenHiCnt=4;       // cap at 4
            SensorHisto[bsn*5+SenHiCnt]+=1;  // increment histogram
            SensorHiCount[bsn]=0;            // clear Hi counter
          }        
           if(SensorFilter[bsn]>0){ SensorFilter[bsn]-=1;     //only clear block bit after 2nd or 3rd "unoccupied"
            if(bsn<maxSensors){ i2cData[i2cDatai-2] |= 0x80; //flag as uncertain for i2c
              IFsc if(bsn>=minSensors) {
                if(bsn<8)Spr("0");
                Spr(bsn,OCT);{if(bpd>99)Spr(":?"); else Spr(":?_");}
                Spr(bpd);             
                if(SensorStat[bsn]){Spr(OCc);Spr(OCc);if(OCc!=char(12))Spr('*');Spr(" ");}//Ch(12) oversize(x1.5)
                else Spr("_?* ");
              }
            }
           }else{        //sustained LOW, so set UN-OCCUPIED
            SensorStat[bsn]=false; SensorFilter[bsn] =1-min2flip;  //reset filter (-ve) to count UP towards occupied if needed  (-1 requires 2 "occupieds")
            SensorBlockStat[b]= SensorBlockStat[b] & ~mask[bsn&7]; //clear SensorBlock bit
            if(bsn<maxSensors){
              IFsc if(bsn>=minSensors) {if(bsn<8)Spr("0");Spr(bsn,OCT);Spr(":--");Spr(bpd);Spr("--* ");}
            }
          }
        }else{          //high diff thinks OCCUPIED
          SensorHiCount[bsn]+=1;           // increment Hi counter for histogram 
          if(SensorFilter[bsn]<0){ SensorFilter[bsn]+=1;       //only clear block bit after 2nd or 3rd "occupied"
            if(bsn<maxSensors){i2cData[i2cDatai-2] |= 0x80;    //flag as undecided for i2c
              IFsc if(bsn>=minSensors){ 
                if(bsn<8)Spr("0");
                Spr(bsn,OCT);{if(bpd>99)Spr(":?"); else Spr(":?_");}
                Spr(bpd);Spr("_?* ");
              }
            }
          }else{        //sustained HIGH, so set OCCUPIED (if Twin agrees) 
            if((SensorTwin[bsn]<=0) || (SensorStat[SensorTwin[bsn]])) {   //only trip if no twin or TWIN agrees!
              if(!SensorStat[bsn]) SensorHisto[bsn*5]+=1;      //new actual trip so increment Histo trip counter.
              SensorStat[bsn]=true;  SensorFilter[bsn] = min2flip;   //i.e. occupied, 
                //set dropout filter (+ve) to discourage dropout registration (from noise) 
              SensorBlockStat[b]= SensorBlockStat[b] | mask[bsn&7];  //set block bit to 1.
              if(bsn<maxSensors){ i2cData[i2cDatai-1] |= 0x80;     //flag as occupied for i2c
                emptyStateCtr[bsn]=0;                             //clear "untripped" counter
                IFsc if(bsn>=minSensors){
                  if(bsn<8)Spr("0");
                  Spr(bsn,OCT);{if(bpd>99)Spr(":o"); else Spr(":oo");}
                  Spr(bpd);Spr(OCc);Spr(OCc);if(OCc!=char(12))Spr('*'); Spr(' ');
                }
              }
            }else{      //twin disagrees            
              if(bsn<maxSensors){
                IFsc if(bsn>=minSensors){
                  if(bsn<8)Spr("0");
                  Spr(bsn,OCT);{if(bpd>99)Spr(":o"); else Spr(":oo");}
                  Spr(bpd);Spr("?T* ");
                }
              }
            }
          }
        }
        IF0 {Spr(SensorFilter[bsn]);Spr(' ');Spr(maxDiff);Spr('+');Spr(bright);Spr('=');Spr(maxDiff+bright);}  
        IF0 Serial.println(' ');
        return SensorStat[bsn];
}   // ***************************** 

//  Function to convert an integer to (n) characters after ',' & before '\n'
void toAscii(byte * buff,int value, int n) {
    buff[0] = ',';
    for (int i=n;i>0;i--){
      buff[i]=value%10+0x30;
      value=value/10;
    }
    buff[n+1]= '\n';
}    // ****************************
      
//  FUNCTION TO ACCEPT I2C COMMANDS/DATA (AS SLAVE)
void i2cReceive(int len){                      //function to handle i2c received (interrupt?)
 // uint8_t Nbytes=32; 
  int  numDigBytes = (NUMdigPins+7)>>3;   //preset to make function work!
  byte i2cIncoming[13];         //use to hold translated command. longest: k%%,123,234 12 char's + \n?
  int  i=0;
  int  j=0;
  int  bsn=0;
  char bs=00;
       while(MyWire.available()){
         char  c=MyWire.read();
         if((i>0)||(c!='"')){          //strip any leading dummy'"' (used to suppress master interpreter)
            i2cIncoming[i]=c;          //accumulate incoming buffer
            i++;   
         }
       }
/***/  IF8 if((i2cIncoming[0] != '@') && (i2cIncoming[0]<0x80)){
/**/         Spr('\n');Spr(i);Spr("bytes: ");Spr(char(i2cIncoming[0]));Spr(' ');Spr(i2cIncoming[1],HEX);Spr(' ');
/**/                                                                            Spr(i2cIncoming[2],HEX);Spr(' ');
/**/     Spr(i2cIncoming[3],HEX);Spr(' ');Spr(i2cIncoming[4],HEX);Spr(' ');Spr(i2cIncoming[5],HEX);Spr(' ');
/**/         Serial.println(i2cIncoming[6],HEX);       
/**/   }

       if(i2cIncoming[0]>=0xE0){           //then treat as EX-IOEXPANDER code and translate to sensorCAM command    
/***/     IF8 if((E6counter>0)&&(i2cIncoming[0]!=0xE6)) Serial.write('E');     //don't output x for EXIORDD
          i = i2cExpanderReceive(i2cIncoming[0],(char *)i2cIncoming,(byte *)dataPkt);      //expander protocol
/*             MyWire.write(dataPkt,Nbytes);   //preload onRequest buffer  */    // ###########
          i2cIncoming[i]='\n';             //ensure ends with newline
                                           //don't use normal 10Hz processing slot for priority cmds.
          
          if((i!=0)&& !newi2cCmd){         //only accept if no newi2cCmd being processed   
            newi2cCmd=true;
            for (j=0; j<13; j++) i2cCmd[j]=i2cIncoming[j];
/***/       IFI2C Spr(":@:");    
          }
/***/     IFI2C  if(i>0) {Serial.print(i,HEX);Serial.print(char(i2cCmd[0]));Spr(newi2cCmd);}
          return;            
       }  //if header was >+0xE0 then never get past here

          //   case EXIORDD:                   //EXIORDD(0xE6)read ALL digital bits (@100Hz)
       if(i2cIncoming[0]=='@') {        // lower case '@' alternate cmd for EXIORDD
         dataPkt[0] = '`';              //new header (ver 300+) CS only sees  '`'
         for (int b=0;b<numDigBytes;b++)  dataPkt[b+1] =  SensorBlockStat[b];
         dataPkt[1] = dataPkt[1] & 0xBF;  //clear S06 bit to aid CS bad packet identification  //######
   //     MyWire.write(dataPkt,(NUMdigPins+7)>>3);    //preload onRequest buffer         //##########    
         EXCScmd0='@';       //save flag for i2cRequest()
         return;
       }  
          
       //convert cap alpha to lower alpha and ascii string       
       if(i2cIncoming[0]<='^'){       // then capital cmd instead of EXIOWRAN (0xEA) encoded
         if(i2cIncoming[0]=='V' && i2cIncoming[2]==0) i2cIncoming[0]='^'; //make 'V 0' equal '^'
         if(i2cIncoming[0]==']')i2cIncoming[0]='F';  //wants a (webcam) reset rather than 'f' frame
         else i2cIncoming[0]=i2cIncoming[0]+0x20;    //to lowercase (ver '^' -> '~' , data '\' -> '|' )

         uint16_t par1= i2cIncoming[2]+ i2cIncoming[3]*256;
         uint8_t  par2= i2cIncoming[4];
         uint16_t par3= i2cIncoming[5]+ i2cIncoming[6]*256;
         i2cIncoming[12]='\n';        //ensure ends with newline
          
         if((i2cIncoming[0] == 'a')&&(par3 < 999)) {   //assumes 4 par format
           par1=i2cIncoming[1]; // par1=(i2cIncoming[1]/10)*8+(i2cIncoming[1]%10 & 0x07);    //bsn back to bsNo
												 
           for(j=1;j<12;j++) i2cIncoming[j]=' ';
           toAscii(&i2cIncoming[3],par2,3); 
           toAscii(&i2cIncoming[7],par3,3);
           i=11;  

/***/  //     Spr("new cmd:"); for (int i=0;i<12;i++) Spr(char(i2cIncoming[i]));          
         }else{
           for(j=1;j<12;j++) i2cIncoming[j]=' ';
           if(par1>=10000) par1=(par1-10000)/100;  //decode parameters in par1 if needed
           if(par3<98) {
            toAscii(&i2cIncoming[3],par3,2);
            i=6;
           }else i=3;
         }
         if((i2cIncoming[0]=='p') && (par1<10)) par1=par1*10;
         if((i2cIncoming[0]=='q') && (par1<10)) par1=par1*10;
         i2cIncoming[1]= par1/10+0x30;
         i2cIncoming[2]= par1%10+0x30; 
         
         if(i2cIncoming[0]=='j'){  
           toAscii(&i2cIncoming[1],par3,1);      
           i2cIncoming[1]= par1;
         }      
       }   // should now have an ascii command in i2cIncoming starting with lower case cmd 
       
       if(!newi2cCmd) { 
         newi2cCmd=true;     //skip new command if old one not yet finished.
         for (j=0;j<13;j++) i2cCmd[j]=i2cIncoming[j];  //assumes last command completely processed!
       }                 //assume it is ASCII cmd.

       IFI2C {Spr("CS:");for(int k=0;k<13;k++)Spr(i2cCmd[k]);Spr('\n');}
	   
      // need to prepare response buffer for selected commands 
      //   case EXIOVER:                  //EXIOVER(0xE3)
       if((i2cIncoming[0]=='~')||(i2cIncoming[0]=='v' && i2cIncoming[2]==0)) {    // lower case '^' gives '~'
         dataPkt[0] = '~';              
         dataPkt[1] = byte(BCDver/100);
         dataPkt[2] = byte(BCDver%100);   
   //      MyWire.write(dataPkt,3);     //preload onRequest buffer      
         i2cCmd[0]='^';                 //alt. version cmd       
         i = 1; 
       }
    
       if(i2cIncoming[0]=='b') {
         if (!isDigit(i2cIncoming[1])) i2cIncoming[1]=0x39; 
         /*Nbytes=*/ i2cPrepare((char)i2cIncoming[0],(char)i2cIncoming[1],(char *)dataPkt);   //make an up-to-date dataPkt      
         //MyWire.write(dataPkt,Nbytes);                                   //preload onRequest buffer
       }
       if(i2cIncoming[0]=='f') {
         if(isDigit(i2cIncoming[1])) {       //i2cIncoming[1] updates dMaxDiff & dBright only once every 100mSec
            bsn=i2cIncoming[1]-0x30;         //get bsNo
               //check for 2nd # // No error check if 8 or 9!
            if(isDigit(i2cIncoming[2])) bsn=bsn*8+((i2cIncoming[2]-0x30)&7);  //8=0 & 9=1 !
         }
		     
         /*Nbytes=*/ i2cPrepare((char)i2cIncoming[0],(char)(bsn+0x30),(char *)dataPkt);   //make an up-to-date dataPkt      
         //MyWire.write(dataPkt,Nbytes);                                   //preload onRequest buffer 
         for(j=0;j<32;j++) {        //pull down next row
           dataPkt[j]=dataPkt[j+32];dataPkt[j+32]=dataPkt[j+64];dataPkt[j+64]=dataPkt[j+96];    
         }
       }
       if((i2cIncoming[0]=='m')||(i2cIncoming[0]=='n')) {
         bs=00;         
         /*Nbytes=*/ i2cPrepare((char)i2cIncoming[0],(char)bs,(char *)dataPkt);   //make an up-to-date dataPkt 
         //MyWire.write(dataPkt,Nbytes);   
       }
       if(i2cIncoming[0]=='i') {
         if(isDigit(i2cIncoming[1])) {       //i2cIncoming[1] updates dMaxDiff & dBright only once every 100mSec
            bsn=i2cIncoming[1]-0x30;         //get bsNo and convert to integer
               //check for 2nd #  // No error check if 8 or 9!
            if(isDigit(i2cIncoming[2])) bsn=bsn*8+((i2cIncoming[2]-0x30)&7);  //8=0 & 9=1 !
         }
	       bs=char(bsn+0x30);	      // "char" from 0x00 '0' to 0x4F 
         /*Nbytes=*/ i2cPrepare((char)i2cIncoming[0],(char)bs,(char *)dataPkt);   //make an up-to-date dataPkt      
         //MyWire.write(dataPkt,Nbytes);             
	     } 
            
       i2cCmd[12]='\n';            //ensure ends with newline
       newi2cCmd=true;            //set a flag for "loop()" to process cmd as if from USB
/***/  IFT {Spr(len); Serial.println(" bytes onReceive");}     //debug to USB
       digitalWrite(FLASHLED,LOW);   //don't let it stay on too long
       return;
}   // *****************************  

//  FUNCTION TO SEND I2C DATA TO MASTER        //slave doesn't need beginTransmission() write() or endTransmission()
void i2cRequest(){                             //function to handle i2c request (interrupt?)
 int i=0;
 int bsn=0;
					
     if(EXCScmd0=='@'){ MyWire.write(dataPkt,11); EXCScmd0=0; return;}   //preloaded onRequest buffer   
        
     if(EXCScmd==true) { i2cExpanderRequest(EXCScmd0, dataPkt); return; }    
     i=i2cCmd[1];               //get 1st parameter from LAST onReceive
     if (strchr((const char *)F("aeFgjlorsuwx"),i2cCmd[0]) != nullptr) 
       MyWire.write(EXIORDY);  //CS acknowledgement     
     else 
     switch(i2cCmd[0]) {        //first character decides return data      
       case 'p':            //p$;  //write ascending sensor coordinates into packet
         i=i2cPrepare((char)i2cCmd[0],(char)i2cCmd[1],(char *)dataPkt);
         MyWire.write(dataPkt,32);
         break;
       case 'b':            //b$:  //write descending blocks into packet (1 to 10 bytes)
         MyWire.write(dataPkt,32);//i);  //update onRequest buffer (write won't stop at first NUL char!)   
         break;
       case 'd':               //NOTE: master should request data 3 times to try to flush false values
         if(isDigit(i)) {      //loop() updates dMaxDiff & dBright only once every 100mSec
           bsn=i-0x30;         //get bsNo
           i=i2cCmd[2];        //check for 2nd # // No error check if 8 or 9!
           if(isDigit(i)) bsn=bsn*8+((i-0x30)&7);  //8=0 & 9=1 !
           dbsNo=bsn;          //loop() updates dMaxDiff & dBright only once every 100mSec 
           MyWire.write('d');       //write "header"
           MyWire.write(bsn);   
           MyWire.write(byte(dMaxDiff+dBright));MyWire.write(byte(dMaxDiff));MyWire.write(byte(dBright));
         }break;
       case 'q':            //q$:  //write descending blocks into packet (1 to 10 bytes)
         if(!isDigit(i)) i=0x39;   //default "q " to "q9"   
         MyWire.write('q');        //write "header"
         MyWire.write(i);          //no of ASCII # bytes?
         for (i=(i-0x30);i>=0;i--) MyWire.write(SensorActiveBlk[i]);
         break;
       case 'n':           //n$,## //set nLed and minSensors			 
       case 'm':           //m$,## //set min2flip and maxSensors   
         MyWire.write(dataPkt,7);
         break;         
       case 'i':            //i%%: //write state of sensor #/#    
         MyWire.write(dataPkt,10);
         break; 
       case 't':            //t$$$       //write old threshold value, & set new
         i2cData[0]='t';
         MyWire.write(i2cData);    //threshold should be in i2cData[0] and sensor[0] diff in i2cData[1]
         break;
       case 'f':
         if(isDigit(i)) {      //i2cCmd[1] updates dMaxDiff & dBright only once every 100mSec
           bsn=i-0x30;         //get bsNo
           i=i2cCmd[2];        //check for 2nd #   // No error check if 8 or 9!
           if(isDigit(i)) bsn=bsn*8+((i-0x30)&7);  //8=0 & 9=1 !
         }
         dataPkt[0]='f';
         MyWire.write(dataPkt,28);      //send first 27 bytes in 1 packet
         if(dataPkt[2]>=0x03){                     //have sent 4th row so get new sample
           i2cPrepare((char)i2cCmd[0],char(bsn+0x30),(char *)dataPkt);   //make new up-to-date dataPkt      
         }else for(i=0;i<32;i++) {                 //pull down next row
           dataPkt[i]=dataPkt[i+32];dataPkt[i+32]=dataPkt[i+64];dataPkt[i+64]=dataPkt[i+96];
         }break; 
       //case EXIORDD:               //0xE6 -> 1+NUMdigPins/8 bytes + spare analog
       //as alternative to '@'/'`'
       case '@':
         MyWire.write(dataPkt,10/*numDigBytes*/+1);  //preload onRequest buffer 
         break;	
       case '~':               //'^' from <N Ver> version request
       case '^':
         MyWire.write(dataPkt,3);   //preload onRequest buffer        
         break;
     
       default:             //if invalid onReceive Cmd, onRequest returns dud i2cCmd as data packet
         MyWire.write(CAMERR);     //return header byte as Error flag 0xFE(254.)    
         MyWire.write(i2cCmd);     //send back (preload) old ASCII received data packet.    //#######
         Serial.print("onRequest dud command preloaded. ");Serial.println(i2cCmd[0],HEX);
     } 
}   // ***************************** 

//  FUNCTION TO TRANSLATE IOEXPANDER I2C DATA & prefill MyWire.write i2c buffer
int i2cExpanderReceive(byte cmd,char *i2cCmdBuf,byte *datapk){
 //  returns the number of characters placed in i2cmdBuf[]
 //cmd: EXIOINIT, EXIODPUP, EXIOVER, EXIOWRD, EXIORDD, ( EXIOWRAN )
  static int CSstartup=2;
  int valu=0;
  uint8_t Nbytes;
  unsigned int y;
  int x;
  const int numDigBytes=(NUMdigPins+7)>>3;

    EXCScmd = true;  
/***/  IF8 if (E6counter>0) { E6counter--; if(cmd!=EXIORDD)Serial.write(0x30|(cmd&0x0F));}    //print cmd               
/***/  if (cmd == EXIOINIT){ CSstartup--;
/***/   if (CSstartup>0){Serial.write("e0"); E6counter=200;}
/***/  }
 
    EXCScmd0 = cmd;                   //save for onRequest
    switch(cmd){
      case EXIOINIT:                  //EXIOINIT(0xE0) nPins first
        //should check npins matches NUMdigPins - what does exioexpander do??
        firstVpin=i2cCmdBuf[2]+i2cCmdBuf[3]*256;
        datapk[0]=EXIOPINS;
        datapk[1]=NUMdigPins;
        datapk[2]=NUManalogPins;
 //       MyWire.write(datapk,3);     //preload onRequest buffer      // ######
        return 0;                 

      case EXIOINITA:                 //EXIOINITA(0xE8)initialise analog & return map onRequest
 //     just write analog map NUManalogPins bytes on Request. 
 //       MyWire.write(analoguePinMap,NUManalogPins);                 //nano has 6, uno 4   
        return 0;
 
      //as alternative to '^'or '~'      // '^' gives '~'
      case EXIOVER:                   //EXIOVER(0xE3)
        datapk[0] = EXIOVER;
        datapk[1] = byte(BCDver/100);
        datapk[2] = byte(BCDver%100);
 //       MyWire.write(datapk,3);     //preload onRequest buffer      // ######
        i2cCmdBuf[0]='v';           
        i2cCmdBuf[1]='e';       
        return 2;              
      
      case EXIOWRD:                   //EXIOWRD(0xE5) pin value   use for cmds 'r' & 'a'
        datapk[0]=EXIORDY; 
  //      MyWire.write(datapk,1);      //preload onRequest buffer      // ###### 
/***/   IFI2C { Serial.print(i2cCmdBuf[1],HEX);   Serial.print(i2cCmdBuf[2],HEX); }  
        if (i2cCmdBuf[1] < 80) {       //refresh 'r' or 'a' a bsNo
          i2cCmdBuf[2]=(i2cCmdBuf[1]& 0x07)+ 0x30;
          i2cCmdBuf[1]=(i2cCmdBuf[1]>>3)+ 0x30;
          if(i2cCmdBuf[2]==0){         //e.g. <Z 123 0> equals refresh vpin 719 bsn 19 (023)
            i2cCmdBuf[0]='r';
          }else{                       //e.g. <Z 123 1> equals activate/enable vpin 719 bsn 19 (023)
            i2cCmdBuf[0]='a';
          }
          return 3;
        }
        if (i2cCmdBuf[1] >= 80) {     //reserved control vpins above sensors
          int vCmd=i2cCmdBuf[1]-80;       //convert vpin(>=80 to digit 0-3 (X0-X3)  
          switch(vCmd){
            case 1:                   //if vpin = bsn == 81 do a w/wait or run (end wait)
              i2cCmdBuf[0]='w';                        //wait
              if(i2cCmdBuf[2]==0) i2cCmdBuf[0]='\n';   //resume run
/***/          IFI2C Serial.print(i2cCmdBuf[0],HEX);
              return 1;
            default:
             return 0;
          }      
        } 
        return 0;
        
      case EXIODPUP:                  //EXIODPUP(0xE2) pin pullup     USE (1) for 'a' or (0) 'r'  
        datapk[0]=EXIORDY;        
   //     MyWire.write(datapk,1);      //preload onRequest buffer      // ######
        return 0;   
             
   // case EXIORDAN:                  //EXIORDAN(0xE4) Read full set analogue values (@20Hz)**duplicate RDD**
        //as alternative to '@' or '`'  //lower case'@' gives '`'    
      case EXIORDD:                     //EXIORDD(0xE6)read ALL digital bits (@100Hz)
        datapk[0] = EXIORDD;            //new header (ver 200+)
        for (int b=0;b<numDigBytes;b++)  datapk[b+1] =  SensorBlockStat[b];
        datapk[1] = datapk[1] & 0xBF;   //clear S06 bit to aid CS bad packet identification
   //     MyWire.write(datapk,(NUMdigPins+7)>>3);    //preload onRequest buffer      // ######     
        return 0;

      case EXIOENAN:                  //EXIOENAN(0xE7)nEnable Analogue 
        datapk[0]=EXIORDY;            //just acknowledge for now
   //     MyWire.write(datapk,1);       //preload onRequest buffer      // ######     
        return 0;

      case EXIOWRAN:                  //(0xEA)
        datapk[0]=EXIORDY;
        y=i2cCmdBuf[1]%100;           //for sensorCAM vpin must start at #00
        x = 256*i2cCmdBuf[3]+i2cCmdBuf[2];
        valu=i2cCmdBuf[4];            //profile
        if(valu<240) {      //treat as an 'a' request if Buf[4] looks like a row
          valu=int(i2cCmdBuf[3]*256+i2cCmdBuf[2]);
/***/     IFI2C {        Spr(int(i2cCmdBuf[1]),DEC); Spr(' ');Spr(valu);
/***/       Spr(' ');Spr(int(i2cCmdBuf[4])); Spr(' ');Serial.println(i2cCmdBuf[5]+i2cCmdBuf[6]*256);}        
          y=i2cCmdBuf[4];                     //get row
          i2cCmdBuf[4]=byte((y/100) | 0x30);  //extract row
          y=y%100;
          i2cCmdBuf[5]=y/10 | 0x30;
          i2cCmdBuf[6]=y%10 | 0x30;
      //    toAscii(&i2cCmdBuf[3],y,3);
      //    toAscii(&i2cCmdBuf[7],x,3);
          i2cCmdBuf[7]=',';
          i2cCmdBuf[8]=x/100| 0x30;           //extract column x
          x=x%100;
          i2cCmdBuf[9]=x/10 | 0x30;
          i2cCmdBuf[10]=x%10 | 0x30;
          i2cCmdBuf[3]=',';
          i2cCmdBuf[2]=i2cCmdBuf[1]%8 | 0x30; //extract b/s from vpin
          i2cCmdBuf[1]=i2cCmdBuf[1]/8 | 0x30;
          i2cCmdBuf[0]='a';           //although k requested treat as extended 'a'
/***/     IFI2C for(x=0;x<11;x++) Serial.print(char(i2cCmdBuf[x]));      
 //         MyWire.write(datapk,1);   //preload onRequest buffer      // ######
          return 11;
        }
/***/   IFI2C { Serial.print(i2cCmdBuf[1],HEX);   Serial.print(i2cCmdBuf[2],HEX); }
        if(/*(y==80) ||*/ (valu>=240)){                    // primary command vpin
          const char cmds[]={'o','l','a','n','r','s','u','F','i','-','\n','w','g','e','m','v'};
          int  cmdNo=i2cCmdBuf[4]%240;              //get command #
      
           valu=x%100;  //(256*i2cCmdBuf[3]+i2cCmdBuf[2])%100;    //get bsNo from id limit < 100
           i2cCmdBuf[2]=(valu%10) | 0x30;   //treat valu as bsNo not bsn
           i2cCmdBuf[1]=(valu/10) | 0x30;
           i2cCmdBuf[0]=cmds[cmdNo];
           if((cmdNo==4)&& (x>100)){        //r%%*
             i2cCmdBuf[3]='*'; 
             return 4;
           }                          
           if(cmdNo<=6)                     // 3-character commands   
             if(cmdNo!=3)return 3;          //0-6  o l a (n) r s u        
           if(cmdNo==7)                     //'F' (was to be 'f' frame)
             return 1;						
           if((cmdNo>=10) && (cmdNo<=13))   // 1-char. commands   
             return 1;                      //10,11,12,13  \n w g e
           if(cmdNo==15) {                  //15 v#  (2 char cmd)
             i2cCmdBuf[1]=(x & 0x07)+0x30;  //limit to one digit (0-7)
             return 2;                      //arrange for the digit to force R             
           }
             
           int par1=x;      // n i t m only
           int par2=98;    //98 always invalid
           if(x>=10000) {x=x%10000; par1=x/100; par2=x%100;}           
           if(x>=100) { par1=x/100; par2=x%100;} 
           
           i2cCmdBuf[1]=par1/10 | 0x30;
           i2cCmdBuf[2]=par1%10 | '0';		
           i2cCmdBuf[3]=',';
           i2cCmdBuf[4]=par2/10 + 0x30;
           i2cCmdBuf[5]=par2%10 + 0x30;
/***/      Spr("command:");for (int ii=0;ii<6;ii++)Spr(char(i2cCmdBuf[ii]));
           //prepare early reply and preload outbuffer if appropriate
/***/        Serial.print(cmds[cmdNo]);   Serial.println(x);
             //prepare packet for all remaining cmds ( m n i t )
           {   //8=248=cmds[8}='i' (also 'm')
             // par1 and par2 already extracted  // if(x>=10000) x=(x-10000)/100;
             uint8_t bs=(x/10)*8+((x%10)&0x07);    //convert to valid bsn
             bs=char(x+0x30);       // char from 0x00 '0' to 0x4F 
             Nbytes = i2cPrepare((char)cmds[cmdNo],(char)bs,(char *)datapk);   //make an up-to-date dataPkt      
             MyWire.write(dataPkt,Nbytes);  
             return 6;                    
           }
           if(par2==98) return 3;
           Serial.println("invalid EXIOWRAN cmd");
           return 0;      // this rejects uninterpreted commands needing ascii formatting e.g. i & t
        }
        datapk[0]=EXIOERR; 
        return 0;               
      default:
        datapk[0]=EXIOERR; 
    //      MyWire.write(datapk,1);   //preload onRequest buffer      // ######
/***/   IF8 Serial.write((cmd&0x0F)+0x30);    
/***/   IF8 Serial.write("EF");           
        return 0;
        break;    
    }
    return 0;
}    // ***************************** 

//  FUNCTION TO RESPOND TO IOEXPANDER I2C REQUEST  
void i2cExpanderRequest(byte cmd,byte *datapk){
  //cmd: EXIOINIT, EXIODPUP, EXIOVER, EXIOWRD, EXIORDD, EXIOWRAN
  // output buffer for MyWire.write was prefilled by i2cREquest so don't need again??
  const int numDigBytes=(NUMdigPins+7)>>3;
    switch(cmd){
      case EXIOINIT:
        MyWire.write(datapk,3);   //preload onRequest buffer      // ######
        break;
                   
      case EXIOINITA:             //0xE8 -> NUManalogPins bytes
        MyWire.write(analoguePinMap,NUManalogPins);                 //nano has 6, uno 4
        break; 
     
      //as alternative to '^'/'~'
      case EXIOVER:               //0xE3 -> 0 1 ve%100
        MyWire.write(datapk,3);   //preload onRequest buffer      // ######  
        break;

      case EXIOWRD:               //0xE5 -> EXIORDY  //(1)sets 'a'; (0)sets 'r' but if 0/0 (1) for 'w' and (0) for '\n'
        MyWire.write(datapk,1);   //preload onRequest buffer      // ###### 
        break;
        
      case EXIODPUP:              //0xE2 -> EXIORDY
        MyWire.write(datapk,1);   //preload onRequest buffer      // ######
        break;
   
   //   case EXIORDAN:              //0xE4 -> 2*NUManalogPins
   //as alternative to '@'/'`'
      case EXIORDD:               //0xE6 -> 1+NUMdigPins/8 bytes + spare analog
        MyWire.write(datapk,numDigBytes+1);  //preload onRequest buffer // ######
        break;
      
      case EXIOENAN:              //0xE7 -> EXIORDY
        MyWire.write(datapk,1);   //preload onRequest buffer      // ###### 
        break;
              
     case EXIOWRAN:               //0xEA -> EXIORDY
        MyWire.write(datapk,1);   //preload onRequest buffer      // ###### 
        break;

      default:
        break;
    } 
    EXCScmd=false;
  //      newi2cCmd=false;  Should leave true for main code to process any cmd for screen output.             
}   // *****************************
       
//  FUNCTION TO PREPARE AN I2C DATA PACKET (AS SLAVE)
int i2cPrepare(char cmd, char block, char *datapk) {    //function to prepare an i2c packet for "onRequest"
  //cmd: 'p' or 'b'   bank: (0-9)  datapk[32] byte buffer for pkt
byte  i2cparity;
int   blk=0;
int   i=1; 
int   n=0;
int   bsn;
const int pitch=320*2;                  //row length in RGB565 buffer (QVGA 320x240)   
      datapk[0] = cmd;                  //MyWire.write('b');        //write "header"
      i2cparity = cmd;                  //initialise  
      switch (cmd){
        case 'b':{              //b$:   //write descending banks into packet (1 to 10 bytes) 
         if(!isDigit(block)) block='9'; //default "b " to "b9"
         block=block-0x30;         
          for (blk=block;blk>=0;blk--) { 
            datapk[1+int(block)-blk] = SensorBlockStat[blk];     //MyWire.write(SensorBlockStat[i]);
            i2cparity = i2cparity ^ SensorBlockStat[blk];    //exclusive OR parity generation.
          }
          datapk[2+block]=i2cparity;  //   ;MyWire.write(i2cparity); //xor parity byte for preceeding $+1 bytes
 /***/    IFI2C { for (blk=0;blk<(block+3);blk++) {Spr(datapk[blk],HEX);Spr(" ");} Serial.println(" ");}
          return block+3;             //return byte count
        }
        case 'i':{            //i%%:  //write state of sensor %/%   
          bsn=(byte(block&0x7F)-0x30);             //get bsn     0x30-0x39 looks like binary bsn +0x30 ('0' to 177)
          if(block<0x30)bsn=00;       //assume block has been loaded with b*8+s
                //if 'i' from CS driver, it must output Ascii bsNo. even though it gets an id format.
          datapk[0]=byte('i');
          datapk[1]=byte(bsn);
          datapk[2]=SensorStat[bsn];
          datapk[3]=SensorActive[bsn];   //bool         
          i=int((Sensor[bsn]%pitch)/2);  //column
          datapk[4]=byte(i&0xFF);
          datapk[5]=byte(i>>8);
          datapk[6]=byte(Sensor[bsn]/pitch); //row
          datapk[7]= SensorTwin[bsn];
          n=0;
          for (int j=0;j<48;j++) n+=Sensor666[j+bsn*48];   //sum brightness
          datapk[8]= n>>4;   //return 3 times average value (avoids divide)max=3F*3=189.
          datapk[9]=pvtThreshold[bsn];      
          return 10;
        }
        case 'n':
        case 'm':{
          datapk[0]=byte(cmd);
          datapk[1]=byte(min2flip);
          datapk[2]=byte(minSensors);
          datapk[3]=byte(maxSensors);
          datapk[4]=byte(nLED);
          datapk[5]=byte(threshold);
          datapk[6]=byte(TWOIMAGE_MAXBS);      
          return 7;
        }
        case 'p':{            //p$;  //write positions of defined sensors in block starting at sensor $0         
          if(!isDigit(block)) block='9';   //default "p " to "p9"
          block=block-0x30; 
          for (blk=(block*8);blk<=(block*8+7);blk++) {               //note: parity unverified!
            if(Sensor[blk] != 0) {      //only send defined blocks
              datapk[i+1] = blk;     
              datapk[i+2] = Sensor[blk]/pitch;    //get row
          int xVal=(Sensor[blk]%pitch)/2;         //get x (0-319)
              if (xVal>=256) datapk[i+1]=datapk[i+1]|0x80;   //add x hi bit to blk
              datapk[i+3] = xVal&0xFF;
              i2cparity = i2cparity ^ datapk[i+1] ^ datapk[i+2] ^ datapk[i+3] ^ datapk[i+4];
              i=i+3;         
            }                
          }   
          datapk[1]=i+2;                          //include byte count & parity
          datapk[i+1]=i2cparity ^ datapk[1];      //xor in byte count
          return datapk[1];                       //byte count
        }
        case 'f':{
          n=block-0x30;               //treat character 0x30 to 0x7F as bsn (0x30=0/0 0x7F=9/7)
          //create FOUR datapackets for rows 0-3  
          for (i=0;i<4;i++){         // 4 rows/pkts required
            datapk[1+i*32]=byte(n);  //(n=0 to 0x4F) put bsn in byte[1] & row # in byte[2]
            datapk[2+i*32]=byte(i);
            for (int j=0;j<12;j++){  //put row of sensor(0) in bytes 3-14, & sensor(n) in 16-27
              datapk[ 3+j+i*32]=Sensor_ref[j+i*S666_row+n*S666_pitch];  //get i'th row Sensor_ref[n] in 3-14
              datapk[16+j+i*32]= Sensor666[j+i*S666_row+n*S666_pitch];  //get i'th row Sensor[n] in 16-27
            }
          }
          return 28;    // header,n,row,12ximage(0),12ximage(n) (1 row)
        }
        default: 
          return 0;     //error return ZERO bytes - unrecognised cmd.
      }
}   // *****************************

//  FUNCTION TO FLASH LED
void flashLed(int pulse){       //flash <pulse> uSec long
unsigned long flash;             // *assumes FLASHLED,OUTPUT
        digitalWrite(FLASHLED, HIGH); //turn on flash
        flash=micros()+pulse; 
        while(flash>micros()){}       //delay <pulse> uSec for bright white flashLed
        digitalWrite(FLASHLED, LOW); 
}   // *****************************

//  FUNCTION TO CONNECT TO WIFI    
static bool ConnectWifi(const char *_ssid, const char *wifipass){
  WiFi.disconnect(true, true);
  WiFi.begin(_ssid, wifipass);
  uint8_t wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20)
  {
    Spr(".");
    delay(1000);
    if(wifiAttempts == 10)
    {
      WiFi.disconnect(true, true);  //Switch off the wifi on making 10 attempts and start again.
      WiFi.begin(_ssid, wifipass);
    }
    wifiAttempts++;
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    WiFi.setAutoReconnect(true);    //Not necessary
    Serial.println();//Not necessary
    Spr("Connected with IP: ");//Not necessary
    Serial.println(WiFi.localIP());     //Not necessary
    return true;
  }
  else
  {
    WiFi.disconnect(true, true);
    return false;
  }
  delay(100);
}   // *****************************

//  FUNCTION TO GET AND DISPLAY CURRENT CAMERA SETTINGS
void getCAMstatus() {
        sensor_t *s = esp_camera_sensor_get();
  //      printf("\n\"xclk\":%u,", s->xclk_freq_hz );
  //      printf("\"framesize\":%u,", s->status.framesize);
  //      printf("\"quality\":%u,", s->status.quality);
        printf("(g) BRIGHTNESS:%d, CONTRAST:%d", s->status.brightness, s->status.contrast);
        printf(" SATURATION:%d, AWB:%u,", s->status.saturation, s->status.awb);
        printf(" awb_gain:%u, AEC:%u,", s->status.awb_gain, s->status.aec);
        printf(" AECd:%u, aec_value:%u,", s->status.aec2, s->status.aec_value);
        printf(" AELevel:%u, AGC:%u,", s->status.ae_level, s->status.agc);     
        printf(" AGgain:%u, bpc:%u,", s->status.agc_gain, s->status.bpc);
        printf(" wpc:%u, raw_gma:%u,", s->status.wpc, s->status.raw_gma);
        printf(" lenc:%u,", s->status.lenc);
  //      printf("\"hmirror\":%u,", s->status.hmirror);
  //      printf("\"dcw\":%u,", s->status.dcw);
  //      printf("\"COLORBAR\":%u", s->status.colorbar); 
        printf("\n");
}  // *****************************

//FUNCTION TO AVERAGE Numbr OF FRAMES AND UPDATE _ref[bsn] TO AVERAGE VALUES
void averageRcalculation(int AVbsn,int Numbr){      //NOTE: function can only average ONE bsNo at a time!
      //AVbsn: decimal value of bsn for which need to generate a new average Sensor_ref[] AVbsn=0 triggers ALL Active sensors averaged
      //Numbr: number of frames being averaged      //time to average = Numbr/10 seconds
      //uses global (int) agerageRcounter to count down number of frames in average and, when ==0, saves newref[] into Sensor_ref[n]
      //uses global (int) averageSensor[80x48] to hold Sensor accumulations
  static int averageSensor[48*80]={0};    //accumulation space for up to 64 frames of a sensor 
  int  i, n;
  byte newref[48];
  int  maxN=79;     
      if (averageRcounter>0) { 
        averageRcounter -= 1;             //decrement down counter;
        if(AVbsn>0) maxN=AVbsn;           //limit to just AVbsn. (else average ALL active sensors)
        for ( n=AVbsn; n<=maxN; n++) 
          if (SensorActive[n]){           //do averageing only if ACTIVE
            for ( i=0;i<48;i++){          //accumulate 48 (4x4x3) pixel bytes individually
              averageSensor[n*48+i] += Sensor666[n*48+i];   
              if (averageRcounter==0){    //end of summations so update ref
                newref[i] = byte(averageSensor[n*48+i]/Numbr);
                averageSensor[n*48+i]=0;  //clear accumulator
              }
            }
            if (averageRcounter==0) {         //now process into Sensor_ref[]         //includes new Cratios & brightness calculations
              grab_ref(n,newref,48,0L,&Sensor_ref[n*48],&Sen_Brightness_Ref[n],&SensorRefRatio[n*12]);  //includes new Cratios & bright   
/***/         IFN {Spr(" Have set _ref[");Spr(n,OCT);Spr("] to average values.  _ref[0] & _ref[bsn] below:\n");}
/***/         IFN if(AVbsn!=0) write_img_sample(Sensor_ref,12,AVbsn*48,24);      //print _ref[00] and (av) _ref[bsn]
            }          
          }
      }        
  //    if (averageRcounter==0) Spr(".......");
      autoRefSuspended = false;                     //allow auto referencing to resume
}  // *****************************

 //  FUNCTION TO SET BLOCK LED
void setLED(int b, bool sLED){
  int16_t BLKLED[] = {BLK0LED,BLK1LED,BLK2LED,BLK3LED,0,0,0,0,0,0,PLED,QLED};
  for (int i=0; i<=11; i++) if(BLKLED[i]!=0) digitalWrite(BLKLED[i],HIGH); //clear led
  for (int i=0; i<=9; i++) {
    b=i;
    if(i==NLED) b=11;       //nLED overrides qLED
    if(i==nLED) b=10;       //if block assigned to programmable LED 
    if(BLKLED[b] > 0) {
      if(SensorBlockStat[i] != 0) digitalWrite(BLKLED[b],LOW);  //set LED ON 
    }  //note: if multiple blocks on same led, LED used to reflect last block ONLY!
  }
}  // *****************************

// FUNCTION TO AVERAGE LAST 2 FRAMES before compare()
void av2frames(int bsn){          //byte RingBuff[2*80*16*3];
  // records new raw sensor in ring buffer and averages with previous one to replace current value
  static byte RingBuff[2*80*16*3];     //ring buffer to save grabs to average before trip
       for(int i=0;i<48;i++) {
       int indx=bsn*16*3;          //48 bytes/bsn
           if (avOddFrame) RingBuff[indx+i]=Sensor666[indx+i];    //save frame
           else    RingBuff[80*16*3+indx+i]=Sensor666[indx+i];
           Sensor666[indx+i] = (RingBuff[indx+i] + RingBuff[80*16*3+indx+i])/2;             
       }
       return;
}  // *****************************

//FUNCTION TO SUSPEND LOOPING
void wait(){
  //stops looping indefinitely until input a newline (\n) from USB or i2c  
       if(!newi2cCmd) printf("Waiting - 'Enter' to continue\n");
       while (!Serial.available() && !newi2cCmd) {
         if((micros()-starttime)>CYCLETIME) starttime += CYCLETIME;  //handles 71min overflow
       }
}  // *****************************

//FUNCTION TO PLACE A BOX AROUND SENSORS IN A CAMERA IMAGE FOR DISPLAY
void boxIt(unsigned long Point,int BSno){           //Use sensor() pointer into image 
  int i;
  int color565[]={0x0801,0x8920,0xF820,0xFC20,0xFFD0,0x07E0,0x081F,0xF81F,0x630C,0xFFFF}; //0-9
    if ((Point>FBPITCH) && (Point<(FBPITCH*(240-5-SEN_SIZE)))){  //no box if too close to top or bottom edge  
      for(i=0;i<((4+SEN_SIZE)*RGB565p);i++){
        imageFB[Point-FBPITCH+i]=0xFF; //DE;           //top row white
        imageFB[Point+FBPITCH*(4+SEN_SIZE)+i]=0xDE;    //bottom row
      }
      for(i=-1;i<(5+SEN_SIZE);i++) {
        imageFB[Point-2+i*FBPITCH]=byte(color565[BSno>>3]>>8);
        imageFB[Point-1+i*FBPITCH]=byte(color565[BSno>>3]&0xFF);
        imageFB[Point+(4+SEN_SIZE)*2+i*FBPITCH]=byte(color565[BSno&0x07]>>8);
        imageFB[Point+1+(4+SEN_SIZE)*2+i*FBPITCH]=byte(color565[BSno&0x07]&0xFF);
      }
      if((pvtThreshold[BSno]<255)&&(pvtThreshold[BSno]>127)) {          //then must be a linear sensor
        int dr= pvtThreshold[BSno]&0x7F;
        int dx= lineardX[BSno];

        for(int q=0;q<=3;q++){       //do for 4 2x2 spots (quads)
          for(i=0;i<2;i++){          //2 pixels per row.
            int offset=(i+((q*dr)>>2))*FBPITCH+((q*dx)>>2)*2;
            imageFB[Point+offset]  =byte(color565[3]>>8);   //try orange(3)
            imageFB[Point+offset+1]=byte(color565[3]&0xFF); //2 bytes/pixel
            imageFB[Point+offset+2]=byte(color565[3]>>8);   //next pixel
            imageFB[Point+offset+3]=byte(color565[3]&0xFF);
          }
        }
      }
    }
    return;
}   // *****************************

// FUNCTION TO EXECUTE A REFRESH OF SENSORREF[00] r00 
 void DOr00() {
      for (int i=0;i<80;i++){             
        if (Sensor[i]>0)          //DEFINED so grab_ref(); Gets an image from Sensor666[] and computes new cRatios and brightness (*NO AVERAGING!*)
           grab_ref(bsn,Sensor666,S666_pitch,long(i*S666_pitch),&Sensor_ref[i*S666_pitch],&Sen_Brightness_Ref[i],&SensorRefRatio[i*12]);  //includes new Cratios & brightness
        averageRbsn=0;            //set a flag for main loop to compute a multi-second average Reference and update _ref[0-79]
        averageRcounter=AVCOUNT;  //initiate down counter.  (10 per second)               
      }     //end of r00
      printf("Ref: r00 refreshes ALL defined sensor references with new Cratios & brightness\n");         
      refBrightness=0;                      //recalculate a refBrightness from Sensor_ref - perhaps should ONLY do when r00 is executed not whenever ref(00) updated
      for(int i=0;i<S666_pitch;i++) refBrightness+=int(Sensor_ref[i]);    //sum S666_pitch pixlets for 6x6 Sensor[0] max 6804r02        
}   // ***************************** 

// FUNCTION TO COMPUTE NEW REF'S REGULARLY IF ACTIVE SENSORS NOT TRIPPED
void refRefresh(bool suspend){ 
  static bool lastsuspend=true;
  static int avRefAccumulation[48];
  static int avRefCtr=0;
  byte newRef[48]; 
  int i;
          //if suspend false, sequentially calculate new ref image for each enabled & vacant sensor
    if (!lastsuspend&&suspend) lastsuspend=true;
    if ( lastsuspend&&suspend) { Spr("SUS "); return; }   //return if SUSpension still in place
//    IFd9 printf("DEBUG-refRefresh suspend: %d lastsuspend: %d rbsn: %d force_refs: %d \n",suspend,lastsuspend,rbsn,force_refs);  
    if ( lastsuspend && !suspend) {       //on transition to !suspend, dump all averaging and restart               
      lastsuspend=false;                  
      for (i=0;i<80;i++) emptyStateCtr[i]= 0;   //initialise for reference averaging
      for (i=0;i<48;i++) avRefAccumulation[i]= 0;\
      avRefCtr=0;
      return;
    }else{ 
      avRefCtr++;                   //increment sample size 
      if (avRefCtr <= NUM2AVERAGE){ //continue accumulation
        for (i=0;i<48;i++)
          avRefAccumulation[i] += Sensor666[rbsn*48+i];  //accumulate 48 bytes (4x4x3)
      }
      else{         //accumulator full
        if(avRefCtr > NUM2AVERAGE+2){                 //ignore 2 frames then check trip
          if(force_refs>0) emptyStateCtr[rbsn]=1000;  //disables trip check     
          if(emptyStateCtr[rbsn] >NUM2AVERAGE+4){     //then good average (no trips)
            for (i=0;i<48;i++){                       //update ref
              newRef[i] = byte(avRefAccumulation[i]/NUM2AVERAGE);  //calc. average
              avRefAccumulation[i]=0;                 //clear accumulator
            }               //now calculate new reference parameters/ratios etc.
            grab_ref(rbsn,newRef,S666_pitch,0L,&Sensor_ref[rbsn*S666_pitch],&Sen_Brightness_Ref[rbsn],&SensorRefRatio[rbsn*12]);
          }else{            //there was a trip so abort rbsn attempt
            for (i=0;i<48;i++) avRefAccumulation[i]=0;   //clear accumulator
          }
          avRefCtr=0;                   //restart counter
          int oldrbsn=rbsn;
          if(rbsn>=force_refs) force_refs=0; //clear completed forced refs
          rbsn = (rbsn+1)%80;           //move to next sensor
          while(!SensorActive[rbsn]){   //skip to next enabled sensor
            if(rbsn>=force_refs) force_refs=0; //clear completed forced refs
            rbsn = (rbsn+1)%80;         
            if(rbsn==0) rbsn=1;         //leave sensor[00] to accumulate 64 samples for now
            if(rbsn==oldrbsn) break;    //abort if search finds no more active sensors
          }
        }
      }
    }
}   // *****************************

//FUNCTION TO CONVERT SENSOR TO LINEAR STYLE AND CREATE FOLLOWING STD SENSOR (iff NOT Sb7) 
bool makeLinear( int bsn, int deltaR, int deltaX,bool newS, int sign ) {  //return true if good
    // bsn:    number of sensor to convert to linear
    // deltaR: number of steps/pixels to next Sensor position (0 to 63)
    // deltaX: number of steps/pixels to next Sensor position (-63 to 63)
    // newS: if true, make a new Sensor, else don't.
    // sign: down=1 up=-1    
    if((deltaR>63) | (abs(deltaX)>63)) {Serial.print("steps too big\n"); return false;}       // size too big.
    if(Sensor[bsn]>(235-((deltaR*3)>>2))*FBPITCH)
      {deltaR=0; Serial.print('S');Serial.print(bsn,OCT);Serial.println(" line flattened at edge");}  //flatten line if going to  to edge
    pvtThreshold[bsn]=0xFF & (0x80 + (deltaR) );      ;   //record offsets
    lineardX[bsn] = sign*deltaX;
    SensorTwin[bsn]=0;
    SensorActive[bsn]=true;
    SensorActiveBlk[bsn>>3] |= mask[bsn&0x07];   
    if(newS && (bsn%8 != 7)) {         //create next sensor
      Sensor[bsn+1] = Sensor[bsn] + FBPITCH*sign*deltaR + deltaX*2;  //next sensor 2bytes/pixel (RGB565)
         //check if new sensor within bounds
      if(Sensor[bsn+1] > long((fbheight-5)*FBPITCH)-90) {       //note: no check on line length wrap around.
        Serial.print(Sensor[bsn+1]),Serial.println(" sensor[bsn+1] on bottom limit");    
        long rowAdjust=(Sensor[bsn+1]/FBPITCH-(fbheight-6))*FBPITCH;    
        Sensor[bsn+1]=(Sensor[bsn+1]-rowAdjust);        //crop row
      }                      //now complete new standard Sensor[bsn+1]
      pvtThreshold[bsn+1]=255;   //no pvtThreshold      
      SensorTwin[bsn+1]=0;  
      SensorActive[bsn+1]=true;
      SensorActiveBlk[(bsn+1)>>3] |= mask[(bsn+1)&0x07]; 
      grab_ref(bsn+1,Sensor666,S666_pitch,long((bsn+1)*S666_pitch),&Sensor_ref[(bsn+1)*S666_pitch],
       /* make new sensor legitimate non blank */             &Sen_Brightness_Ref[bsn+1],&SensorRefRatio[(bsn+1)*12]); 
    }  //now grab a new ref image for the converted sensor
    grab_ref(bsn,Sensor666,S666_pitch,long(bsn*S666_pitch),&Sensor_ref[bsn*S666_pitch],
      /* make new sensor legitimate non blank */           &Sen_Brightness_Ref[bsn],&SensorRefRatio[bsn*12]);      
    return true;
}   // *****************************

//FUNCTION TO PARSE ASCII COMMAND STRING
int parz(char *cmdString, int16_t *param){
  //cmdString: cmd ASCII character array with ',' or ' ' separators & ending with \n
  //param:  array of numParam integer parameters 
  //return numParam parameters in param[]. return 0 if NO valid parameters in cmdString
  int numParam=0; int j=0;
    if((cmdString[0]=='<') && (cmdString[1]=='N')) {   //accepts CS format '<N ' cmds. 
      cmdString[0]=' '; cmdString[1]=' ';              //  ( except <NR> and <NF> ) 
      for (j=0;j<6;j++) if(cmdString[j] != ' ') break; // find command position. Max 6 spaces 
      cmdString[j] = tolower(cmdString[j]);
    }
    for (j=0;j<6;j++) if(cmdString[j] != ' ') break;   // find command position. Max 6 spaces 
    if(!((cmdString[j] =='+')||(cmdString[j]=='&')||(cmdString[j]=='/')))   //accept +&/@~
      if((cmdString[j] < '@')||(cmdString[j]>'~')) return 0; // accept ascii 0x40 to 0x7E
    param[0]=cmdString[j];
    numParam=1; 
    for (int i=1;i<=5;i++) param[i] = -1;   //max 5 parameters
    int p = -1;
    for (int i=j+1;i<20;i++){  //max 20 characters \%%,##,##,#\n  Limited spaces
      if ( isDigit(cmdString[i]) ) {  //does not handle '-'
        if(p < 0) p=0;
        p=p*10 + (int(cmdString[i]) - '0');
      }
      else {     //non-digit
        if((cmdString[i]== '\n') || (cmdString[i]=='>')) break;     //end of command array
        if(!((cmdString[i]== ',')||(cmdString[i]== ' '))){ //not separator
          if((param[0]!='j')&&(param[0]!='y')) return 0;   //others must be numeric            
          if((cmdString[i] < '-')||(cmdString[i]>'z')) return 0;  //limit range
          p=cmdString[i];    //special case for 'j'&'yy' - accept alpha.
        }
        if(p>=0) {           //have a new parameter
          param[numParam]=p;
          numParam++;
          p=-1;    
        }
      }     
    }    
      // extracted all valid parameters
    if(p>=0) {         //add a new parameter
      param[numParam]=p;
      numParam++;
    }
      //convert 'b' in old style to new until deprecated
    if( (cmdString[0] == 'b')&&(param[1] > 9) ) {  
      param[2]=param[1]%10;
      param[1]=param[1]/10;
      numParam=3;
    } 
    return numParam;
}   // ***************************** 

//  FUNCTION TO EXTRACT BSN FROM DECIMAL PARZ INTERPRETATION
int bsN(int dec){
   //dec: %% bsNo that was parsed as decimal.
  if ((dec < 0)||(dec > 99)) return 00; //must return a valid bsn!
  int bsn = dec%10;           //extract "s" for bsNo
  if (bsn > 7) bsn=7;         //adjust if ends in '8' or '9'
  return ((dec/10)*8 + bsn);  //00-79.
}   // *****************************
   
