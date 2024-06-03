/*  30/MAY/24 
 *  © 2022, Peter Cole. All rights reserved.
 *  © 2023, Barry Daniel ESP32 revision 
 *  © 2024, Harald Barth. All rights reserved.
 *
 *  This file is part of EX-CommandStation
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
bool verPrint=true;
#define CAMver 201
// v201 deleted code for compatibility with CAM pre v171. Needs CAM ver201 with o06 only
// v200 rewrite reduces need for double reads of ESP32 slave CAM. Deleted ESP32CAP. 
//  Inompatible with pre-v170 sensorCAM, unless set S06 to 0 and S07 to 1 (o06 & l07 say)
/*
* The IO_EXSensorCAM.h device driver can integrate with the sensorCAM device.
* It is an extension on the IO_EXIOExpander.h device driver to include specific needs of the ESP32 sensorCAM 
* This device driver will configure the device on startup, along with
* interacting with the device for all input/output duties.
*
* To create EX-SensorCAM devices, define them in myHal.cpp:
* (Note the IOExpander device driver is included in CS by default)
*
* void halSetup() {
*   // EXSensorCAM::create(vpin, num_vpins, i2c_address);
*   EXSensorCAM::create(700, 80, 0x11);
* }
* 
* All pins on an EX-IOExpander device are allocated according to the pin map for the specific
* device in use. There is no way for the device driver to sanity check pins are used for the
* correct purpose.
*
* I2C packet size of 32 bytes (in the Wire library).
*/
# define DIGITALREFRESH 25000UL              // min uSec between digital reads EXIORDD
#ifndef IO_EX_EXSENSORCAM_H
#define IO_EX_EXSENSORCAM_H


#define CAMERR 0xEF
 
#define Sp Serial.print

#include "IODevice.h"
#include "I2CManager.h"
#include "DIAG.h"
#include "FSH.h"
size_t digitalBytesNeeded;
uint8_t _CAMresponseBuff[34];
int countB=0;
/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * IODevice subclass for EX-SensorCAM.
 */
class EXSensorCAM : public IODevice {
public:
  enum ProfileType : uint8_t {
    Instant = 0,  // Moves immediately between positions (if duration not specified)
    UseDuration = 0, // Use specified duration
    Fast = 1,     // Takes around 500ms end-to-end
    Medium = 2,   // 1 second end-to-end
    Slow = 3,     // 2 seconds end-to-end
    Bounce = 4,   // For semaphores/turnouts with a bit of bounce!!
    NoPowerOff = 0x80, // Flag to be ORed in to suppress power off after move.
  };
  static void create(VPIN vpin, int nPins, I2CAddress i2cAddress) {
    if (checkNoOverlap(vpin, nPins, i2cAddress)) new EXSensorCAM(vpin, nPins, i2cAddress);
  }

private:
  // Constructor
  EXSensorCAM(VPIN firstVpin, int nPins, I2CAddress i2cAddress) {
    _firstVpin = firstVpin;
    // Number of pins cannot exceed 256 (1 byte) because of I2C message structure.
    if (nPins > 256) nPins = 256;
    _nPins = nPins;
    _I2CAddress = i2cAddress;
    addDevice(this);
  }
  //*************************
  void _begin() {
    uint8_t status;
    // Initialise EX-SensorCAM device
    I2CManager.begin();
    if (I2CManager.exists(_I2CAddress)) {
      // Send config, if EXIOPINS returned, we're good, setup pin buffers, otherwise go offline
      // NB The I2C calls here are done as blocking calls, as they're not time-critical
      // during initialisation and the reads require waiting for a response anyway.
      // Hence we can allocate I/O buffers from the stack.
      uint8_t receiveBuffer[3];
      uint8_t commandBuffer[4] = {EXIOINIT, (uint8_t)_nPins, (uint8_t)(_firstVpin & 0xFF), (uint8_t)(_firstVpin >> 8)};
      status = I2CManager.read(_I2CAddress, receiveBuffer, sizeof(receiveBuffer), commandBuffer, sizeof(commandBuffer));
/***/ // If ESP32 CAM read again for good data
//      if (_I2CAddress<=ESP32CAP)
      status = I2CManager.read(_I2CAddress, receiveBuffer, sizeof(receiveBuffer), commandBuffer, sizeof(commandBuffer));
      if (status == I2C_STATUS_OK) {
        if (receiveBuffer[0] == EXIOPINS) {
          _numDigitalPins = receiveBuffer[1];
          _numAnaloguePins = receiveBuffer[2];
          // allocate space for sensorCAM packet buffers
          _tempBuf = (byte*) calloc(32, 1);               //is this still used??
    //      _CAMresponseBuff = (byte*) calloc(32, 1);
          // See if we already have suitable buffers assigned
          if (_numDigitalPins>0) {
            digitalBytesNeeded = (_numDigitalPins + 7) / 8;
            if (_digitalPinBytes < digitalBytesNeeded) {
              // Not enough space, free any existing buffer and allocate a new one
              if (_digitalPinBytes > 0) free(_digitalInputStates);
              if ((_digitalInputStates = (byte*) calloc(digitalBytesNeeded, 1)) != NULL) {
                _digitalPinBytes = digitalBytesNeeded;
              } else {
                DIAG(F("EX-IOExpander I2C:%s ERROR alloc %d bytes"), _I2CAddress.toString(), digitalBytesNeeded);
                _deviceState = DEVSTATE_FAILED;
                _digitalPinBytes = 0;
                return;
              }							
            }																					 																													
          }
      // some EXIOExpander code deleted (no use for CAM)
      } else {   
          DIAG(F("EX-SensorCAM I2C:%s ERROR configuring device"), _I2CAddress.toString());
          _deviceState = DEVSTATE_FAILED;
          return;
        }
      }
      // There is no analogue pin map to retrieve

      if (status == I2C_STATUS_OK) {
        // Attempt to get version, if we don't get it, we don't care, don't go offline
        uint8_t versionBuffer[3];
        commandBuffer[0] = EXIOVER;
        status = I2CManager.read(_I2CAddress, versionBuffer, sizeof(versionBuffer), commandBuffer, 1); 
/***/ // If ESP32 CAM read again for good data
//        if (_I2CAddress<=ESP32CAP)
          status = I2CManager.read(_I2CAddress, versionBuffer, sizeof(versionBuffer), commandBuffer, 1);
        if (status == I2C_STATUS_OK) {
          _majorVer = versionBuffer[0];
          _minorVer = versionBuffer[1];
          _patchVer = versionBuffer[2];
        }
        DIAG(F("EX-SensorCAM device found, I2C:%s, Version v%d.%d.%d"),
            _I2CAddress.toString(), _majorVer, _minorVer, _patchVer);
      }
      if (status != I2C_STATUS_OK)
        reportError(status);
    } else {
      DIAG(F("EX-SensorCAM I2C:%s device not found"), _I2CAddress.toString());
      _deviceState = DEVSTATE_FAILED;
    }
  }
  //*************************
  // Digital input pin configuration, used to enable on EX-IOExpander device and set pullups if requested.
  // Configuration isn't done frequently so we can use blocking I2C calls here, and so buffers can
  // be allocated from the stack to reduce RAM allocation.
  bool _configure(VPIN vpin, ConfigTypeEnum configType, int paramCount, int params[]) override { 
    if(verPrint) DIAG(F("_configure() IO_EXSensorCAM v0.%d.%d vpin: %d "), CAMver/100,CAMver%100,vpin);
    verPrint=false;
    if (paramCount != 1) return false;
  // EXIOExpander code deleted (no use for CAM)
    return true; //at least confirm that CAM is (always) configured (no vpin check!)
  }
  //*************************															 
  // Analogue input pin configuration, used to enable an EX-IOExpander device.
  // Use I2C blocking calls and allocate buffers from stack to save RAM.
  int _configureAnalogIn(VPIN vpin) override { 
    DIAG(F("_configureAnalogIn() IO_EXSensorCAM vpin %d"),vpin); 
  // EXIOExpander code deleted (no use for CAM)
    return false;
  }
  //*************************
  // Main loop, collect both digital and analogue pin states continuously (faster sensor/input reads)
  void _loop(unsigned long currentMicros) override {
    if (_deviceState == DEVSTATE_FAILED) return;    // If device failed, return
      // Request block is used for "analogue" (cmd. data) and digital reads from the sensorCAM, which are performed
      // on a cyclic basis.  Writes are performed synchronously as and when requested.

    if (_readState != RDS_IDLE) {                  //expecting a return packet
      if (_i2crb.isBusy()) return;                 // If I2C operation still in progress, return
      uint8_t status = _i2crb.status;
      if (status == I2C_STATUS_OK) {               // If device request ok, read input data

        //there should be a packet in _CAMresponseBuff[32] 
/***/ //  countB+=1; Sp(_CAMresponseBuff[0],HEX);Sp(' '); if (countB>40){countB=0;Sp('\n');}
            
        if ((_CAMresponseBuff[0] & 0x40) != 0) {   //response seems to have ascii cmd header (bit6 set) (o06)
                      // process received values according to header ('@', 't', 'm','i' etc)     
            int error = processIncomingPkt( _CAMresponseBuff, _CAMresponseBuff[0]);   // '@' 'i' 'm' 't' etc
            if (error>0) DIAG(F("CAM packet header not recognised 0x%x"),_CAMresponseBuff[0]);
                           
        }else{ // Header not valid - mostly replaced by bank 0 data.  To avoid any bad response latch S06 to 0 ( o06 ) 
            //versions of sensorCAM.h after 170 should return header of EXIORDD(0xE6) or '@'(0x40) with sensor state array
   /***/    if (_CAMresponseBuff[0]!=0){ Serial.print(_CAMresponseBuff[0],HEX);Serial.print('~');
    //        DIAG(F("i2c header not valid")); 
    //        for(int i=0; i<int(digitalBytesNeeded);i++)  _digitalInputStates[i] = _CAMresponseBuff[i];  //memcpy??
          }else{     //timeout ?  Ignore?

          }
        }
      }  else   reportError(status, false);   // report eror but don't go offline.
      _readState = RDS_IDLE;
    }      

  
    // If we're not doing anything now, check to see if a new EXIORDD transfer, or for 't' repeat, is due.
    if (_readState == RDS_IDLE) {
     
       //check if time for digitalRefresh
      if (_numDigitalPins>0 && currentMicros - _lastDigitalRead > _digitalRefresh) { // Delay for digital read refresh
        // Issue new read request for digital states.  As the request is non-blocking, the buffer has to
        // be allocated from heap (object state).
             
        _readCommandBuffer[0] = EXIORDD;       //start new EXIORDD
        I2CManager.read(_I2CAddress, _CAMresponseBuff, 32 /*(_numDigitalPins+7)/8*/, _readCommandBuffer, 1, &_i2crb);  // non-blocking read      

        _lastDigitalRead = currentMicros;
        _readState = RDS_DIGITAL;
        
      }else{    //slip in a repeat <NT n> if pending
        if (currentMicros - _lastAnalogueRead > _analogueRefresh)  // Delay for "analogue" commands
         if (_savedCmd[2]>1) {  
                    //repeat a 't' command         
          for (int i=0;i<7;i++)  _readCommandBuffer[i] =_savedCmd[i];
          int errors = ioESP32(_I2CAddress, _CAMresponseBuff, 32, _readCommandBuffer, 7);    
          _lastAnalogueRead = currentMicros;
          _savedCmd[2] -=1;     //decrement repeats
                  
          if (errors==0){                     
              return;
          }else {
            DIAG(F("ioESP32 error %d header 0x%x"),errors,_CAMresponseBuff[0]);  
          }
          _readState = RDS_ANALOGUE;       //this should stop further cmd requests until a packet read (inc. timeout)
        }
      }   //end repeat 't'
    }
  }      //end void loop()
  //*************************
  // Obtain the correct analogue input value, with reference to the analogue
  // pin map.  
  // Obtain the correct analogue input value
  int _readAnalogue(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    int pin = vpin - _firstVpin;
//  if(_I2CAddress <= ESP32CAP){        
      uint8_t pinByte = pin / 8;      //if ESP32 (CAM), return bank status byte
      int     value = _digitalInputStates[pinByte];
      return  value;  
//  }    
  // EXIOExpander code deleted (no use for CAM)
//  return -1;  // pin not found in table
  }
  //*************************
  // Obtain the correct digital input value
  int _read(VPIN vpin) override {
    if (_deviceState == DEVSTATE_FAILED) return 0;
    int pin = vpin - _firstVpin;
    uint8_t pinByte = pin / 8;
    bool value = bitRead(_digitalInputStates[pinByte], pin - pinByte * 8);
    return value;
  }
  //*************************
  // Write digital value.  
  void _write(VPIN vpin, int value) override { 
    DIAG(F("**_write() vpin %d = %d"),vpin,value);
    return ;
  }
  //*************************
  // ESP32 requires special code to get valid data and interpret appropriately
  // i2cAddr of ESP32 CAM
  // rBuff   buffer for return packet 
  // inbytes number of bytes to request from CAM 
  int ioESP32(uint8_t i2cAddr,uint8_t *rBuf,int inbytes,uint8_t *outBuff,int outbytes) {
    // only for commands translated or responding with EXIORDY i.e. EXIODPUP(E2) EXIOWRD(E5) EXIOENAN(E7)
    // these sensorCAM commands can invoke a detailed return data packet
    //                      0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19}; 
const char translate2[20]={'-','-','-','-','-','-','-','-','i','t','-','-','-','-','m','-','-','-','-','-'};
const int returnBytes[20]={ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 9 ,32 , 0 , 0 , 0 , 0 , 7 , 0 , 0 , 0 , 0 , 0 };

int valu;
int valu2;
uint8_t cmdNo;  
uint8_t status = _i2crb.status;

//    if(i2cAddr > ESP32CAP) return 0;  //do nothing if not ESP32 
 
 uint8_t sensorCmd = '-';
      cmdNo=outBuff[4];           //(profile)
      if (cmdNo>=240) {           //if "row" >239 treat as a CAM command irrespective of vpin
        cmdNo=cmdNo-240;
        outBuff[1]=00;            //should already be 00 from myFilter      
      }
      if ((outBuff[1]==00) && (outBuff[4]>=240)){   //for vpin 00 treat as command
        sensorCmd=translate2[cmdNo];        
        if (sensorCmd != '-'){    //translate to ascii. c##[,$$] format   
          valu=outBuff[2]+(outBuff[3]<<8);
      int fst1=1; 
          if(sensorCmd=='m'){     // m$[,##]
            fst1=0;               
            if (valu>=5) valu=valu+1000;  //force $ to '0' if 5-99
          }       
          if(valu>100){
             valu2=valu%100;      
             valu=valu/100;
             outBuff[fst1+2]=',';
             outBuff[fst1+3]=valu2/10+0x30;
             outBuff[fst1+4]=valu2%10+0x30;       //outbytes can still be 7
          } 
          outBuff[fst1+0] = valu/10+0x30;         //extract high bsNo. ascii digit 
          outBuff[fst1+1] = valu%10+0x30;         //extract low bsNo. ascii digit
          outBuff[0] = sensorCmd;
          inbytes   = returnBytes[cmdNo];
        } 
      }
       if (status != I2C_STATUS_OK){ 
        DIAG(F("i2c bus busy.  Try again."));
        return status;
      }
  //    DIAG(F("num. bytes requested: %d out:%d "),inbytes,outbytes);
      status = I2CManager.read(i2cAddr, rBuf, inbytes, outBuff, outbytes);
          
      if (status != I2C_STATUS_OK){ 
        DIAG(F("EX-SensorCAM I2C:%s Error:%d %S"), _I2CAddress.toString(), status, I2CManager.getErrorMessage(status));
        reportError(status); return status;}
      return 0;  // 0 for no error != 0 for error number.
   }
  //*************************
//function to read and interpret packet from sensorCAM.ino
//i2cAddr to identify CAM# (if # >1)
//rBuf contains packet of up to 32 bytes possibly with (ascii) cmd in rBuf[0]
//inbytes max bytes in pkt if known
//outBuff location for _digitalInputStates[]; table
//sensorCmd command header byte from CAM (in rBuf[0]?)
  int processIncomingPkt(uint8_t *rBuf,uint8_t sensorCmd) {
  int k;
   //    DIAG(F("header 0x%x"),sensorCmd);
   //if (rBuf[0] != EXIORDY) {   //perhaps got back ascii cmd
   //   if (rBuf[0] == sensorCmd){
          switch (sensorCmd){
            case EXIORDD:   //response to request for digital table    
              for (int i=0; i<int(digitalBytesNeeded);i++)  _digitalInputStates[i] = rBuf[i+1];
              break;   
            case EXIORDY:   //some commands give back acknowledgement only
              break;
            case CAMERR:  //cmd format error code from CAM
              DIAG(F("CAM cmd error 0xFE 0x%X"),rBuf[1]);
              break;
            case'i':    //information from i%%
			        k=256*rBuf[5]+rBuf[4];
              DIAG(F("(i%%%%[,%%%%]) Info: Sensor 0%o(%d) enabled:%d status:%d  row=%d x=%d Twin=0%o A~%d"),
                      rBuf[1],rBuf[1],rBuf[3],rBuf[2],rBuf[6],k,rBuf[7],int(rBuf[8])*16);
              break;
            case 'm':
              DIAG(F("(m$[,##]) Min/max: $ frames min2flip (trip) %d, minSensors 0%o, maxSensors 0%o, nLED %d, threshold %d TWOIMAGE_MAXBS 0%o"),
                      rBuf[1],rBuf[2],rBuf[3],rBuf[4],rBuf[5],rBuf[6]);                                                               
              break;            
            case 't':                                            //threshold etc. from t##
                //print raw pkt
                //for (int j=0; j<32; j++) {Sp(rBuf[j],HEX);Sp(' ');} Serial.println(' ');
              //bad pkt if 't' FF's
              if(rBuf[1]==0xFF) {Serial.println("bad 't' packet: 74 FF");_savedCmd[2] +=1; return 0;}  //ignore bad packet
              Sp("(t[##[,##]]) Threshold:"); Sp(rBuf[1]);Sp(" sensor S00:"); Sp("-"); k=rBuf[2]&0x7F; if(k>99)k=99; Sp(k);
              if(rBuf[2]>127) Sp("##* "); 
              else{ 
                if(rBuf[2]>rBuf[1]) Sp("-?* "); 
                else Sp("--* ");
              }
              for(int i=3;i<31;i+=2){
                uint8_t valu=rBuf[i];        //get bsn
                if(valu==80) break;          //80 = end flag
                else{  
                  if((valu&0x7F)<8)Sp("0");Sp(valu&0x7F,OCT);Sp(':');if(valu>=128)Sp("?-"); else {if(rBuf[i+1]>=128)Sp("oo");else Sp("--");}
                  valu=rBuf[i+1]; k=valu&0x7F; if(k>99)k=99; Sp(k);if(valu<128)Sp("--* "); else Sp("##* ");
                }
              } 
              Serial.print('\n');
              break;
            default:   //header not a listed character
			        Serial.print(rBuf[0],HEX);DIAG(F("CAM packet header not valid (0x%x) (0x%x) (0x%x)"),rBuf[0],rBuf[1],rBuf[2]);
              return 1;
          }
          return 0;  
  }
  //*************************											  
  // Write analogue (command) values.  Write the parameters to the sensorCAM
  //   writeAnalogue(VPIN vpin, int value, uint8_t profile, uint16_t duration)
  void _writeAnalogue(VPIN vpin, int value, uint8_t parameter1, uint16_t parameter2) override {
    uint8_t servoBuffer[7];
    int errors=0;
    if (_deviceState == DEVSTATE_FAILED) return;
    int pin = vpin - _firstVpin;
    servoBuffer[0] = EXIOWRAN;
    servoBuffer[1] = pin;
    servoBuffer[2] = value & 0xFF;
    servoBuffer[3] = value >> 8;
    servoBuffer[4] = parameter1;
    servoBuffer[5] = parameter2 & 0xFF;
    servoBuffer[6] = parameter2 >> 8;

      if(servoBuffer[4]==249){   //then 't' cmd
        if(value<31) {   //repeated calls if param < 31
            for (int i=0;i<7;i++) _savedCmd[i]=servoBuffer[i];
        }else _savedCmd[2] = 0;   //no repeats if ##>30 
      }else _savedCmd[2] = 0;       //no repeats unless 't'
      
      _lastAnalogueRead = micros();    //don't repeat until _analogueRefresh mSec

      errors = ioESP32(_I2CAddress, _CAMresponseBuff, 32 , servoBuffer, 7);   			    			   
      if (errors==0) return;
      else {DIAG(F("ioESP32 error %d header 0x%x"),errors,_CAMresponseBuff[0]);  
        if (_CAMresponseBuff[0] != EXIORDY) {      //can't be sure what is inBuff[0] !
          DIAG(F("Vpin %u cannot be used as a servo/PWM pin"), (int)vpin);
        }
      }
  }
  //*************************  // Display device information and status.
  // Display device information version and status. 
  void _display() override {
    DIAG(F("EX-SensorCAM I2C:%s v%d.%d.%d Vpins %u-%u %S"),
        _I2CAddress.toString(), _majorVer, _minorVer, _patchVer,
        (int)_firstVpin, (int)_firstVpin+_nPins-1,
        _deviceState == DEVSTATE_FAILED ? F("OFFLINE") : F(""));
  }
  //*************************
  // Helper function for error handling
  void reportError(uint8_t status, bool fail=true) {
    DIAG(F("EX-SensorCAM I2C:%s Error:%d (%S)"), _I2CAddress.toString(), 
      status, I2CManager.getErrorMessage(status));
    if (fail)
      _deviceState = DEVSTATE_FAILED;
  }
  //************************* 
  uint8_t _numDigitalPins = 0;
  uint8_t _numAnaloguePins = 0;

  uint8_t _majorVer = 0;
  uint8_t _minorVer = 0;
  uint8_t _patchVer = 0;
  uint8_t _savedCmd[8]; //for repeat 't' command
  uint8_t* _digitalInputStates  = NULL;
  uint8_t* _tempBuf = NULL;          //_analogueInputStates = NULL;
  uint8_t _readCommandBuffer[8];

  uint8_t _digitalPinBytes = 0;   // Size of allocated memory buffer (may be longer than needed)
  uint8_t _analoguePinBytes = 0;  // Size of allocated memory buffer (may be longer than needed)
  uint8_t* _analoguePinMap = NULL;
  I2CRB _i2crb;

  enum {RDS_IDLE, RDS_DIGITAL, RDS_ANALOGUE};  // Read operation states
  uint8_t _readState = RDS_IDLE;
  uint8_t cmdBuffer[7]={0,0,0,0,0,0,0};
  unsigned long _lastDigitalRead = 0;
  unsigned long _lastAnalogueRead = 0;
        unsigned long _digitalRefresh = DIGITALREFRESH;    // Delay refreshing digital inputs for 10ms
  const unsigned long _analogueRefresh = 130000UL;   // Delay refreshing repeat "analogue" inputs

  // EX-IOExpander protocol flags
  enum {
    EXIOINIT = 0xE0,    // Flag to initialise setup procedure
    EXIORDY = 0xE1,     // Flag we have completed setup procedure, also for EX-IO to ACK setup
    EXIODPUP = 0xE2,    // Flag we're sending digital pin pullup configuration
    EXIOVER = 0xE3,     // Flag to get version
    EXIORDAN = 0xE4,    // Flag to read an analogue input
    EXIOWRD = 0xE5,     // Flag for digital write
    EXIORDD = 0xE6,     // Flag to read digital input
    EXIOENAN = 0xE7,    // Flag to enable an analogue pin
    EXIOINITA = 0xE8,   // Flag we're receiving analogue pin mappings
    EXIOPINS = 0xE9,    // Flag we're receiving pin counts for buffers
    EXIOWRAN = 0xEA,   // Flag we're sending an analogue write (PWM)
    EXIOERR = 0xEF,     // Flag we've received an error
  };
};

#endif
