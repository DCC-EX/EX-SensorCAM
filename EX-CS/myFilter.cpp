#include "Arduino.h"
#include "IODevice.h"
#ifndef SENSORCAM_OPCODE      //if not 'U', define in config.h
#define SENSORCAM_OPCODE 'U'
#endif
#ifndef SENSORCAM_VPIN0       //if not 700, define in config.h
#define SENSORCAM_VPIN0 700
#endif
int CAMbasevPin = SENSORCAM_VPIN0;

void myFilter(Print * stream, byte & opcode, byte & paramCount, int16_t p[])
{
  const char cmds[16]={'o','l','a','n','r','s','u','f','i','t','x','w','g','e','m','v'};
  int16_t param1;
  int16_t param2;
  uint8_t param3;
  unsigned int i;

  if ((opcode == SENSORCAM_OPCODE)||(opcode=='N')){    
    if (paramCount == 1){ 
      param3 = p[0];      
      for(i=0;i<sizeof(cmds);i++) 
	    if(cmds[i]==(param3+0x20)) param3=i+240;  //convert ascii to code
      DIAG(F("CAM: %c =%d"), cmds[param3%240], param3);
      IODevice::writeAnalogue(CAMbasevPin, 0, param3);     
    }
    if(paramCount == 2){ 
      if(p[0]=='C') {       //use 'C' to switch between multiple CAMs
        CAMbasevPin=p[1];
        DIAG(F("CAM base vpin: %c %d "),p[0],p[1]);
      }else{  
        param2 = p[1];
        param3 = p[0];
        for(i=0;i<sizeof(cmds);i++) if(cmds[i]==param3+0x20) param3=i+240;     //convert ascii to code
        DIAG(F("CAM: %c %d "), cmds[param3%240], param2);     
        IODevice::writeAnalogue(CAMbasevPin, param2, param3); 
      }    
    }
    if (paramCount == 3){   //reserved for vpin rowY colx, default 'a'
      param1 = p[0];        //vpin int16_t
      param2 = p[2];        //column int16_t
      if(p[1]<234) param3 = p[1];    //row byte
      else param3 = 234;    //limit row to 234
      // reverse param2 & 3 as column must go first to cover 0-319 
      DIAG(F("CAM: %c %d %d %d"), 'A', param1, param3, param2);
      IODevice::writeAnalogue(param1, param2, param3);
    }
    if (paramCount == 4){       //for 'a id row col' 
      if (p[0]=='a'-0x20){      //must start with 'a' or 'A'
        i = p[1]%100;           //ensure reasonable bsNo.
        param1 = CAMbasevPin+(i/10)*8+((i%10)&0x07);  //convert bsNo to vpin  
        if(p[3]<317) param2 = p[3];            
        else param2 = 316;            //column int16_t
        if(p[2]<237) param3 = p[2];   //row byte
        else param3 = 236;            //limit row to 236 
        i=param1-CAMbasevPin;
        DIAG(F("CAM: a%d%d,%d,%d"),i>>3,i&0x7, param3, param2);
        IODevice::writeAnalogue(param1, param2, param3);
      } 
      else DIAG(F("Invalid cmd: %d %d %d %d"),p[0],p[1],p[2],p[3]);
    }
    opcode = '\0';
  }
}
