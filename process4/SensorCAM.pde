
/* 2026/3/22  version 207
 *
 * sensorCAM Serial Monitor 
 * by Barry Daniel
 * 
 * Lets you do simple edits of a command line
 * Sends a line out the serial port when Enter key pressed
 * listens for bytes received, and displays their value.
 * sensorCAM command enhancements for imaging capability
 */

import processing.serial.*;

int comNo=0;  //0 for first COM port. May force another port e.g. =1 for second port.
final int SF=2;  // image scale factor (2-4) (4 for highest resolution screens) 

String VER = "SensorCAM RGB565 320 x 240   Monitor v207 for Processing4";
int BAUD=115200;
Serial myPort;         // The serial port
int baud = 115200;

int whichKey = '\n';   // Variable to hold keystoke values
int inByte = -1;       // Incoming serial data
int chr=0;         

char inLine[] = new char[80];     //80 character command buffer
int i=0;
int j=0;
boolean echo=true;     //^E toggles
boolean S3=false;      //^S toggles
int red;               //pixel
PImage img;            //processing 4 screen images
PImage img2;
PImage mouseCursor;
int cursr  =  '_';  //flashable
int oldcursr= 0x20; //space
char cursorChar = '_';  //'▮';   // U+25AE vertical block cursor

boolean newS = false;
int coordX=-1, coordY=-1;
int WidithRequest=240; //number of rows-height.  240x320 frame max. 
int xStartPt=0;        //image request start column default value
int Yrow=0;            //image request(Y) start row.
int Zlength=320;       //No of columns/pixels per line

int  mirrorX=1;        //NO mirror if =1
int  mirrorY=1;

char[] inBuf = new char[1024]; //Buffer for image data from USB

int timeoutMax = 120;          //mSec
int timeout=timeoutMax;        //start timeout counter
boolean verbose=false;					  

void settings(){
  size(330*SF,270*SF);
}

void setup() { 
  //  size(990,810); //330*SF, 270*SF); //must be 1st line of setup() or settings() 
  
    print("choosing comNo ");println(comNo);
  
//  mouseCursor = loadImage("8pixSQ.png");  could try a special cursor 
  // create a font with the third font available to the system:
  
  PFont myFont = createFont("Consolas", 16);    
//    PFont myFont = createFont(PFont.list()[147], 16);
    textFont(myFont,16);
    background(0);
    fill(204,102,0);
    rect(10,30,320*SF,240*SF);
    fill(255,255,255);            //colour for headings

//create a test background image  
    img = createImage(320, 240, ARGB);
    colorMode(RGB, 255, 255, 255);
    int r=100; int b=100; int g=100;
    for(j = 0; j < img.pixels.length; j++) {
      b=(j & 0x1F)<<3;      //extract colour and make 8 bit.
      g=(j >> 3)& 0xFA;
      r=(j >> 8)& 0xF8;
      img.pixels[j] = color(g,b,r);     //writes one pixel (at j)   reversed colour for effect
    }

    image(img, 10, 30,320,240); //width/2,height/2);           //draw background image on display window
  
  // List all the available serial ports:
    printArray(Serial.list());

  // Try Serial.list()[0]. (the first available COM port?)
  // Open whatever #(comNo) port you're using for sensorCAM.

    String portName = Serial.list()[comNo];
    print("using "); println(portName);
  
    text("Cmd: ", 10, 264*SF);  //was 530
    text(VER, 10, 20);  
    myPort = new Serial(this, portName, BAUD,'N',8,1);
    myPort.setDTR(false);      //GPIO0 high 3.3V
    myPort.setRTS(false);      //EN high?            (both low triggers ESP32 reset!

    print("end setup()\n");
    delay(2000);
}

void draw() {      // loops continuously

  while (myPort.available() > 0) {  // just display any USB incoming characters
    inByte = myPort.read();
    print(char(inByte));
  }
 
  if ((inLine[0] == 'k') || (inLine[0] == 'a')){    //set cursor for sensor creation
    if (i>=3 && mouseY <  250*SF && mouseY > 30 && mouseX >10) {
      cursor(HAND); // cursor(mouseCursor,0,0);
      newS = true;
      if (coordX >= 0) {          //create command for new sensor
        print (" make sensor Cmd: ");
        i=3;                      //always add after "a%%"
        inLine[i]=','; i++;    
        String strng =str(coordY);         //row
        char[] posn = strng.toCharArray(); 
        arrayCopy(posn,0,inLine,i,posn.length);
        i=i+posn.length;
        inLine[i]=','; i++;
        strng = str(coordX);
        posn = strng.toCharArray();
        arrayCopy(posn,0,inLine,i,posn.length);
        i=i+posn.length;    
        coordX=-1;                  //don't let it repeat
      
        fill(200);                    //clear and rewrite command line
        rect(45,256*SF,303*SF,12*SF); //was 512,605,25
        fill(0);
        for(int j=0;j<i;j++)  text(inLine[j],50+j*10,264*SF);
        text(char(1),50+i*10,264*SF);    //try to add a claytons "cursor" 
        String aCmd= new String(inLine);
        print(aCmd);
      }
    }
    else  {cursor(CROSS); newS=false; }     
  }
  else  {cursor(CROSS);newS=false;}
  
  chr=whichKey;    
  if(chr != 0){     //print (char(whichKey));   //accept new key
    if(chr > 0x7F || chr < 0) chr=0;     //rejects
    else{                                //key is ASCII
       if(echo) print(char(chr));
       inLine[i] = char(chr);

       if (inLine[i] == '\n')            // LF so send command including LF
       { if (myCommand(inLine[0],i)) {  //identify and process special commands W X Y Z
           print("done\nP4-Cmd: ");
           for(int j=0;j<i+1;j++) print(inLine[j]);  //prints myCmd AFTER command execution.
         } else {            //send command to CAM
           println(" ");      
           for (int j=0;j<i+1;j++) myPort.write(inLine[j]);
           if((inLine[0]== 'a')&&(i>7))              //new sensor  created
             drawNewSensor(10);
         }
         i=0;
       }else{                      //NOT linefeed
         if (inLine[i] == 8) {     // BS correction
           if( i>0 ) i--;   
         }else{
           if( i < 39) i++;        //limit line length
         }
       } 
       whichKey=0;      
       delay(100);
       fill(200);                     //clear and rewrite command line
       rect(45,256*SF,303*SF,12*SF);  //was 512,605,25
       fill(0);
       for(int j=0;j<i;j++){  
         text(inLine[j],50+j*10,264*SF);
       text(char(cursorChar),50+i*10,264*SF);  //add a claytons ^A "cursor"
       } 
    }
  }    
  if((millis() & 0x1FF) < 0x100) cursr=0x20;   //try to flash cursor.
     else cursr = cursorChar;                   
  if(!S3 && (cursr != oldcursr)) { oldcursr=cursr;  //redraw command line
    fill(200);                                 //clear and rewrite command line
    rect(45,256*SF,303*SF,12*SF);    
    fill(0);
    for(int j=0;j<i;j++)  
      text(inLine[j],50+j*10,264*SF);
    text(char(cursr),50+i*10,264*SF);
  }
}

void drawNewSensor(int size){        //redraws image around new sensor (with box)
int savedWidithRequest=240;
int savedYrow=0;
int aRow=10;
int xStart=0;
      delay(120);
      aRow=getNum(4);                    //get 'a' row parameter
      savedWidithRequest=WidithRequest;
      WidithRequest=size;
      savedYrow=Yrow;
      Yrow=aRow-2;
      if(aRow<10) xStart=getNum(6)-2;
      else xStart=getNum(7)-2;  
      sendCmd('x',xStart);
      sendCmd('z',size);
      executeYcmd();                     //redraw 10x10 image       
      WidithRequest=savedWidithRequest;  //restore settings
      sendCmd('x',xStartPt);
      sendCmd('z',Zlength);
      Yrow=savedYrow;
}

void keyPressed() {       // check & return the keystroke
    int ch = key; 
    whichKey = 0;         // check for ctrl keys first
    switch(ch) {          // for debugging only
      case 0x05:          // ^E for toggle echo on/off
        echo=!echo;
        println("P4^E echo ",echo); 
        break;
      case 0x12:          // ^R for SensorCAM Reset
        print("REBOOTING CAM\n");
        myPort.setRTS(true);   //'S' EN high stops FTDI
        myPort.setDTR(true);   //'D' GPIO0 low for download!S
        myPort.setDTR(false);  //'R' GPIO0 highTT
        myPort.setRTS(false);  //'T' EN low as well triggers reboot
        break;
      case 0x13:          // ^S for S3 toggle (cursor flash)
        S3 = !S3;
        cursr = cursorChar;    // ensure curser left as visible
        if(S3) println("esp32-S3 mode true"); 
        else  println("normal esp32-S mode");
        break;
      case 0x16:          // ^V for toggle verbose/quiet mode
        verbose=!verbose;
        if(verbose) print("verbose mode ");
        else print("quiet mode ");
        break;
      default:
        whichKey = ch;
    }
}

boolean myCommand(char cmd,int nch) { //execute primary (capital) commands
  char request[] = new char[80];      //80 character request buffer

  switch (cmd) {
    case 'H':            //mirror x
      mirrorX = -mirrorX;
      return true;
     
    case 'V':            //mirror y
      mirrorY = -mirrorY;
      return true;
     
    case 'W':            //set a limited useful image strip width (height)
      WidithRequest = getNum(1);   //nothing to send as only # of repeats of 'y'
      if(WidithRequest < 0) WidithRequest=240;
      return true;
  
    case 'Z':            //get useful image width
      Zlength = getNum(1);
      if(Zlength < 0) Zlength=320; //try default row length
      sendCmd('z',Zlength);
      println(Zlength);
      return true;  
   
    case 'X':                 //set left side of useful image
      xStartPt = getNum(1);
      if(xStartPt < 0) xStartPt=0;
      sendCmd('x',xStartPt);
      println(xStartPt);
      return true;
     
    case 'Y':                 //set top row for y, and fill image
      if(inLine[1] == '0') {  //could reprogram to use ^V ?
        verbose=!verbose;
        if(verbose) print("verbose mode ");
        else print("quiet mode ");
        return true;
      }	
      if(inLine[1] > 0x1F) Yrow = getNum(1);   //from command line
      else Yrow=0;                             //default from top line	  
      if(Yrow < 0) Yrow=0;
      if(verbose) println(Yrow);
      return executeYcmd();
      
    case 'y':         //don't let accidental lower case "Y" through.
      if(verbose)return false;   //let (diagnostic?)'y' go through
      println(" y & yy command disabled in non-verbose mode, Use Y"); 
      return true;
         
    default:     //no Process commands recognised
      return false;  
  }    
}     //end myCommand()   

 
//fetch an image strip at Yrow (WidithRequest rows wide, starting at xStartPt column, ending after Zlength pixels) 
boolean executeYcmd() { 
/***/   if(verbose) println(" executeYcmd verbose:",verbose);  //detailed 'Y' cmd execution    
    int dataZlength=0;     //line length
    timeout=120;           //set big timeout for first 'y'
    int tryYrow=0;         //fetch one row at a time
    int row_i=0;
    int inPtr = 0;
    int inPtrMsk = 0x3FF;          //limits size of ring.
    int Ymax=min(240,Yrow+WidithRequest); // test/limit parameters
    int Xmax=min(320,xStartPt+Zlength);   // assume x & z already sent to CAM
                            // by X & Z cmds.  Will get them back in headers 
    Zlength=Xmax-xStartPt;                // trim Z to <= 320
    img2 = createImage(320, 240, ARGB);   // create new image for CAM //#####
 
    while (myPort.available() > 0) {      // clear remaining incoming char's
      inByte = myPort.read();
      if(verbose) print(char(inByte));
    } 

    for (row_i=Yrow;row_i<Ymax;row_i++) {   //do for row Y## to row Y##+W## -1
/***/ //if(verbose) print("try to get row row_i="); println(row_i);
      for(tryYrow=4;tryYrow>0;tryYrow--){   //try 4 times to get good row
        timeout=20;       // check this is appropriate in wait 
        sendCmd('y',row_i);   //ask for image packet SENDS 'y' COMMAND TO CAM
        if(tryYrow<4){ print(tryYrow);
          timeout=timeoutMax;   //allow 120mSec initially for CAM to loop & pickup 'y'
        }
        inPtr=0;                  //initialise ring buffer pointers
        while(myPort.available() == 0){  //wait for first character from'y'
          if (waitForCh()<=0) break;     //delay 1mSec and count down timeout
        } 
        dataZlength=0;
        while(dataZlength==0){    //keep reading until find potential header
          //search Buff.length for ':'
          while (myPort.available() == 0) {  // any USB incoming characters
            if (waitForCh()<=0)        //delay 1mSec and count down timeout
              break;     //timed out
          } 
          if (timeout > 0)   {         //proceed with input      
            timeout=timeoutMax;        //reset timeout counter                            
            inPtr=int(inPtr+1)&inPtrMsk;   //advance inPtr to new cell
            inBuf[inPtr] = char(myPort.read());  //read char into ring buffer.
            if (chInBuf(inPtr)==':'){       //end character of header perhaps
/***/         if(verbose)println("found a :"); 
              if (isMyHeader(inPtr,row_i)){ //good header (xStartPt? Zlength?)
              int xStartPt = int(chInBuf(inPtr-5)) *2 ;  //uses header x val
                dataZlength = int(chInBuf(inPtr-3)) *2 ;   //uses header z val
                int ckSum = int(chInBuf(inPtr-1))*256 + int( chInBuf(j-2));  
/***/           if(verbose){ print("GOOD header Received. dataZlength is "); 
                  println(dataZlength);
                  for(int bf=inPtr-8;bf<(inPtr+18);bf++) 
                    print(chInBuf(bf));println("");
                }
                int rx = xStartPt+320*row_i;  //img2 pointer. Try copy to img2
                int numPixels=dataZlength;                              
                int actCkSum=0;                  //calculate ckSum for header
                for (j=(inPtr-8);j<(inPtr-2);j++) 
                  actCkSum=actCkSum+int(chInBuf(j));  //now done with header
                for (int k=rx;k<(rx+numPixels);k++){ 
                  while (myPort.available() < 2) {  // any USB incoming char.
                    if (waitForCh()<=0) break;     //timeout? try to recover?
                  }
                  if (timeout>=0)  {    //proceed to get 2 bytes (else abort) 
                    int byteHi=myPort.read();     //get 2 bytes & put in Img2
                    int byteLo=myPort.read();                                      
                    decodeRGB565((byteLo+(byteHi<<8)),k);  //decode into img2                                 
                  }else {
                    k=999999; //abort data read of this 'y' row
/***/               if (verbose) println("ran out of data ");
                  }
                }             //end of reading data and colouring pixels
                              //could consider retry if timeout??
/***/           if(verbose){print("Done row ");print(row_i);print(" for W");
                   print(WidithRequest);print(" Z");print(Zlength);print(" X");
                   print(xStartPt);print(" Y");println(Yrow);}         
                print('#');
                if (timeout>=0) tryYrow=-1;   //no need to try row repeat 'y' request 
              }     //end of good header code
            }      //end of if( == ':')
          }        
          if (timeout<=0)     // should try row again?
            dataZlength=-1;   // breakout of while loop   //reset flag 
/***/     if(verbose)print('E');   //end of header search         
        }          //end of while(header not found (dataZlength==0)
        if (timeout<=0) { 
/***/     if(verbose){print("timeout tryYrow again. row:");println(row_i);} 
        }      
/***/   if(verbose){print("end header search "); println(tryYrow);}
      }        //end of for(tryYrow>0) no header so try 'y' again
      if(tryYrow==00)println("failed repeat tries to get row-try next row");
               //header processed, move to next row
      inPtr=0;              //return header buffer to "empty"
    }          //end of for(;row_i<Ymax;)- finish & go back and do for next row
    if(verbose) print("end of W rows - "); 
    
      //draw whole image to display window.  Won't see it before end of draw()    
    scale(mirrorX,mirrorY);
    image(img2, 10*mirrorX, 30*mirrorY,320*SF*mirrorX,240*SF*mirrorY);    

    myPort.write("yy\n");     //send "yy" to resume normal CAM frame sampling
    return true;                       
}         

int getNum(int c){    //get a 3 digit number (0-999) from inLine 
  int num=-1;    // returns -1 if invalid number
      if(inLine[c]==',') c=c+1;
      if(chIsDigit(inLine[c])>=0) num = chIsDigit(inLine[c]); 
      else{ if(c<4)println("getNum(): invalid number ");
            return num; 
      }
      if(chIsDigit(inLine[c+1])>=0) num = num*10+chIsDigit(inLine[c+1]);
      else return num;
      if(chIsDigit(inLine[c+2])>=0) num = num*10+chIsDigit(inLine[c+2]);
      return num;
}

int chIsDigit(char ch){   //return value of ch or -1 if error
      if(ch>= '0' && ch <= '9') 
           return ch-'0';
      else return -1;
}

void sendCmd(char ch, int val){
       myPort.write(ch);
       String outString = Integer.toString(val);
       myPort.write(outString); 
       myPort.write(10);
}

void decodeRGB565(int rgb565,int cell){  // split RGB565 and colour pixel[j]
int    b=(rgb565 & int(0x1F))<<3;           //extract colour and make 8 bit.
int    g=(rgb565 >> 3)& int(0xFA);
int    r=(rgb565 >> 8)& int(0xF8);
       img2.pixels[cell] = color(r,g,b);   //writes one pixel (at i) with colour (0,153,204) brightness a(255-0)
       return;
}

boolean isMyHeader(int j,int row){
          if((chInBuf(j-8)=='y') && (chInBuf(j-6)=='x') && (chInBuf(j-4)=='z')) {    //found header
            if( int(chInBuf(j-7)) == row)  return true;    //correct  header 
            print("header row mismatch\n");
            return false;           
          }
          return false;
}

 char chInBuf(int Ptr){           //ring buffer
      return inBuf[Ptr&0x3FF]; 
}

int waitForCh(){
      delay(1);                       //wait for a character
      timeout--;
      if (timeout<1) if(verbose)print(timeout); //println(" timed out waiting on USB input ");
      return timeout;
}

void mousePressed(){
      if (newS){
        coordX=(mouseX-10)/SF;
        coordY=(mouseY-30)/SF;
      } 
}
         
