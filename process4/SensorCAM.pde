
/* 31/5/24  version 200
 *
 * sensorCAM Serial Monitor 
 * by Barry Daniel
 * 
 * Lets you do simple edits of a command line
 * Sends a line out the serial port when Enter key pressed
 * listens for bytes received, and displays their value.
 * sensorCAM command enhancements for imaging capability
 * 1/4/2023
 */

import processing.serial.*;

int comNo=1;  //0 for first COM port, may force another port  e.g. 1 for second port.

String VER = "SensorCAM RGB565 320 x 240   Monitor v200 for Processing4";
int BAUD=115200;

Serial myPort;      // The serial port
int whichKey = 10;  //'\n'  Variable to hold keystoke values
int inByte = -1;    // Incoming serial data
int chr=0;         
//String cmd="12345678901234567890";    //20 character command buffer
char inLine[] = new char[80];     //80 character command buffer

int i=0;
int j;
boolean echo=false;
int red;
PImage img;
PImage img2;
PImage mouseCursor;
int roi;

boolean newS = false;
int coordX=-1, coordY=-1;
int Wval=240; //240 max;      //my Processing command default values
int Xval=0;
int Yrow=0;
int Zlength=320;
int dataZlength=0;
int tryYrow=0;
int  mirrorX=1;
int  mirrorY=1;

int inPtr = 0;
int readPtr = 0;
char[] inBuf = new char[1024];         //Buffer for image data from USB
int inPtrMsk = 0x3FF;                  //limits size of ring.
int timeoutMax = 120; //mSec
int timeout=timeoutMax;        //start timeout counter

int savedWval=240;
int arow=1;

void setup() {

   print("choosing comNo ");println(comNo);
   
  size(660, 540);              //must be 1st line of setup()
//  mouseCursor = loadImage("8pixSQ.png");
  // create a font with the third font available to the system:
 PFont myFont = createFont(PFont.list()[147], 16);
  textFont(myFont,16);
  background(0);
  fill(204,102,0);
  rect(10,30,640,480);
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
//  img2 = createImage(320, 240, ARGB);   //create new image for CAM
//   colorMode(RGB, 255, 255, 255); 
   
//test pattern #2
/* 
 for(j=1;j<221;j++){
   red=(red+5)& 0xFF;
   for (int i=50;i<100;i++) {
     img2.pixels[i+320*j] = color(red,0,100);
   }
 }
    print(j);print(' ');

   image(img2, 90, 80, 320,240);//, height/2); //add half sized image
   img2 = createImage(320, 240, ARGB);   //create new image for CAM//  delay(5000);

*/

  // List all the available serial ports:
  printArray(Serial.list());

  // I open Serial.list()[0]. (the first available COM port?)
  // Open whatever port is the one you're using.

  String portName = Serial.list()[comNo];
  print("using "); println(portName);
  
  text("Cmd: ", 10, 530);
  text(VER, 10, 20);  
  myPort = new Serial(this, portName, BAUD,'N',8,1);
  //delay(3000);
  myPort.setDTR(false);      //GPIO0 high 3.3V
  myPort.setRTS(false);      //EN high?            (both low triggers ESP32 reset!

  print("end setup()\n");
  delay(3000);
}

void draw() {      // loops continuously
  while (myPort.available() > 0) {  // just display any USB incoming characters
    inByte = myPort.read();
    print(char(inByte));
  }
  
  if ((inLine[0] == 'k') || (inLine[0] == 'a')){    //set cursor for sensor creation
    if (i>=3 && mouseY <  500 && mouseY > 30 && mouseX >10) {
      cursor(HAND); // cursor(mouseCursor,0,0);
      newS = true;
      if (coordX >= 0) {          //create command for new sensor
        print ("make sensor");
        i=3;                     //always add after "a##"
        inLine[i]=','; i++;    
        String strng =str(coordY);    //row
        char[] posn = strng.toCharArray(); //str.toCharArray();
        arrayCopy(posn,0,inLine,i,posn.length);
        i=i+posn.length;
        inLine[i]=','; i++;
        strng = str(coordX);
        posn = strng.toCharArray();
        arrayCopy(posn,0,inLine,i,posn.length);
        i=i+posn.length;    
        coordX=-1;                  //don't let it repeat
      
        fill(200);                    //clear and rewrite command line
        rect(45,512,605,25); 
        fill(0);
        for(int j=0;j<i;j++)  text(inLine[j],50+j*10,530);
        text(char(1),50+i*10,530);    //try to add a claytons "cursor"      
      }
    }
    else  {cursor(CROSS); newS=false; }     
  }
  else  {cursor(CROSS);newS=false;}
 
 

  chr=whichKey;    
  if(chr != 0){  //print (char(whichKey));   //accept new key
    if(chr > 0x7F || chr < 0) chr=0;       //ignore 
    else{                                //key is ASCII
       if(echo) print(char(chr));
       inLine[i] = char(chr);

       if (inLine[i] == 10) {           // LF so send command including LF
         if (myCommand(inLine[0],i)) {  //identify and process special commands W X Y Z
           print("done myCmd: ");
           for(int j=0;j<i+1;j++) print(inLine[j]);   //prints myCmd AFTER command execution.
         } else {            //send command to CAM
           if(inLine[0]=='a') arow=getNum(4);         //save 'a' second parameter
           println(" ");      
           for (int j=0;j<i+1;j++) myPort.write(inLine[j]);
           if((inLine[0]== 'a')&&(i>7)) {        //new sensor reated
             delay(120);
             savedWval=Wval;
             Wval=10;
             Yrow=arow-2;
             print(arow);
             executeYcmd();            
             Wval=savedWval;
           }
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
       fill(200);                    //clear and rewrite command line
       rect(45,512,605,25);
       fill(0);
       for(int j=0;j<i;j++){  
         text(inLine[j],50+j*10,530);
       } 
       text(char(1),50+i*10,530);    //try to add a claytons "cursor"
    }
  }
}
/*
void serialEvent(Serial myPort) {        //interrupt driven (on receive?)
   // print ("incoming");
    inBuf[inPtr] = char(myPort.read());
    print(char(inByte));
    inPtr=(inPtr+1)& 0x3FF;
}
*/

void keyPressed() {
  // Send the keystroke out:
  int ch = key; 
  whichKey = ch;
  switch(ch) {      //for debugging only
    case 0x0E:      // ^N for no echo
        echo=false; whichKey=0; break;
    case 0x05:      // ^E for echo on
        echo= true; whichKey=0; break;
    case 0x12:      // ^R for SensorCAM Reset
        print("REBOOTING CAM\n");
        myPort.setRTS(true);   //'S' EN high stops FTDI
        myPort.setDTR(true);   //'D' GPIO0 low for download!S
        myPort.setDTR(false);  //'R' GPIO0 highTT
        myPort.setRTS(false);  //'T' EN low as well triggers reboot
        whichKey=0;            // hide 'R' key
  }
}

boolean myCommand(char cmd,int nch) { 
 int red=0;
 int char3=-1;
 char request[] = new char[80];     //80 character request buffer


 switch (cmd) {
   case 'H':            //mirror x
     mirrorX = -mirrorX;
     return true;
     
   case 'V':            //mirror y
     mirrorY = -mirrorY;
     return true;
     
   case 'W':            //get useful image strip width (height)
     Wval = getNum(1);   //no need to send anything to CAM as only for # of repeats of 'y'
     if(Wval < 0) Wval=240;
     ;  //try a default 
     return true;
  
   case 'Z':            //get useful image width
     Zlength = getNum(1);
     if(Zlength < 0) Zlength=320;  //try default row length
     sendCmd('z',Zlength);
     println(Zlength);
     return true;  
   
   case 'X':            //get top left corner of useful image
     Xval = getNum(1);
     if(Xval < 0) Xval=0;
     sendCmd('x',Xval);
     println(Xval);
     return true;
     
   case 'Y':            //get coordinate y, and fill image
     println("*****YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY***** start of Y");
     Yrow = getNum(1);                //from command line
     if(Yrow < 0) Yrow=0;
     println(Yrow);
     return executeYcmd();
         
   default:     //no Process commands recognised
     return false;
  
 }    //end switch
}     //end myCommand()   
     
boolean executeYcmd() {   
     //fetch an image strip at Yrow (Wval rows wide,starting at Xval column, ending after Zlength pixels)
     timeout=120;           //set big timeout for first 'y'
     int Ymax=min(240,Yrow+Wval);    // test/limit parameters
     int Xmax=min(320,Xval+Zlength); //   assume x & z already sent to CAM by X & Z cmds  Will get them back in headers 
     Zlength=Xmax-Xval;              // trim Z to <= 320
     img2 = createImage(320, 240, ARGB);   //create new image for CAM //#############################
 
     while (myPort.available() > 0) {     // display and clear any remaining incoming characters
        inByte = myPort.read();
        print(char(inByte));
     } 
/***/println("deb1:");
     for (roi=Yrow;roi<Ymax;roi++) {            //do for each row requested by W                    //for row Y## to rowY##+W## -1
/***/    print("try to get row roi="); println(roi); // delay (5000);
         for(tryYrow=4;tryYrow>0;tryYrow--){    //try 4 times to get good row
/*****/   //         print(tryYrow);
 /***/     //   print("deb1.5 "); println(roi);
              sendCmd('y',roi);                 //ask for image packet SENDS 'y' COMMAND TO CAM
 /***/     //   print("deb2 sent y");  println(roi);                                                          //d2
              timeout=120;                  // allow a 120mSec delay initially for CAM to loop and pickup 'y'cmd
              readPtr=0;                    //initialise ring buffer pointers
              inPtr=0;
              while(myPort.available() == 0){  //wait for first character from'y'
                 if (waitForCh()<=0) break;          //delay 1mSec and count down timeout
              }
/***/           //  print("deb3 received first char after 'y' time:"); println(120-timeout); 
              timeout=timeoutMax;   // 10mS check this is appropriate  
              dataZlength=0;
              while(dataZlength==0){   //keep reading characters til find potential header
          
          //                               for (j=readPtr;j<(readPtr+8+Zlength);j++) {   //search for valid header                   //search Buff.length for ':'
                while (myPort.available() == 0) {  // any USB incoming characters
                   if (waitForCh()<=0)          //delay 1mSec and count down timeout
                   break;     //timed out
                } 
                if (timeout > 0)   {        //proceed with input      
                  timeout=timeoutMax;        //reset timeout counter                            
                  inPtr=int(inPtr+1)&inPtrMsk;   //advance inPtr to new cell
                  inBuf[inPtr] = char(myPort.read());     //read char into ring buffer     assumes character available after above waiting for available
             //     print("deb4. read ch:");println(inBuf[inPtr]); 
                  if (chInBuf(inPtr)==':'){       //end character of header perhaps
 /***/              println("found a :"); //delay(20000);
                    if (isMyHeader(inPtr)) {   //good header (haven't checked Xval or Zlength)    //if is good header - process
                          
                               int Xstart = int(chInBuf(inPtr-5)) *2 ;          //uses header x val
                                   dataZlength = int(chInBuf(inPtr-3)) *2 ;     //uses header z val
                               int ckSum = int(chInBuf(inPtr-1))*256 + int( chInBuf(j-2));            //read expected chSum
 /***/                         print("GOOD header RReceived. dataZlength is "); println(dataZlength);for( int bf=inPtr-8;bf<(inPtr+20);bf++) print(chInBuf(bf));println("");
 /***/                      //   print("deb5 Xstart:"); println(Xstart); //delay(5000);                                               //d5
                               //try and copy data to img2
                               int rx = Xstart+320*roi;                  //img2 pointer
                     //          int BufPtr = inBuf;
 /***/                     //    print("deb6 inPtr:"); println(inPtr); // print(" RECEIVED Buf.len:"); println( Buf.length());               //d6
                               int numPixels=dataZlength;                              
 /***/                     //    print("deb8 numPixels:"); println( numPixels);                                                   //d8
                               int actCkSum=0;                                     //calculate ckSum for header
                               for (j=(inPtr-8);j<(inPtr-2);j++) actCkSum=actCkSum+int(chInBuf(j));      
                 //done with header
                               for (int k=rx;k<(rx+numPixels);k++){                            
                                 while (myPort.available() < 2) {  // any USB incoming characters
                                   if (waitForCh()<=0) break;      //timeout?  should try to recover.
                                 }
                                 if (timeout>=0)  {    //proceed to get 2 bytes (else abort)                              
                                     int byteHi=myPort.read();                //  get 2 bytes & put in Img2
                                     int byteLo=myPort.read();                                      
 /***/                               if (echo) { print(k-rx);print(" ");print(k); print(' ');print(byteHi);print(' '); print(byteLo);}
 /***/                               if (echo) println(" Ho Lo deb10 read a pixel");
                                     decodeRGB565((byteLo+(byteHi<<8)),k);                      //decodergb565 and put in img2 pixel            //step pointer to next pixel
                                    
                                 }else {
                                    k=999999;    //abort data read of this 'y' row
                                    println("ran out of data ");
                                 }
                               }              //end of reading data and colouring pixels
                                              //could consider retry if timeout??
/***/                          print("Done row for W");print(Wval);print(" Z");print(Zlength);print(" X");print(Xval);print(" Y");println(Yrow); //delay(5000);                                              
                //               image(img2, 10, 30,640,480);           //draw image to display window          //write 1 row image
                //               img2 = createImage(320, 240, ARGB);            //create new image for CAM              //stat up a new img2
/***/                          print(roi); println("=roi");// processed header&data & done row image()");//delay(5000);
                               tryYrow=-1;  //no need to try row repeat 'y' request 
                    }     //end of if(is myheader)
                  }      //end of if( == ':')
/***/       //      println("move on from found : (if==:"); // delay(5000);
                }        //end of if( timeout>0 no timeout in header)
                if (timeout<=0){     // should try row again?
                    dataZlength=-1;  // breakout of while loop   //resetflag just before while(dataZlength)
                }
/***/           print('E'); //end of header search dataZlength:); println(dataZlength); // delay(5000);
     //           
              }          //end of while(header not found (dataZlength==0)
             if (timeout<=0) { print("timeout tryYrow again. remaining tries:");println(tryYrow);  }      
/***/        print("end header search "); println(tryYrow); //delay(5000);
         }  //end of for(tryYrow>0) no header so try 'y' again
         if(tryYrow==00){println("failed repeat tries to get row - try next row");}  //header successfully processed move to next row
         inPtr=0;            //return header buffer to "empty"
         readPtr=0;
     }  //now go back and do for next row
     println("end of W rows"); //delay(10000);
         scale(mirrorX,mirrorY);
         image(img2, 10*mirrorX, 30*mirrorY,640*mirrorX,480*mirrorY);           //draw whole image to display window     //  Can't see it anyway before en of draw() loop
   //      img2 = createImage(320,240.ARGB);      //set up for another Y# cmd
     //end of for (roi<<Ymax)
     myPort.write("yy\n");            //send "yy" to resume normal CAM frame sampling
     return true;                       
}         



int getNum(int c){    //get a 3 digit number (0-999) from inLine 
                 // returns -1 if invalid number
  int num=-1;
      if(chIsDigit(inLine[c])>=0) num = chIsDigit(inLine[c]); 
      else{ if(c<4)println("getNum(): invalid number ");
            return num; }
      if(chIsDigit(inLine[c+1])>=0) num = num*10+chIsDigit(inLine[c+1]);
      else return num;
      if(chIsDigit(inLine[c+2])>=0) num = num*10+chIsDigit(inLine[c+2]);
      return num;
}

int chIsDigit(char ch){   //return value of ch or -1 if error
   int tmp=int(ch)-0x30;
   if ((tmp>-1) && (tmp<10)) 
     return tmp;
   else 
     return -1;
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
/***/ // if(roi==125){ print('r');print(r>>2); print('g');print(g>>2); print('b');print(b>>2);}
       img2.pixels[cell] = color(r,g,b);   //writes one pixel (at i) with colour (0,153,204) brightness a(255-0)
//     if (cell>20000)img2.pixels[cell] = color(255,0,0);   //writes one pixel (at i) with colour (0,153,204) brightness a(255-0)  

     return;
}

boolean isMyHeader(int j){
          if((chInBuf(j-8)=='y') && (chInBuf(j-6)=='x') && (chInBuf(j-4)=='z')) {    //found header
            if( int(chInBuf(j-7)) != roi) {     //wrong  header 
              print("header row mismatch\n");
              return false;
            }
            return true;
          }
          return false;
}
char  chInBuf(int Ptr){
          return inBuf[Ptr&0x3FF]; 
}
// while (myPort.available() == 0) {  // any USB incoming characters
int waitForCh(){
      delay(1);                       //wait for a character
      timeout--;
      if (timeout<1) print(timeout); //println(" timed out waiting on USB input ");
      return timeout;
}
void mousePressed(){
  if (newS){
    coordX=(mouseX-10)/2;
    coordY=(mouseY-30)/2; 
//    print("set one at row=");print(coordY);
//    print(" x=");  println(coordX);
  }
  
}
         
