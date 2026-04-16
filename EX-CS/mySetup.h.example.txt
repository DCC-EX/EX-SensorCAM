// setup for sensorCAM on an ESP32-CAM NUMDigitalPins <= 80
// assumes 700 is first vpin (e.g. create in myAutomation.h: HAL(EXSensorCAM,700,80,0x11)
// the optional SETUP operations below initiate jmri monitoring of sensors for any change of state
// Mostly useful during debug of initial system but load CS with MPU & RAM use. Use judiciously
I2CManager.setClock(100000);   //to set i2c bus clock rate (redundant after CS v5.5.24)
//         id vPin	      	    
SETUP("<Z 100 700 0>");     // set up for control OUTPUT on vpin #00
// start of up to 80 sensors numbered bsNo's 100 to 197 (CAM1: 0/0 to 9/7)
SETUP("<S 100 700 0>");     // first sensor (S00) (reserved for reference sensor)  
SETUP("<S 101 701 0>");    
SETUP("<S 102 702 0>"); 
//        as many as you want.  You can add later manually with CS native commands       	 
// avoid 106 (S06) as reserved for CAM management
SETUP("<S 107 707 0>");     
SETUP("<S 110 708 0>");     // Note: suggested id is b/s format (~OCT); vpin is DEC.
SETUP("<S 111 709 0>");     // myFilter.cpp REQUIRES this relationship for bsNo to vPin conversion
SETUP("<S 112 710 0>");
SETUP("<S 113 711 0>");
SETUP("<S 114 712 0>");
//etc.                          
SETUP("<S 120 716 0>");
SETUP("<S 121 717 0>");
SETUP("<S 122 718 0>");
//SETUP("<S 123 719 0>");
//etc.      
// can also create a bulk set of sensors (e.g. 3/0 to 6/7 but uses up RAM) with c++ code so: 
//for(uint16_t b=3; b<=6;b++) for(uint16_t s=0;s<8;s++) Sensor::create(100+b*10+s,700+b*8+s,1);      
//
SETUP("<S 181 765 0>");
SETUP("<S 191 773 0>");
