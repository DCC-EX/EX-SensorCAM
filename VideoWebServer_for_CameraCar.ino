// VideoWebServer_for_CameraCar_v006.ino
//  
// Video Cam Car HTTP Server version
// 2023/05/07 Kazuhiko Tomomatsu
// Base Software: Sketch_05.2_As_VideoWebServer by Freenove
// VideoWebServer_for_CameraCar_v001.ino

// 2023/06/05 Kazuhiko Tomomatsu
// Added ESP chip information detection and display
// VideoWebServer_for_CameraCar_v002.ino
//
// 2023/06/06 Kazuhiko Tomomatsu
// Added SNMP Agent
// VideoWebServer_for_CameraCar_v003.ino
//
// 2023/06/07 Kazuhiko Tomomatsu
// SNMP MIB 1.3.6.1.4.1.4998.2.1.1 returns WiFi_Sig_Strength
// VideoWebServer_for_CameraCar_v004.ino
//
// 2023/06/07 Kazuhiko Tomomatsu
// SNMP MIB 1.3.6.1.4.1.4998.2.1.2 returns frame_p_1000s
// SNMP MIB 1.3.6.1.4.1.4998.2.1.3 returns HeadLightStatus
// VideoWebServer_for_CameraCar_v005.ino
//
// 2023/06/10 Kazuhiko Tomomatsu
// XAIO_ESP32S3 Version using CAMERA_MODEL_XIAO_ESP32S3
// VideoWebServer_for_CameraCar2_v005.ino
// 
// 2023/06/12 Kazuhiko Tomomatsu
// MIB OID Changed to 1.3.6.1.4.1.4998.3.1.x
// Add 1.3.6.1.4.2.4998.3.1.1 for ssidId as read/write
// Add 1.3.6.1.4.1.4998.3.1.2 for ssid
// Add ssid change feature. 1.3.6.1.4.1.4498.3.1.2 changes ssid ID.
// Changed Video Quality Default from 12 to 38.
// Add VIdeo Quality reporting in the atetus report via serial every 10 sec.
// VideoWebServer_for_CameraCar2_v006.ino

#include "esp_camera.h"
#include <WiFi.h>

// 2023/06/06 Added SNMP Agent
#include <WiFiUdp.h>
#include <SNMP_Agent.h>
#include <SNMPTrap.h>
// End of 2023/06/06 Added SNMP Agent

// Select camera model
#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM

#include "camera_pins.h"

int ssidId = 0;
int prev_ssidId = 0;
char* ssid[]     = {"Medrock", "NWRR_WiFi"};   //input your wifi name
char* password[] = {"arrisise", "NWR@6d22"};   //input your wifi passwords
//char* ssid     = "NWRR_WiFi";   //input your wifi name
//char* password = "NWR@6d22";   //input your wifi passwords

// 2023/06/06 Added SNMP Agent
WiFiUDP udp;
// Starts an SMMPAgent instance with the read-only community string 'public', and read-write community string 'private
SNMPAgent snmp = SNMPAgent("public", "private");  

// Numbers used to response to Get requests
uint32_t tensOfMillisCounter = 0;
uint32_t prev_tensOfMillisCounter = 0;
uint32_t diff_tensOfMillisCounter = 0;
std::string sysDescr = "Video Cam Car 2 - VideoWebServer_for_CameraCar2_v006.ino";
char* sysContact;
char* sysName;
char* sysLocation;

// arbitrary data will be stored here to act as an OPAQUE data-type
uint8_t* stuff = 0;


// If we want to change the functionaality of an OID callback later, store them here.
ValueCallback* ssidIdOID;
ValueCallback* ssidOID;
ValueCallback* WiFi_Sig_StrengthOID;
ValueCallback* frame_p_1000sOID;
ValueCallback* HeadLightStatusOID;
ValueCallback* sysUpTimeOID;
ValueCallback* sysDescrOID;
ValueCallback* sysContactOID;
//TimestampCallback* timestampCallbackOID;
TimestampCallback* sysUptimeOID;


// Setup an SNMPTrap for later use
SNMPTrap* settableNumberTrap = new SNMPTrap("public", SNMP_VERSION_2C);
char* changingString;

// End of 2023/06/06 Added SNMP Agent

void startCameraServer();

char* ssid_name;
int WiFi_Sig_Strength;
int frame_p_1000s; // number of frame per 1000 sec
extern int HeadLight;
extern int VideoQuality;
int HeadLightPin = D1;

// Configures static IP address
/*
// Set your Static IP address
IPAddress local_IP(192, 168, 1, 201);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(192, 168, 1, 1); //optional
if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
   Serial.println("STA Failed to configure");
}
*/

void setup() {
  pinMode(HeadLightPin, OUTPUT); 

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("VideoWebServer_for_CameraCar_v006.ino");
  Serial.println("2023/06/12 Kazuhiko Tomomatsu");
// 2023/06/05 Added ESP chip information detection and display
  Serial.printf("This chip has %d cores\n", ESP.getChipCores());
  Serial.printf("Total heap: %d\n", ESP.getHeapSize());
  Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
  Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
  Serial.printf("Free PSRAM: %d\n", ESP.getFreePsram());
  Serial.printf("ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
	Serial.printf("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());
  Serial.println();
// End of 2023/06/05 Added ESP chip information detection and display


  WiFi.begin(ssid[ssidId], password[ssidId]);
  Serial.println("");
  Serial.print("WiFi connecting to ");
  Serial.println(ssid[ssidId]);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(WiFi.status());
  }
  Serial.println("");
  Serial.print("WiFi connected to ");
  Serial.println(ssid[ssidId]);

// 2023/05/07 Added WiFi signal strength reporting
  Serial.print("Signal Strength (RSSI): ");
  WiFi_Sig_Strength = WiFi.RSSI();
  Serial.println(WiFi_Sig_Strength);  /*Print WiFi signal strength*/
// End of 2023/05/07 Addition

// 2023/06/06 Added SNMP Agent
// give snmp a pointer to the UDP object
    snmp.setUDP(&udp);
    snmp.begin();

    // setup our OPAQUE data-type
    stuff = (uint8_t*)malloc(4);
    stuff[0] = 1;
    stuff[1] = 2;
    stuff[2] = 24;
    stuff[3] = 67;

    ssid_name = ssid[ssidId];
    ssidIdOID = snmp.addIntegerHandler(".1.3.6.1.4.1.4998.3.1.1", &ssidId, true);
    ssidOID = snmp.addReadOnlyStaticStringHandler(".1.3.6.1.4.1.4998.3.1.2", ssid_name); 
    WiFi_Sig_StrengthOID = snmp.addIntegerHandler(".1.3.6.1.4.1.4998.3.1.3", &WiFi_Sig_Strength);
    frame_p_1000sOID = snmp.addIntegerHandler(".1.3.6.1.4.1.4998.3.1.4", &frame_p_1000s);
    HeadLightStatusOID = snmp.addIntegerHandler(".1.3.6.1.4.1.4998.3.1.5", &HeadLight, true);

    // SNMPv2-MIB
    sysDescrOID = snmp.addReadOnlyStaticStringHandler(".1.3.6.1.2.1.1.1.0", sysDescr);    
    sysUpTimeOID = (TimestampCallback*)snmp.addTimestampHandler(".1.3.6.1.2.1.1.3.0", &tensOfMillisCounter);
    sysContact = (char*)malloc(25 * sizeof(char));
    snprintf(sysContact, 25, "ktomoma@mac.com");
    sysContactOID = snmp.addReadWriteStringHandler(".1.3.6.1.2.1.1.4.0", &sysContact, 25, true);
    sysName = (char*)malloc(25 * sizeof(char));
    snprintf(sysName, 25, "XIAO ESP32S3 Sense");
    snmp.addReadWriteStringHandler(".1.3.6.1.2.1.1.5.0", &sysName, 25, true);
    sysLocation = (char*)malloc(25 * sizeof(char));
    snprintf(sysLocation, 25, "Warrenville IL, USA");
    snmp.addReadWriteStringHandler(".1.3.6.1.2.1.1.6.0", &sysLocation, 25, true);

    // Setup SNMP TRAP
    // The SNMP Trap spec requires an uptime counter to be sent along with the trap.
    settableNumberTrap->setUDP(&udp); // give a pointer to our UDP object
    settableNumberTrap->setTrapOID(new OIDType(".1.3.6.1.2.1.33.2")); // OID of the trap
    settableNumberTrap->setSpecificTrap(1); 

    // Set the uptime counter to use in the trap (required)
    settableNumberTrap->setUptimeCallback(sysUptimeOID);

    // Set some previously set OID Callbacks to send these values with the trap (optional)
//    settableNumberTrap->addOIDPointer(changingNumberOID);
//    settableNumberTrap->addOIDPointer(settableNumberOID);

    settableNumberTrap->setIP(WiFi.localIP()); // Set our Source IP

    // Ensure to sortHandlers after adding/removing and OID callbacks - this makes snmpwalk work
    snmp.sortHandlers();
// End of 2023/06/06 Added SNMP Agent


  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.fb_location = CAMERA_FB_IN_PSRAM; // Set the frame buffer storage location
//    config.frame_size = FRAMESIZE_UXGA;
//    config.frame_size = FRAMESIZE_SXGA;
//    config.frame_size = FRAMESIZE_XGA;    
    config.frame_size = FRAMESIZE_SVGA;
//    config.frame_size = FRAMESIZE_VGA;
//    config.frame_size = FRAMESIZE_HGA;
//    config.frame_size = FRAMESIZE_CIF;
//   config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 38;
    config.fb_count = 2;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  sensor_t * s = esp_camera_sensor_get();
  // drop down frame size for higher initial frame rate
//  s->set_framesize(s, FRAMESIZE_UVGA);
//  s->set_framesize(s, FRAMESIZE_SXGA);
//  s->set_framesize(s, FRAMESIZE_XGA);
//  s->set_framesize(s, FRAMESIZE_SVGA);
//  s->set_framesize(s, FRAMESIZE_VGA); 
//  s->set_framesize(s, FRAMESIZE_HVGA);
//  s->set_framesize(s, FRAMESIZE_CIF);
//  s->set_framesize(s, FRAMESIZE_QVGA);

// Flip the image
// Uncomment this if display image needs to be lotated 180 deg.
//  s->set_vflip(s, 1);
//  s->set_hmirror(s, 1);
  
  startCameraServer();
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
// 2023/06/06 Added SNMP Agent
  snmp.loop(); // must be called as often as possible
 /**
  if(settableNumberOID->setOccurred){    
    Serial.printf("Number has been set to value: %i\n", settableNumber);
    if(settableNumber%2 == 0){
      // Sending an SNMPv2 INFORM (trap will be kept and re-sent until it is acknowledged by the IP address it was sent to)
      settableNumberTrap->setVersion(SNMP_VERSION_2C);
      settableNumberTrap->setInform(true); // set this to false and send using `settableNumberTrap->sendTo` to send it without the INFORM request
    } else {
      // Sending regular SNMPv1 trap
      settableNumberTrap->setVersion(SNMP_VERSION_1);
      settableNumberTrap->setInform(false);
    }
    settableNumberOID->resetSetOccurred();
    // Send the trap to the specified IP address
    // If INFORM is set, snmp.loop(); needs to be called in order for the acknowledge mechanism to work.
    IPAddress destinationIP = IPAddress(192, 168, 1, 243);
    if(snmp.sendTrapTo(settableNumberTrap, destinationIP, true, 2, 5000) != INVALID_SNMP_REQUEST_ID){ 
      Serial.println("Sent SNMP Trap");
    } else {
      Serial.println("Couldn't send SNMP Trap");
    }
  }
 **/
  tensOfMillisCounter = millis()/10;
// End of 2023/06/06 Added SNMP Agent
  diff_tensOfMillisCounter = tensOfMillisCounter - prev_tensOfMillisCounter;
  if (diff_tensOfMillisCounter > 1000){ //Print RSSI every 10 sec
    prev_tensOfMillisCounter = tensOfMillisCounter;
    WiFi_Sig_Strength = WiFi.RSSI();
    Serial.printf("Signal Strength (RSSI): %d", WiFi_Sig_Strength); // Print WiFi signal strength
    Serial.printf(" Free PSRAM: %d", ESP.getFreePsram());
    Serial.printf(" SSID ID: %d", ssidId);
    Serial.printf(" Quality: %d", VideoQuality);
    Serial.printf(" fps: %.3f", float(frame_p_1000s)/1000);
    Serial.printf(" Headlight: %d\n", HeadLight);
  }
  digitalWrite(HeadLightPin, HeadLight);

  // 2023/06/12 Add ssid change routine
  // Serial.printf("ssidId = %d, prev_ssidId = %d\n", ssidId, prev_ssidId);
  if(ssidId!=prev_ssidId) {
    if(ssidId<2){
      Serial.print("Switching Wifi to ");
      Serial.println(ssid[ssidId]);
      delay(5000);
      WiFi.disconnect();
      delay(5000);
      WiFi.begin(ssid[ssidId], password[ssidId]);
      prev_ssidId = ssidId;
      int i = 0;
      while ((WiFi.status() != WL_CONNECTED) & (i<100)) {
        delay(500);
        Serial.print(WiFi.status());
        i++;
      }
      if(i<100){
        Serial.println("");
        Serial.print("WiFi connected to ");
        Serial.println(ssid[ssidId]);
      }
      else{
        Serial.println("");
        Serial.print("WiFi connecttion to ");
        Serial.print(ssid[ssidId]);
        Serial.println(" failed");
        ssidId = 1 - ssidId;
      }
    }
    else{
      ssidId=prev_ssidId;
    }
  }
  // End of 2023/06/12 Add ssid change routine

  delay(100); // This 100 ms delay is requried for snmp.loop() not to take more CPU.
}
