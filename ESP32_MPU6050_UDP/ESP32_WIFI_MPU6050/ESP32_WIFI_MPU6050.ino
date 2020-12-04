/* ============================================
    USED ESP32: TTGO DISPLAY
  IMU6050

  3.3V         VCC              Power
  GND          GND              Ground
  21           SDA              I2C Data
  22           SCL              I2C Clock

  ===============================================
*/

#include <ESP8266WebServer.h>

#if defined(ESP8266)
#include <ESP8266WiFi.h>  //ESP8266 Core WiFi Library         
#else
#include <WiFi.h>      //ESP32 Core WiFi Library    
#endif

#include <WiFiUDP.h>
#include <OSCMessage.h> /// https://github.com/CNMAT/OSC
#include <OSCBundle.h> /// https://github.com/CNMAT/OSC
#include <Wire.h>
#include <ESPmDNS.h>
#include <OSCData.h>
#include <WebServer.h> //Local DNS Server used for redirecting all requests to the configuration portal (  https://github.com/zhouhan0126/DNSServer---esp32  )
#include <DNSServer.h> //Local WebServer used to serve the configuration portal (  https://github.com/zhouhan0126/DNSServer---esp32  )

#include "Wire.h"
#include "I2Cdev.h"
#include <BfButton.h> //https://github.com/mickey9801/ButtonFever

#include "SPI.h"
#include "TFT_eSPI.h" //https://github.com/Bodmer/TFT_eSPI //LOOK FOR TTGO DISPLAY IN 
TFT_eSPI tft = TFT_eSPI();
#include "fonts.h"

#define SDA 21
#define SCL 22

#define GFXFF 1
#define FF18 &FreeSans12pt7b
#define CF_OL24 &Orbitron_Light_24
#define CF_OL32 &Orbitron_Light_32
#define CF_RT24 &Roboto_Thin_24
#define CF_S24  &Satisfy_24
#define CF_Y32  &Yellowtail_32



// ================================================================
// ===                        MAIN INIT                         ===
// ================================================================


char *espname = "HandGyro";
WiFiUDP Udp;
IPAddress destIp;

#define destPort 8000  //EDIT osc input port
#define localPort 9000 //EDIT osc output port (only used at startup for announcement)

unsigned int ledState = LOW;              // LOW means led is *on*

OSCErrorCode error;

#define LED_PIN 2 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
int proxBtn = 0;

// ================================================================
// ===                        BUTTON INIT                         ===
// ================================================================

#define NUM_OF_BUTTONS 4
#define BUTTON_1_GPIO 37
#define BUTTON_2_GPIO 13
#define BUTTON_3_GPIO 15
#define BUTTON_4_GPIO 2

#define NUM_OF_SETUP_BUTTONS 2

#define BUTTON_RST_GPIO 0
#define BUTTON_SHTD_GPIO 35

const int BUTTON_GPI0_LIST[] = {BUTTON_1_GPIO, BUTTON_2_GPIO, BUTTON_3_GPIO, BUTTON_4_GPIO};
const int SETUP_BTN_LIST[] = {BUTTON_RST_GPIO, BUTTON_SHTD_GPIO};

BfButton btn_1(BfButton::STANDALONE_DIGITAL, BUTTON_1_GPIO, false, LOW);
BfButton btn_2(BfButton::STANDALONE_DIGITAL, BUTTON_2_GPIO, false, LOW);
BfButton btn_3(BfButton::STANDALONE_DIGITAL, BUTTON_3_GPIO, false,  LOW);
BfButton btn_4(BfButton::STANDALONE_DIGITAL, BUTTON_4_GPIO, false, LOW);

BfButton btn_reset(BfButton::STANDALONE_DIGITAL, BUTTON_RST_GPIO, true, LOW);
BfButton btn_shutdown(BfButton::STANDALONE_DIGITAL, BUTTON_SHTD_GPIO, true, LOW);

BfButton BTN_LIST[] = {btn_1, btn_2, btn_3, btn_4};
BfButton SETUP_LIST[] = {btn_reset, btn_shutdown};


void directionHandler (BfButton *btn, BfButton::press_pattern_t pattern) {
  // If single press detected
  if (pattern == BfButton::SINGLE_PRESS) {
    int pressed_btn_gpio = btn->getID();
    Serial.print("MAIN BTN: ");
    Serial.print(btn->getID());
    // Determine which button was pressed
    for (int i = 0; i < NUM_OF_BUTTONS; i++) {
      if (pressed_btn_gpio == BUTTON_GPI0_LIST[i]) {
        Serial.println(" PRESSED ONE OF THE FOUR HORSEMEN");
      }
    }

    if (pressed_btn_gpio == BUTTON_GPI0_LIST[0]) {
      Serial.println(" BUTTON A ");
      tft.fillScreen(TFT_BLACK);
      tft.setCursor(10, 65); tft.setTextColor(TFT_WHITE);  tft.setTextSize(1); tft.println("KEY_LEFT_ARROW");
      Serial.println(" LEFT");
      delay(300);

    }

    if (pressed_btn_gpio == BUTTON_GPI0_LIST[1]) {
      //        selected_tone_preset = i + 1;
      Serial.println(" BUTTON B  ");
      tft.fillScreen(TFT_BLACK);
      tft.setCursor(10, 65); tft.setTextColor(TFT_WHITE);  tft.setTextSize(1); tft.println("KEY_UP_ARROW");
      Serial.println(" UP");
      delay(300);

    }

    if (pressed_btn_gpio == BUTTON_GPI0_LIST[2]) {
      Serial.println(" BUTTON C ");
      tft.fillScreen(TFT_BLACK);
      tft.setCursor(10, 65); tft.setTextColor(TFT_WHITE);  tft.setTextSize(1); tft.println("KEY_DOWN_ARROW");
//      proxBtn++;
      Serial.println(" DOWN");
      delay(300);

    }

    if (pressed_btn_gpio == BUTTON_GPI0_LIST[3]) {
      Serial.println(" BUTTON D ");
      tft.fillScreen(TFT_BLACK);
      tft.setCursor(10, 65); tft.setTextColor(TFT_WHITE);  tft.setTextSize(1); tft.println("KEY_RIGHT_ARROW");
      Serial.println(" RIGHT");
      delay(300);
    }
  }


  if (pattern == BfButton::LONG_PRESS) {
    int pressed_btn_gpio = btn->getID();
    Serial.print("MAIN BTN: ");
    Serial.print(btn->getID());
    // Determine which button was pressed
    for (int i = 0; i < NUM_OF_BUTTONS; i++) {
      if (pressed_btn_gpio == BUTTON_GPI0_LIST[i]) {
        //        selected_tone_preset = i + 1;
        Serial.println(" LONG PRESSED ONE OF THE FOUR HORSEMEN");
        //        sendLoadTonePresetCmd(LOAD_TONE_PRESET_LIST[i]);
      }
    }
  }
}

//
void resetHandler (BfButton *btn_reset, BfButton::press_pattern_t pattern) {
  // If single press detected
  if (pattern == BfButton::SINGLE_PRESS) {
    int setup_btn_gpio = btn_reset->getID();
    Serial.print("SETUP BTN: ");
    Serial.print(btn_reset->getID());
    // Determine which button was pressed
    for (int i = 0; i < NUM_OF_SETUP_BUTTONS; i++) {
      if (setup_btn_gpio == SETUP_BTN_LIST[i]) {
        Serial.println(" PRESSED THE TWO TOP BUTTONS");
      }
    }


    if (setup_btn_gpio == SETUP_BTN_LIST[0]) {
      proxBtn++;

      tft.fillScreen(TFT_BLACK);
      tft.setCursor(10, 65); tft.setTextColor(TFT_WHITE);  tft.setTextSize(1); tft.println("MODES UP");

      delay(300);
      Serial.println(" BUTTON TOP A ");  //        selected_tone_preset = i + 1;
    }
    if (setup_btn_gpio == SETUP_BTN_LIST[1]) {
      proxBtn = 0;
      tft.fillScreen(TFT_BLACK);
      tft.setCursor(10, 65); tft.setTextColor(TFT_WHITE);  tft.setTextSize(1); tft.println("RESET MODES");

      //      Keyboard.releaseAll();
      delay(300);
      //  delay(10);
      Serial.println(" BUTTON TOP B ");  //        selected_tone_preset = i + 1;
    }

  }

  if (pattern == BfButton::LONG_PRESS) {
    int setup_btn_gpio = btn_reset->getID();
    Serial.print("SETUP BTN: ");
    Serial.print(btn_reset->getID());
    // Determine which button was pressed
    for (int i = 0; i < NUM_OF_SETUP_BUTTONS; i++) {
      if (setup_btn_gpio == SETUP_BTN_LIST[i]) {
        Serial.println(" LONG PRESSED THE TWO TOP BUTTONS");
      }
    }
  }
}


void inputSetup() {
  // Setup callback for single press detection on all four input buttons
  for (int i = 0; i < NUM_OF_BUTTONS; i++) {
    BTN_LIST[i].onPress(directionHandler);
  }  // Setup callback for single press detection on all four input buttons
  for (int i = 0; i < NUM_OF_SETUP_BUTTONS; i++) {
    SETUP_LIST[i].onPress(resetHandler);
  }
}


void btnTest() {
  //  TEST BUTTON LOOOP
  //
  for (int i = 0; i < NUM_OF_BUTTONS; i++) {
    //    Serial.println(NUM_OF_BUTTONS);
    BTN_LIST[i].read();
  }
  for (int i = 0; i < NUM_OF_SETUP_BUTTONS; i++) {
    //    Serial.println(NUM_OF_SETUP_BUTTONS);
    SETUP_LIST[i].read();
  }


}

// ================================================================
// ===                       MPU6050 INIT                       ===
// ================================================================
int averageCount = 0;
double averageX = 0;
double averageY = 0;
double averageZ = 0;

//#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050_9Axis_MotionApps41.h"

MPU6050 mpu;

void ICACHE_RAM_ATTR dmpDataReady(); //intrrrupt necessary for esp8266/32

//#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_YAWPITCHROLL_OSC //UNCOMMENT THIS FOR YAW, PITCH, ROLL
#define OUTPUT_TEAPOT_OSC // UNCOMMENT THIS FOR NORMAL TEAPOT OSC

//DEFAULT IMU SETUP

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw, pitch, roll;

//int left_button_pin = 27; // Left button
//int right_button_pin = 33; // right button
int leftClickFlag = 0;
const int sensitivity = 45;
const int sensitivityB = 45;
float vertZero, horzZero;
float vertValue, horzValue;  // Stores current analog output of each axis
bool invertAll = false;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void tftSetup() {
  tft.init();
  tft.setRotation(1);

  tft.setFreeFont(CF_OL32);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(10, 80); tft.setTextColor(TFT_WHITE);  tft.setTextSize(1); tft.println("HELLO");

  delay(1000);

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(35, 100); tft.setTextColor(TFT_WHITE);  tft.setTextSize(3); tft.println("3");
  delay(250);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(75, 100); tft.setTextColor(TFT_WHITE);  tft.setTextSize(3); tft.println("2");
  delay(250);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(150, 100); tft.setTextColor(TFT_WHITE);  tft.setTextSize(3); tft.println("1");
  delay(250);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(10, 115); tft.setTextColor(TFT_WHITE);  tft.setTextSize(4); tft.println("GO");
}

//IF YOU WANT A STATIC IP
int channel = 11;

IPAddress local_IP(192, 168, 0, 199);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 0, 0);
//IPAddress primaryDNS(8, 8, 8, 8); //optional
//IPAddress secondaryDNS(8, 8, 4, 4); //optional


//SMART CONFIGR
/////////////////////////////////////////////////////////////////

void wifiSetting(bool staticIp) {
  //  smartConfig(true); //set to true to stop static IP

  //UNCOMMENT FOR STATIC IP 198.168.0.199
  if (!staticIp) {
    if (!WiFi.config(local_IP, gateway, subnet)) {
      Serial.println("STA Failed to configure");
    }
  }
  // 記憶しているAPへ接続試行
  WiFi.mode(WIFI_STA);
  WiFi.begin();

  // 接続できるまで10秒待機
  for (int i = 0; WiFi.status() != WL_CONNECTED && i < 100; i++) {
    delay(100);
  }

  // 接続できない場合はSmartConfigを起動
  // https://itunes.apple.com/app/id1071176700
  // https://play.google.com/store/apps/details?id=com.cmmakerclub.iot.esptouch

  //
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("SmartConfig start.");
    WiFi.beginSmartConfig();
    while (WiFi.status() != WL_CONNECTED) {
      delay(100);
    }
    WiFi.stopSmartConfig();
    //    WiFi.mode(WIFI_MODE_APSTA);
  }

  //END SMART CONFIG
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("ESP Mac Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Subnet Mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway IP: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("DNS: ");
  Serial.println(WiFi.dnsIP());
  Serial.println(String("IP:") + WiFi.localIP().toString());;
  delay(1000);

  //STARTING OSC
  MDNS.begin(espname);
  MDNS.addService("_osc", "_udp", localPort);
  destIp = WiFi.localIP();
  Udp.begin(destPort );
  OSCMessage msg("/ready"); //announcement
  msg.add(espname);
  msg.add(int(destIp[0]));
  msg.add(int(destIp[1]));
  msg.add(int(destIp[2]));
  msg.add(int(destIp[3]));
  msg.add(destPort );
  destIp[3] = 255;  //use broadcast ip x.x.x.255

  Serial.print("Accel IP: ");
  Serial.println(destIp);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
#ifdef ESP32
  Serial.println(localPort);
#else
  Serial.println(Udp.localPort());
#endif


}


void setup()
{
  //    Wire.begin();                              // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin(SDA, SCL);
  //  Wire.setClock(400000);
  // join I2C bus (I2Cdev library doesn't do this automatically)

  Serial.begin(115200);                       // initialize serial communication
  while (!Serial);                            // wait for Leonardo enumeration, others continue immediately

  //WIFI SETUP
  wifiSetting(true); //SET TO FALSE FOR STATIC IP
  //BTN SETUP
  inputSetup();
  //DISPLAY SETUP
  tftSetup();
  pinMode(LED_PIN, OUTPUT);                 // configure LED for output

  // ================================================================
  // ===                      MPU6050 SETUP                       ===
  // ================================================================

  Serial.println();
  Serial.println(F("------------------------------------"));
  Serial.println(F("Imu6050 - starting"));
  Serial.println(F("------------------------------------"));
  mpu.initialize();
  devStatus = mpu.dmpInitialize();


  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-34);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0)
  {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    Serial.println(F("Enabling DMP..."));

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(0);
    Serial.println(F(")..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;                        // set our DMP Ready flag so the main loop() function knows it's okay to use it
    packetSize = mpu.dmpGetFIFOPacketSize();      // get expected DMP packet size for later comparison

  }
  else
  { // ERROR!        1 = initial memory load failed         2 = DMP configuration updates failed        (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  yaw = 0.0;
  pitch = 0.0;
  roll = 0.0;
  vertZero = 0;
  horzZero = 0;

  // ================================================================
  // ===                   MOUSE KEYBOARD SETUP                   ===
  // ================================================================
  //  u8g2.begin();
  //  u8g2.setDrawColor(1);
  //  u8g2.setBitmapMode(1);
  //  delay(500);
  //  tft.fillScreen(TFT_BLACK);
  //  tft.setCursor(10, 35); tft.setTextColor(TFT_WHITE);  tft.setTextSize(2); tft.println("WAITING FOR BLUETOOTH");

  //  Keyboard.begin();
  //  Mouse.begin();


  Serial.println();
  Serial.println(F("------------------------------------"));
  Serial.println(F("All passed and ready"));
  Serial.println(F("------------------------------------"));


  tft.fillScreen(TFT_BLACK);


}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void OSCMsgReceive(void) {
  OSCMessage msg;
  int size = Udp.parsePacket();

  if (size > 0) {
    while (size--) {
      msg.fill(Udp.read());
    }
    if (!msg.hasError()) {
      msg.dispatch("/led", led);
    } else {
      error = msg.getError();
      Serial.print("error: ");
      Serial.println(error);
    }
  }
}


void loop() {
  //
  Serial.println(F("00!"));
  btnTest();
  initTest();
  Serial.println(F("01!"));

}

//
//
void mpuLoop() {
  // ================================================================
  // ===                      MPU6050 LOOP                       ===
  // ================================================================

  if (!dmpReady) return;                                                    // if programming failed, don't try to do anything
  mpuInterrupt = true;
  fifoCount = mpu.getFIFOCount();                                           // get current FIFO count
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)                           // check for overflow (this should never happen unless our code is too inefficient)
  {
    mpu.resetFIFO();                                                      // reset so we can continue cleanly
    Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x01)                                             // otherwise, check for DMP data ready interrupt (this should happen frequently)
  {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();        // wait for correct available data length, should be a VERY short wait
    mpu.getFIFOBytes(fifoBuffer, packetSize);                             // read a packet from FIFO
    fifoCount -= packetSize;                                              // track FIFO count here in case there is > 1 packet available
#ifdef OUTPUT_READABLE_YAWPITCHROLL                                               // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yaw = ypr[1] / PI * 180;
    pitch = ypr[2] / PI * 180;
    roll = ypr[0] / PI * 180;
    Serial.print("ypr\t");
    Serial.print(yaw);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.println(roll);
    //    tft.setFreeFont(FF18);
    //    tft.fillScreen(TFT_BLACK);
    //    tft.setCursor(10, 35); tft.setTextColor(TFT_WHITE);  tft.setTextSize(1); tft.println("YAW: ");
    //    tft.setCursor(95, 35); tft.setTextColor(TFT_WHITE);  tft.setTextSize(1); tft.println(yaw);
    //    tft.setCursor(10, 65); tft.setTextColor(TFT_WHITE);  tft.setTextSize(1); tft.println("PITCH: ");
    //    tft.setCursor(95, 65); tft.setTextColor(TFT_WHITE);  tft.setTextSize(1); tft.println(pitch);
    //    tft.setCursor(10, 95); tft.setTextColor(TFT_WHITE);  tft.setTextSize(1); tft.println("ROLL: ");
    //    tft.setCursor(95, 95); tft.setTextColor(TFT_WHITE);  tft.setTextSize(1); tft.println(roll);
    //    tft.setCursor(10, 125); tft.setTextColor(TFT_WHITE);  tft.setTextSize(1); tft.println("BTN MODES: ");
    //    tft.setCursor(170, 125); tft.setTextColor(TFT_WHITE);  tft.setTextSize(1); tft.println( proxBtn);

#endif
#ifdef OUTPUT_READABLE_YAWPITCHROLL_OSC                                               // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yaw = ypr[1] / PI * 180;
    pitch = ypr[2] / PI * 180;
    roll = ypr[0] / PI * 180;
    Serial.print("ypr\t");
    Serial.print(yaw);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.println(roll);

#endif


#ifdef OUTPUT_TEAPOT_OSC
#ifndef OUTPUT_READABLE_QUATERNION

    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
#endif
    // Send OSC message
    OSCMessage msg("/imuquat");
    msg.add((float)q.w);
    msg.add((float)q.x);
    msg.add((float)q.y);
    msg.add((float)q.z);

    Udp.beginPacket(destIp, destPort);
    msg.send(Udp);
    Udp.endPacket();

    msg.empty();
#endif

  }

}

void initTest() {

  mpuLoop();//READ MPU
  //  QUATERNION
  acXOSC();
  acYOSC();
  acZOSC();
  acWOSC();
  //  YPR
  imuYaw();
  imuPitch();
  imuRoll();
  //  MOUSE VALUE
  sendMouseX();
  sendMouseY();
}

//YYYY
void imuYaw() {
  //  calcRotation();
  // read btnInput and send OSC
  OSCMessage msgOut("/yaw");
  Serial.print("yaw: ");
  Serial.println(yaw);
  msgOut.add(yaw);
  Udp.beginPacket(destIp, destPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
  // delay(100);
}


void imuPitch() {
  //  calcRotation();
  // read btnInput and send OSC
  OSCMessage msgOut("/pitch");
  Serial.print("pitch: ");
  Serial.println(pitch);
  msgOut.add(pitch);
  Udp.beginPacket(destIp, destPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
  // delay(100);
}

void imuRoll() {
  //  calcRotation();
  // read btnInput and send OSC
  OSCMessage msgOut("/roll");
  Serial.print("roll: ");
  Serial.println(roll);
  msgOut.add(roll);
  Udp.beginPacket(destIp, destPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
  // delay(100);
}

void sendMouseX() {

  blinkState = !blinkState;                                             // blink LED to indicate activity
  //  vertValue = pitch - vertZero;// moving up/down
  horzValue = roll - horzZero;// moving left/right
  vertZero = pitch;
  horzZero = roll;
  float vValue = horzValue * sensitivity;
  OSCMessage msgOut("/vValue");
  Serial.print("vValue: ");
  Serial.println(roll);
  msgOut.add(vValue);
  Udp.beginPacket(destIp, destPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
}

void sendMouseY() {

  blinkState = !blinkState;                                             // blink LED to indicate activity
  vertValue = pitch - vertZero;// moving up/down
  //  horzValue = roll - horzZero;// moving left/right
  vertZero = pitch;
  horzZero = roll;
  float yValue = vertValue * sensitivity;
  OSCMessage msgOut("/hValue");
  Serial.print("hValue: ");
  Serial.println(roll);
  msgOut.add(yValue);
  Udp.beginPacket(destIp, destPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
}

//XXXX
void acXOSC() {
  OSCMessage msgOut("/qx");
  msgOut.add((float)q.x);
  Serial.print("qx: ");
  Serial.println(q.x);
  Udp.beginPacket(destIp, destPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
  // delay(100);
}

//YYYY
void acYOSC() {
  OSCMessage msgOut("/qy");
  Serial.print("qy: ");
  Serial.println(q.y);
  msgOut.add((float)q.y);
  Udp.beginPacket(destIp, destPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
  // delay(100);
}

//ZZZZ
void acZOSC() {
  OSCMessage msgOut("/qz");
  Serial.print("qz: ");
  Serial.println(q.z);
  msgOut.add((float)q.z);
  Udp.beginPacket(destIp, destPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
  // delay(100);
}

//WWWWW
void acWOSC() {
  OSCMessage msgOut("/qw");
  Serial.print("AccW: ");
  Serial.print("qw: ");
  Serial.println(q.w);
  msgOut.add((float)q.w);
  Udp.beginPacket(destIp, destPort);
  msgOut.send(Udp);
  Udp.endPacket();
  msgOut.empty();
  // delay(100);
}

void led(OSCMessage &msg) {
  if (msg.isFloat(0)) {
    ledState = msg.getFloat(0);
  }
  if (msg.isInt(0)) {
    ledState = msg.getInt(0);
  }


  digitalWrite(2, ledState);
  Serial.print("/led: ");
  Serial.println(ledState);
}
