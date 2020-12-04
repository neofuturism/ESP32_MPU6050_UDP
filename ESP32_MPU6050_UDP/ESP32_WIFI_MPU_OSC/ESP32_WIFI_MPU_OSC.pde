//*Modified to work with ESP32- MPUOSCTeapot Processing demo for MPU6050 DMP modified for OSC
// I2C device class (I2Cdev) demonstration Processing sketch for MPU6050 DMP output
// 6/20/2012 by>>>>>>>>>> Jeff Rowberg <jeff@rowberg.net> <<<<<<<<<<
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
/**
 *   https://gitub.com/jrowberg/i2cdevlib
 *   The original demo uses serial port I/O which has been replaced with
 *   OSC UDP messages in this sketch.
 *   The MPU6050 is connected to an ESP8266 with battery so it is completely
 *   wire free.
 *   Tested on Processing 3.3.5 running on Ubuntu Linux 14.04
 *   Dependencies installed using Library Manager
 * Open Sound Control library
 *   oscP5 website at http://www.sojamo.de/oscP5
 * ToxicLibs
 *   quaternion functions http://toxiclibs.org/
 *
 */

// Install oscP5 using the IDE library manager.
// From the IDE menu bar, Sketch | Import Library | Add library.
// In the search box type "osc".
import oscP5.*;
import netP5.*;
// Install ToxicLibs using the IDE library manager
// From the IDE menu bar, Sketch | Import Library | Add library.
// In the search box type "toxic".
import toxi.geom.*;
import toxi.processing.*;

ToxiclibsSupport gfx;
NetAddress gyroscope; 

Quaternion quat = new Quaternion(1, 0, 0, 0);

float qx = 0;
float qy = 0;
float qz = 0;
float qw = 0;

float yaw = 0;
float pitch = 0;
float roll = 0;


OscP5 oscP5;

String ESP32IP=("192.168.0.172");
int ESP32PORT = 8000;

void oscSetup() {
  oscP5 = new OscP5(this, 8000);
  gyroscope = new NetAddress(ESP32IP, ESP32PORT); // not sure if i even need to import netP5
  oscP5.plug(this, "imu", "/imuquat");
}
void setup() { 
  //fullScreen(P3D);//RUN FULLSCREEN
  size(960, 600, P3D);// NORMAL SIZE 960X600
  textSize(20); //SET TEXT SIZE
  gfx = new ToxiclibsSupport(this);

  // setup lights and antialiasing
  lights();
  smooth(8);

  oscSetup();
}
void oscEvent(OscMessage mensagem) {
  // tests if any message is being received at all
  //fill(255);
  //rect(50, 30, 100, 100);

 if (mensagem.checkAddrPattern("/qw")) qw = mensagem.get(0).floatValue();
  else if (mensagem.checkAddrPattern("/qx")) qx  = mensagem.get(0).floatValue();
  else if (mensagem.checkAddrPattern("/qy")) qy = mensagem.get(0).floatValue();
  else if (mensagem.checkAddrPattern("/qz")) qz = mensagem.get(0).floatValue();
  
  //else if (mensagem.checkAddrPattern("/yaw")) yaw = mensagem.get(0).floatValue();
  //else if (mensagem.checkAddrPattern("/pitch")) pitch = mensagem.get(0).floatValue();
  //else if (mensagem.checkAddrPattern("/roll")) roll = mensagem.get(0).floatValue();
}


public void imu(float quant_w, float quant_x, float quant_y, float quant_z) {
  quat.set(quant_w, -quant_x, quant_y, quant_z);
}

void draw() {
  background(250, 200, 0);
  textUi(); 

  //READ QUATERNION
  pushMatrix();
  translate(width / 2, height / 2);  // translate everything to the middle of the viewport

  float[] axis = quat.toAxisAngle();
  rotate(axis[0], -axis[1], axis[3], axis[2]);

  //ELEMENTS
  lights();
  //noStroke();// UNCOMMENT THIS FOR SMOOTHNESS
  fill(30, 200, 0, 200);
  box(30, 30, 400); //BAR
  fill(0, 30, 200, 200);
  box(30, 400, 30);//BAR
  fill(200, 30, 0, 200);
  box(400, 30, 30);//BAR
  fill(255, 10);
  box(140);
  fill(255, 255, 255, 100);
  sphere(100);
  sphere(200);
  popMatrix();
}

void textUi() {
  fill(33);
  text("Quat_x = " + qx, 20, 400);
  text("Quat_y = " + qy, 20, 430);
  text("Quat_z = " + qz, 20, 460);
  text("Quat_w = " + qw, 20, 490);
  
  //text("Yaw = " + yaw, 400,400);
  //text("Pitch = " + pitch, 400, 430);
  //text("Roll = " + roll, 400, 460);
}
