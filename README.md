# ESP32 MPU6050 UDP TRANSFER
Use this script to send the MPU6050 values to Processing using UDP. 
This project require an ESP32/MPU6050 and 4 pushbuttons.

## SETUP

This project requires [Processing](https://processing.org/) and [Arduino](https://arduino.cc) with the [esp32 core](https://github.com/espressif/arduino-esp32) installed.

**I used a TTGO DISPLAY for this project**
  
###Connection for the IMU6050
 
 * 3.3V  >>>    VCC     >>>      Power
 * GND   >>>    GND     >>>      Ground
 * 21    >>>    SDA     >>>      I2C Data
 * 22    >>>    SCL     >>>      I2C Clock
###Button setup, I need this for my project, but you can comment out the entire "inputSetup();
 * PUSHBUTTON BTN_A >>>    4
 * BTN_B >>>    37
 * BTN_C >>>    13
 * BTN_D >>>    15
 *
 *

###THIS PROJECT USES TFT_eSPI, Look for "myUser_TTGO_T_Display.h" file to setup the display.

Compile and upload "ESP32_WIFI_MPU6050.ino" to your esp32. Once the ESP32 is flashed check serial to see if the gyroscope is running. If it is, turn off your device.

Now run "ESP32_WIFI_MPU_OSC.pde", turn on your ESP32 and leave it a few sec (10) for it to get your position. Once it's done, move it around, the globe in the processing script should move.

# Library requirement

* [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI )
* [OSC](https://github.com/CNMAT/OSC)
* [Button Fever](https://github.com/mickey9801/ButtonFever)
