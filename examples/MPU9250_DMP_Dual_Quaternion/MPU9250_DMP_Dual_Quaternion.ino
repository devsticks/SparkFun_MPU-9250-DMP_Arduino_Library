/************************************************************
MPU9250_DMP_Dual_Quaternion
Two-IMU Quaternion example for MPU-9250 DMP Arduino Library 

Devin Stickells
Version creation date: October 1, 2019
https://github.com/devsticks/SparkFun_MPU9250_DMP_Arduino_Library

Original by Jim Lindblom @ SparkFun Electronics
Creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

The MPU-9250's digital motion processor (DMP) can calculate
four unit quaternions, which can be used to represent the
rotation of an object.

This exmaple demonstrates how to configure the DMP to 
calculate quaternions, and prints them out to the serial
monitor. It also calculates pitch, roll, and yaw from those
values.

Development environment specifics:
Arduino IDE 1.6.12
Adafruit Huzzah32 ESP32

Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
- Espressif ESP32
*************************************************************/

#include <SparkFunMPU9250-DMP.h>
#include <Quaternion.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

#ifdef defined(SAMD)
 #define SerialPort SerialUSB
#else
  #define SerialPort Serial
#endif

// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO_1 0
#define ADO_2 1
// Define I2C addresses of the two MPU9250
#define MPU9250_ADDRESS_1 0x68   // Device address when ADO = 0
#define MPU9250_ADDRESS_2 0x69   // Device address when ADO = 1

MPU9250_DMP imu1;
MPU9250_DMP imu2;

void setup() 
{
  int MPU1 = 0x68;
  int MPU2 = 0x69;
  
  SerialPort.begin(115200);

  // Call imu.begin() to verify communication and initialize
  if (imu1.begin(0x69) != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250 1");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  if (imu2.begin(0x68) != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250 2");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  if (imu1.selfTest() != INV_SUCCESS) {
    Serial.println("Self test failed");
  }

  if (imu2.selfTest() != INV_SUCCESS) {
    Serial.println("Self test failed");
  }

  Serial.println("1 " + String(imu1.getAddr()) + ", 2 " + String(imu2.getAddr()));
  
  if (imu1.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              20) != INV_SUCCESS) // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
  {
    while (1)
    {
      SerialPort.println("Unable to start DMP on MPU-9250 1");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  if (imu2.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              20)  != INV_SUCCESS) // Set DMP FIFO rate to 10 Hz
  {
    while (1)
    {
      SerialPort.println("Unable to start DMP on MPU-9250 2");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
      
}

void loop() 
{
  // Check for new data in the FIFO
  if ( imu1.fifoAvailable() || imu2.fifoAvailable() )
  {
      printIMUData();
  }
}

void printIMUData(void)
{  
  // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
  // are all updated.
  // Quaternion values are, by default, stored in Q30 long
  // format. calcQuat turns them into a float between -1 and 1
    imu1.dmpUpdateFifo();
    imu2.dmpUpdateFifo();
        
    Quaternion q1;
    q1.a = imu1.calcQuat(imu1.qw);
    q1.b = imu1.calcQuat(imu1.qx);
    q1.c = imu1.calcQuat(imu1.qy);
    q1.d = imu1.calcQuat(imu1.qz);
   
    Quaternion q2;
    q2.a = imu2.calcQuat(imu2.qw);
    q2.b = imu2.calcQuat(imu2.qx);
    q2.c = imu2.calcQuat(imu2.qy);
    q2.d = imu2.calcQuat(imu2.qz);
    
    SerialPort.println(String(q1.a) + "," + String(q1.b) + "," + String(q1.c) + "," + String(q1.d) + "," + String(q2.a) + "," + String(q2.b) + "," + String(q2.c) + "," + String(q2.d) );
    SerialPort.println();
}
