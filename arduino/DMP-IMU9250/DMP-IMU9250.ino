/* Reaction Wheel controller
 *
 * The following code provides a control for PLATHON project's CubeSat with MAXON motor.
 *
 * The following libraries are used:
 * STM32 Bluepill microcontroller package from http:/ /dan.drown.org/stm32duino/package_STM32duino_index.json
 * MPU9250 IMU library by Rafa Castalla used (https://github.com/rafacastalla/MPU9250-1)
 * 
 * DGD 4/3/23
 */

// −−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−− DEFINITIONS
// −−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−− LIBRARIES

#include <Arduino.h>     // Arduino library
#include <stdint.h>      // Integer types library
#include <Wire.h>        // I2C library

//IMU librarys
#include "MPU6050_6Axis_MotionApps20.h" //Mediante esta biblioteca se pueden obtener datos del DMP (Digital Motion Processor) de la IMU
#include "I2Cdev.h"                     //Mediante esta biblioteca se pueden obtener datos del DMP de la IMU
#include "helper_3dmath.h"              //Mediante esta biblioteca se pueden realizar operaciones de cuaterniones


// −−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−− DEFINITIONS

#define STM32_CLOCK 72000          // Internal STM32 Clock [kHz](72MHz)
#define PI 3.14159265              // Number PI
#define Rad_to_deg 57.29577951     // Convert radians to degrees
#define Deg_to_rad 0.01745329      // Convert degrees to radians

 
//____________________________________________________IMU________________________________________________________________________
const int mpuAddress = 0x68;       // Se define la dirección de la IMU, puede ser 0x68 o 0x69
MPU6050 mpu(mpuAddress);           //Se crea un objeto MPU6050 para poder extraer datos de la DMP
int fifoCount = 0, packetSize;     //Se crea un contador de valores en el FIFO de la MPU y el tamaño del paquete que obtendremos
byte fifoBuffer[42];               //Se crea el buffer que se utilizará para obtener los datos roll, pitch y yaw.
float roll, pitch, yaw, sqx, sqy, sqz, sqw, test; //Se crea tanto las variables de roll, pitch y yaw como las variables "float"
                                                  //intermedias necesarias para obtenerlas
Quaternion q;                      //Se crea el cuaternión necesario para obtener los valores de pitch, yaw y roll
VectorInt16 v;
float vz, vy, vx;                          // ¿¿¿ velocidad angular eje Z???

// −−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−− GLOBAL VARIABLES-------------------------------------------

const float pi = 3.141592653;
float Pitch_deg, Roll_deg, Yaw_deg = 0;                     // Pitch, Roll and Yaw (degrees)
int OBC_mode_value = 0;           // Value of On Board Computer mode (waiting, positioning, detumbling, etc.)
int OBC_data_value = 0;           // Value of On Board Computer data (desired angle turn, etc.)

// −−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−− FUNCTIONS
// −−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−− GENERAL USE FUNCTIONS
// −−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−− OBC Functions

void setup() {

  delay(1000); //  wait to let open the serial
// −−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−− Timers
  
// −−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−− Initiations
  Serial.begin(9600);                              // Serial1 BLUETOOTH, emulating OBC
  Wire.begin();
  Serial.println("START");

// −−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−− Setups

  
  //pinMode(0x68, OUTPUT);    // ?????? NCS Pin definition ?????
  
  
// −−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−− IMU Setup
//IMU
  mpu.initialize();                                 //Inicialización de la imu
  delay(1000); 
  Serial.println("MPU successfully inicialized");
  mpu.dmpInitialize();                              //Inicialización del DMP
  mpu.setDMPEnabled(true);                          //Habilitación del DMP
  Serial.println("successfull DMP acces");
  packetSize = mpu.dmpGetFIFOPacketSize();          //Se obtiene el tamaño del paquete que obtendremos del DMP (42 bytes)


//small imu
//-876  286 1683  162 16  52
  mpu.setXAccelOffset(-876);                        // Valores de calibracion de la IMU
  mpu.setYAccelOffset(286);                        // Obtenidos por el procedimiento de calibracion externo
  mpu.setZAccelOffset(1683);
  mpu.setXGyroOffset(162);
  mpu.setYGyroOffset(16);
  mpu.setZGyroOffset(52);

//wrong, seeed imu
//  mpu.setXAccelOffset(-394);                        // Valores de calibracion de la IMU
//  mpu.setYAccelOffset(416);                        // Obtenidos por el procedimiento de calibracion externo
//  mpu.setZAccelOffset(17221);
//  mpu.setXGyroOffset(118);
//  mpu.setYGyroOffset(61);
//  mpu.setZGyroOffset(108);

//original
//    mpu.setXAccelOffset(6491);                        // Valores de calibracion de la IMU
//  mpu.setYAccelOffset(4850);                        // Obtenidos por el procedimiento de calibracion externo
//  mpu.setZAccelOffset(9354);
//  mpu.setXGyroOffset(-95);
//  mpu.setYGyroOffset(-2);
//  mpu.setZGyroOffset(39);
  Serial.println("MPU calibrated");

  
  //Se define el cuaternión inicial de actitud
  q.w = 1;
  q.x = 0;
  q.y = 0;
  q.z = 0;
  
  delay(100);                                       //Se da un margen de tiempo para que todos los procesos se completen
}

void loop() {
 IMU();
  Serial.print(Roll_deg);
 Serial.print("   ");
  Serial.print(Pitch_deg);
 Serial.print("   ");
 Serial.print(Yaw_deg);
 Serial.print("   ");
 Serial.print(vx);
 Serial.print(" ");
  Serial.print(vy);
 Serial.print(" ");
 Serial.println(vz);

 
 delay(100);
   }
