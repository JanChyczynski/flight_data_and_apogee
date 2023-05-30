#include "Wire.h"
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SD.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "apogeum_finder.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "imu_apogee_finder.h"

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
Adafruit_BMP280 bmp; // I2C
Apogeum_finder height_apogee_finder;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
I2Cdev   I2C_M;
Imu_apogee_finder imu_apogee_finder;

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t   mx, my, mz;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

#define sample_num_mdate  5000

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

//calibration parameter:
//1.00         -2.00         -13.00
static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

const int sdCardChipSelect = 4;
File myFile;

void initialiseSdCard(){
  Serial.print("Initializing SD card...");
  if (!SD.begin(sdCardChipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the sdCardChipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    //while (true);
  }
  randomSeed(analogRead(0));
  String filename = "test"+(String)random(1<<31)+".csv";
  myFile = SD.open(filename, FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.println("file "+filename+" created succesfully");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening "+filename);
    //while(true);
  }
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  // initialize device
  while (!Serial);
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  delay(1000);
  Serial.println("     ");

//  Mxyz_init_calibrated();



  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    //while (1);
  }
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  initialiseSdCard();

  Serial.println("Nr,Gx(degress/s),Gy(degress/s),Gz(degress/s),Ax(g),Ay(g),Az(g),Mx,My,Mz,Imu_apogee,T(*C),Pressure(Pa),alt(m),height_apogee");
}

int i = 0;
void loop() {

  getAccel_Data();
  getGyro_Data();
  getCompassDate_calibrated(); // compass data has been calibrated here
  getHeading();				//before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
  getTiltHeading();

  imu_apogee_finder.insert_accelerations(Axyz);
  double alt = bmp.readAltitude(1013.25); /* Adjusted to local forecast! */
  height_apogee_finder.insertAltitude(alt);

  Serial.print("imu: ");
  Serial.print(Gxyz[0]);
  Serial.print(",");
  Serial.print(Gxyz[1]);
  Serial.print(",");
  Serial.print(Gxyz[2]);
  Serial.print(",");
  Serial.print(Axyz[0]);
  Serial.print(",");
  Serial.print(Axyz[1]);
  Serial.print(",");
  Serial.print(Axyz[2]);
  Serial.print(",");
  Serial.print(Mxyz[0]);
  Serial.print(",");
  Serial.print(Mxyz[1]);
  Serial.print(",");
  Serial.println(Mxyz[2]);
  Serial.print("imu_apogee: ");
  Serial.println(imu_apogee_finder.get_reached_apogee());
  Serial.print("bmp: ");
  Serial.print(bmp.readTemperature());
  Serial.print(",");
  Serial.print(bmp.readPressure());
  Serial.print(",");
  Serial.println(alt);
  Serial.print("height_apogee: ");
  Serial.println(height_apogee_finder.get_reached_apogeum());

  print_data_to_file();
  
  delay(25);
}


void print_data_to_file(){
  myFile.print(i++);
  myFile.print(",");
  myFile.print(Gxyz[0]);
  myFile.print(",");
  myFile.print(Gxyz[1]);
  myFile.print(",");
  myFile.print(Gxyz[2]);
  myFile.print(",");
  myFile.print(Axyz[0]);
  myFile.print(",");
  myFile.print(Axyz[1]);
  myFile.print(",");
  myFile.print(Axyz[2]);
  myFile.print(",");
  myFile.print(Mxyz[0]);
  myFile.print(",");
  myFile.print(Mxyz[1]);
  myFile.print(",");
  myFile.print(Mxyz[2]);
  myFile.print(",");
  myFile.print(imu_apogee_finder.get_reached_apogee());
  myFile.print(",");
  myFile.print(bmp.readTemperature());
  myFile.print(",");
  myFile.print(bmp.readPressure());
  myFile.print(",");
  myFile.print(bmp.readAltitude(1013.25));
  myFile.print(",");
  myFile.println(height_apogee_finder.get_reached_apogeum());
}