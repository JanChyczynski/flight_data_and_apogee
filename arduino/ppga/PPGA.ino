#include "Wire.h"
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <Servo.h>

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

#define SER_OUT if (1)

bool real_launch = 0;

Adafruit_BMP280 bmp; // I2C
Apogeum_finder height_apogee_finder;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
I2Cdev   I2C_M;
Imu_apogee_finder imu_apogee_finder;

Servo servo;

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
String filename;

File myFile;

void idicate_setup_failure() {
  servo.write(150);
  delay(1000);   
  servo.write(0);
  delay(500);   
  servo.write(150);
  delay(1000);   
  servo.write(0);
  delay(200); 
}

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  SER_OUT Serial.begin(38400);

  // initialize device
  SER_OUT while (!Serial);
  SER_OUT Serial.println(F("Initializing I2C devices..."));
  Wire.begin();
  accelgyro.initialize();
  servo.attach(3);

  // verify connection
  SER_OUT Serial.println(F("Testing device connections..."));
  SER_OUT Serial.println(accelgyro.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  if (!accelgyro.testConnection() && real_launch){
    idicate_setup_failure();
  }

  delay(1000);
  SER_OUT Serial.println("     ");

//  Mxyz_init_calibrated();



  if (!bmp.begin()) {
    SER_OUT Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    if (real_launch){
      idicate_setup_failure();
    }
  }
  SER_OUT Serial.println(F("BMP280 working"));
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  initialiseSdCard();

  myFile.println(F("Nr,ms,Gx(degress/s),Gy(degress/s),Gz(degress/s),Ax(g),Ay(g),Az(g),Mx,My,Mz,Imu_apogee,T(*C),Pressure(Pa),alt(m),height_apogee"));
}

void initialiseSdCard(){
  SER_OUT Serial.print("Initializing SD card...");
  if (!SD.begin(sdCardChipSelect)) {
    SER_OUT Serial.println(F("initialization failed. Things to check:"));
    SER_OUT Serial.println(F("1. is a card inserted?"));
    SER_OUT Serial.println(F("2. is your wiring correct?"));
    SER_OUT Serial.println(F("3. did you change the sdCardChipSelect pin to match your shield or module?"));
    SER_OUT Serial.println(F("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!"));
    if (real_launch){
      idicate_setup_failure();
    }
  }
  int seed = analogRead(4)*1024 + analogRead(3);
  SER_OUT Serial.print(F("setting seed to "));
  SER_OUT Serial.println(seed);
  randomSeed(seed);

  String filename = "d"+(String)random(0,1<<14)+".csv";
  myFile = SD.open(filename, FILE_WRITE);

  if (myFile) {
    SER_OUT Serial.print(F("file "));
    SER_OUT Serial.print(filename);
    SER_OUT Serial.println(F(" created succesfully"));
  } else {
    // if the file didn't open, print an error:
    SER_OUT Serial.print(F("error opening "));
    SER_OUT Serial.println(filename);
    
    if (real_launch){
      idicate_setup_failure();
    }
  }
}
int loopCounter = 0;
void loop() {
 
  getAccel_Data();
  
  imu_apogee_finder.insert_accelerations(Axyz);
  float alt = bmp.readAltitude(1013.25); /* Adjusted to local forecast! */
  height_apogee_finder.insertAltitude(alt);

  if(height_apogee_finder.get_reached_apogeum()) {
    servo.write(150);
    delay(1500);   
    servo.write(0);
    delay(1500);   
  }

  SER_OUT Serial.print("imu a: ");
  SER_OUT Serial.print(Axyz[0]);
  SER_OUT Serial.print(F(","));
  SER_OUT Serial.print(Axyz[1]);
  SER_OUT Serial.print(F(","));
  SER_OUT Serial.println(Axyz[2]);
  SER_OUT Serial.print(F("imu_apogee: "));
  SER_OUT Serial.println(imu_apogee_finder.get_reached_apogee());
  SER_OUT Serial.print(F("bmp: "));
  SER_OUT Serial.print(bmp.readTemperature());
  SER_OUT Serial.print(F(","));
  SER_OUT Serial.print(bmp.readPressure());
  SER_OUT Serial.print(F(","));
  SER_OUT Serial.println(alt);
  SER_OUT Serial.print(F("height_apogee: "));
  SER_OUT Serial.println(height_apogee_finder.get_reached_apogeum());

  if ((loopCounter)%3 == 1){
    myFile.flush();
  }
  if ((loopCounter++)%3 == 0){
    print_data_to_file();
  }
  
}


int i = 0;
void print_data_to_file(){
  SER_OUT Serial.print(F("saving to SD... millis: "));
  SER_OUT Serial.println(millis());
  getGyro_Data();
  getCompassDate_calibrated(); // compass data has been calibrated here
  getHeading();				//before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
  getTiltHeading();

  myFile.print(i++);
  myFile.print(F(","));
  myFile.print(millis());
  myFile.print(F(","));
  myFile.print(Gxyz[0]);
  myFile.print(F(","));
  myFile.print(Gxyz[1]);
  myFile.print(F(","));
  myFile.print(Gxyz[2]);
  myFile.print(F(","));
  myFile.print(Axyz[0]);
  myFile.print(F(","));
  myFile.print(Axyz[1]);
  myFile.print(F(","));
  myFile.print(Axyz[2]);
  myFile.print(F(","));
  myFile.print(Mxyz[0]);
  myFile.print(F(","));
  myFile.print(Mxyz[1]);
  myFile.print(F(","));
  myFile.print(Mxyz[2]);
  myFile.print(F(","));
  myFile.print(imu_apogee_finder.get_reached_apogee());
  myFile.print(F(","));
  myFile.print(bmp.readTemperature());
  myFile.print(F(","));
  myFile.print(bmp.readPressure());
  myFile.print(F(","));
  myFile.print(bmp.readAltitude(1013.25));
  myFile.print(F(","));
  myFile.println(height_apogee_finder.get_reached_apogeum());
  
  
}