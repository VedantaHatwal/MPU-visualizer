#include "FastIMU.h"
#include "Madgwick.h"
#include <Wire.h>

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
MPU6500 IMU;               //Change to the name of any supported IMU!

// Currently supported IMUS: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL QMI8658

calData calib = { 0 };  //Calibration data
AccelData IMUAccel;    //Sensor data
GyroData IMUGyro;
MagData IMUMag;
Madgwick filter;


unsigned int printCounter = 0;
const unsigned int PRINT_EVERY_N = 200; // Print every 200th update
const float beta=0.001;



void setup() {
  Wire.begin();
  Wire.setClock(1000000); //400khz clock

  Serial.begin(115200);


  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

#ifdef PERFORM_CALIBRATION
  Serial.println("FastIMU Calibrated Quaternion example");
  if (IMU.hasMagnetometer()) {
    delay(1000);
    Serial.println("Move IMU in figure 8 pattern until done.");
    delay(3000);
    IMU.calibrateMag(&calib);
    Serial.println("Magnetic calibration done!");
  }
  else {
    delay(1000);
  }
  Serial.println("Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  if (IMU.hasMagnetometer()) {
    Serial.println("Mag biases X/Y/Z: ");
    Serial.print(calib.magBias[0]);
    Serial.print(", ");
    Serial.print(calib.magBias[1]);
    Serial.print(", ");
    Serial.println(calib.magBias[2]);
    Serial.println("Mag Scale X/Y/Z: ");
    Serial.print(calib.magScale[0]);
    Serial.print(", ");
    Serial.print(calib.magScale[1]);
    Serial.print(", ");
    Serial.println(calib.magScale[2]);
  }
  delay(5000);
calib.accelBias[0] = 0;
calib.accelBias[1] = 0; 
calib.accelBias[2] = 0;





  IMU.init(calib, IMU_ADDRESS);

  filter.begin(beta);
#endif
    Serial.print(IMUAccel.accelX);
  Serial.print(",   ");
    Serial.print(IMUAccel.accelY);
  Serial.print(",   ");
    Serial.println(IMUAccel.accelZ);



}

void loop() {
  IMU.update();
  IMU.getAccel(&IMUAccel);
  IMU.getGyro(&IMUGyro);
  if (IMU.hasMagnetometer()) {
    IMU.getMag(&IMUMag);
    filter.update(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ, IMUMag.magX, IMUMag.magY, IMUMag.magZ);
  }
  else {
    filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);
  }



  printCounter++;
  if (printCounter >= PRINT_EVERY_N) {
    printCounter = 0;
    
    // Print quaternion
    Serial.print(filter.getQuatW());
    Serial.print(",");
    Serial.print(filter.getQuatX());
    Serial.print(",");
    Serial.print(filter.getQuatY());
    Serial.print(",");
    Serial.println(filter.getQuatZ());

  }
}


