#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "LSM6DS_LIS3MDL.h"

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10

uint32_t timestamp;

void setup() {
  Serial.begin(115200);
  while (!Serial) yield();

  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  }

  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }
  
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();

  Wire.setClock(400000); // 400KHz
}


void loop() {
  float roll, pitch, heading;
  float gx, gy, gz;

  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);

  gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
  gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
  gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

  Serial.print(accel.acceleration.x, 4); Serial.print(" ");
  Serial.print(accel.acceleration.y, 4); Serial.print(" ");
  Serial.print(accel.acceleration.z, 4); Serial.print(" ");
  Serial.print(gx, 4); Serial.print(" ");
  Serial.print(gy, 4); Serial.print(" ");
  Serial.print(gz, 4); Serial.print(" ");
  Serial.print(mag.magnetic.x, 4); Serial.print(" ");
  Serial.print(mag.magnetic.y, 4); Serial.print(" ");
  Serial.print(mag.magnetic.z, 4); Serial.println(" ");

  delay(10);
}
