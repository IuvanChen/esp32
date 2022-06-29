#include "I2Cdev.h"
#include <Wire.h>
#include <MPU6050.h>
#include <BleMouse.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf


/* IMU Data */




BleMouse bleMouse;
MPU6050 imu;
int16_t ax, ay, az, gx, gy, gz;
int vx, vy;
int buttonL = 0; // IO0 button
int buttonR = 4;
int buttonLstate = HIGH; 
int buttonRstate = HIGH; 
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine


void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(buttonL, INPUT_PULLUP);
  pinMode(buttonR, INPUT_PULLUP);
  
  Serial.print("MPU6050 initializing");
  imu.initialize();
  while (!imu.testConnection()) { Serial.print("."); }
  Serial.println();  
  Serial.println("BLE Mouse starting !");
  bleMouse.begin();

#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  imu.initialize();

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  imu.getAcceleration(&ax, &ay, &az);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(ay, az) * RAD_TO_DEG;
  double pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(ay / sqrt(ax * ax + az * az)) * RAD_TO_DEG;
  double pitch = atan2(-ax, az) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  timer = micros();
}

void loop() {
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  vx =  (gx+15)/150;  
  vy = -(gy-100)/150;
  
  Serial.print("gx = ");   Serial.print(gx);
  Serial.print(", gy = "); Serial.print(gy);
  Serial.print("\t");
  Serial.print("X = ");    Serial.print(vx);
  Serial.print(", Y = ");  Serial.println(vy);
  
  bleMouse.move(vx, vy);
    
  buttonLstate = digitalRead(buttonL);
  buttonRstate = digitalRead(buttonR);  
  
  if (buttonLstate == LOW) { // press button to Ground
    bleMouse.click(MOUSE_LEFT);
    delay(100);
  } 
  else if(buttonRstate == LOW) { // press button to Ground
    bleMouse.click(MOUSE_RIGHT);
    delay(100);
  }
  delay(100);

   /* Update IMU sensor values */
  imu.getAcceleration(&ax, &ay, &az);
  imu.getRotation(&gx, &gy, &gz);
  
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(ay, az) * RAD_TO_DEG;
  double pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(ay / sqrt(ax * ax + az * az)) * RAD_TO_DEG;
  double pitch = atan2(-ax, az) * RAD_TO_DEG;
#endif

  double gyroXrate = gx / 131.0; // Convert to deg/s
  double gyroYrate = gy / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");
  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");
  Serial.print("\t");
#endif

  Serial.print(roll); Serial.print("\t");
  Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print("\t");
  Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");

#if 0 // Set to 1 to print the temperature
  Serial.print("\t");
  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature); Serial.print("\t");
#endif

  Serial.print("\r\n");
  delay(2);
}
