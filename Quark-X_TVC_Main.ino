#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h> // Make sure to install this library
#include "Kalman.h" // Make sure to have a Kalman filter library

Servo servoX; // Servo for X-axis control
Servo servoY; // Servo for Y-axis control

MPU6050 mpu;
Kalman kalmanX; // Kalman filter for X-axis
Kalman kalmanY; // Kalman filter for Y-axis

// PID constants remain unchanged
double kp = 1.0, ki = 0.0, kd = 0.2;
double setpoint = 0;

double pidInputX, pidOutputX, pidSetpointX = setpoint;
double pidInputY, pidOutputY, pidSetpointY = setpoint;

unsigned long lastTime;
double errSumX, lastErrX;
double errSumY, lastErrY;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  mpu.initialize();
  servoX.attach(9);
  servoY.attach(10);
  
  servoX.write(90);
  servoY.write(90);
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
  }

  // Kalman filter initialization might go here, depending on your Kalman library's requirements
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert gyroscope units to degrees per second, adjust according to your needs
  float gyroXRate = gx / 131.0;
  float gyroYRate = gy / 131.0;

  unsigned long now = millis();
  double dt = (now - lastTime) / 1000.0; // time change in seconds

  // Use Kalman filter to estimate the angle
  // Assume these functions are provided by your Kalman filter library
  float angleX = kalmanX.getAngle(ax, gyroXRate, dt);
  float angleY = kalmanY.getAngle(ay, gyroYRate, dt);

  pidInputX = angleX;
  pidInputY = angleY;

  ComputePID(pidInputX, pidOutputX, pidSetpointX, errSumX, lastErrX, dt);
  ComputePID(pidInputY, pidOutputY, pidSetpointY, errSumY, lastErrY, dt);

  lastTime = now;

  int servoXPosition = 90 + pidOutputX;
  int servoYPosition = 90 + pidOutputY;

  servoX.write(constrain(servoXPosition, 0, 180));
  servoY.write(constrain(servoYPosition, 0, 180));

  delay(100);
}
