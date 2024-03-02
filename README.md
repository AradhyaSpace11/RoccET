# RoccET

This Repository Contains program files for the open source project of Quark Amateur TVC Rocket Programme. This code gives a representation of the algorithm used to steer a TVC Rocket by controlling its pitch and roll such that the projectile moves in a straight line wrt the Ground. Please note that the code is complete in itself to be used on a test model and any enthusiast can use and upgrade the code to suit their project. The aim of this project is to code a flight controller such that it returns the rocket back on ground upright and intact. Before you compile (or even start this project) you need to download the `Wire.h`, `MPU6050.h`, and `Servo.h` libraries.

```cpp
#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h> // Make sure to install this library

Servo servoX; // Servo for X-axis control
Servo servoY; // Servo for Y-axis control

MPU6050 mpu;

// PID constants
double kp = 1.0, ki = 0.0, kd = 0.2; // Example values, adjust based on testing
double setpoint = 0; // Assuming setpoint is the vertical orientation

double pidInputX, pidOutputX, pidSetpointX = setpoint;
double pidInputY, pidOutputY, pidSetpointY = setpoint;

unsigned long lastTime;
double errSumX, lastErrX;
double errSumY, lastErrY;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
  mpu.initialize();
  servoX.attach(9); // Attach servo on pin 9
  servoY.attach(10); // Attach servo on pin 10
  
  servoX.write(90); // Initialize servos to 90 degrees
  servoY.write(90);
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
  }
}

void loop() {
  // Read orientation from MPU6050
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Convert to degrees (simplified, consider using proper conversion)
  pidInputX = ax;
  pidInputY = ay;

  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);

  ComputePID(pidInputX, pidOutputX, pidSetpointX, errSumX, lastErrX, timeChange);
  ComputePID(pidInputY, pidOutputY, pidSetpointY, errSumY, lastErrY, timeChange);

  lastTime = now;

  int servoXPosition = 90 + pidOutputX; // Assuming PID output is in range that maps to servo position
  int servoYPosition = 90 + pidOutputY;

  servoX.write(constrain(servoXPosition, 0, 180));
  servoY.write(constrain(servoYPosition, 0, 180));

  delay(100); // Delay for stability, adjust as needed
}

void ComputePID(double &input, double &output, double &setpoint, double &errSum, double &lastErr, double timeChange) {
  double error = setpoint - input;
  errSum += (error * timeChange);
  double dErr = (error - lastErr) / timeChange;

  output = kp * error + ki * errSum + kd * dErr;
  lastErr = error;
}

```

