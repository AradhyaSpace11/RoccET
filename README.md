# RoccET

This Repository Contains program files for the open source project of Quark Amateur TVC Rocket Programme. This code gives a representation of the algorithm used to steer a TVC Rocket by controlling its pitch and roll such that the projectile moves in a straight line wrt the Ground. Please note that the code is complete in itself to be used on a test model and any enthusiast can use and upgrade the code to suit their project. The aim of this project is to code a flight controller such that it returns the rocket back on ground upright and intact. Before you compile (or even start this project) you need to download the `Wire.h`, `MPU6050.h`, and `Servo.h` libraries.

```cpp
#include <Wire.h> 
#include <MPU6050.h>
#include <Servo.h> 

Servo sg90;          

int servo_pin = 2;
int servo_pin2 = 3;

MPU6050 sensor;

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
    sg90.attach(servo_pin);
    sg90.attach(servo_pin2);

    Wire.begin();
    Serial.begin(9600);
    sensor.initialize();

    Serial.println(sensor.testConnection() ? "..." : "..");


    delay(1000);
}

// Here's a tip, you can add a Kalman filter to evaluate and filter out the data to give out a smooth angle change

void loop() {
    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax = map(ax, -17000, 17000, 0, 180);
    ay = map(ay, -17000, 17000, 0, 180);

    Serial.println(ax);

    sg90.write(ax);
    sg90.write(ay);

    delay(200);
    // Delays are important for preventing the input terminal from getting overloaded with information.
}
```

