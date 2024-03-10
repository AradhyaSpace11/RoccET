#include <Wire.h>
#include <Adafruit_BMP085.h> // You might need to change this depending on the library you install

Adafruit_BMP085 bmp;

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud
  if (!bmp.begin()) { // Initialize the BMP180 sensor
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1) {} // If sensor not detected, halt program
  }
}

void loop() {
  float temperature = bmp.readTemperature(); // Read temperature (not used directly for altitude but useful for diagnostics)
  float pressure = bmp.readPressure(); // Read the atmospheric pressure
  float altitude = bmp.readAltitude(); // Calculate altitude. Can also use bmp.readAltitude(seaLevelPressure) if you know the current sea level pressure

  // Print the results to the serial monitor
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" Pa");

  Serial.print("Altitude = ");
  Serial.print(altitude);
  Serial.println(" meters");

  delay(1000); // Wait for a second before repeating the loop
}
