#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// The serial connection to the GPS module
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The GPS module's connection is made with software serial
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

void setup() {
  Serial.begin(115200); // Start the serial communication with a computer
  gpsSerial.begin(GPSBaud); // Start the GPS module communication
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) { // If new data is available from GPS
      if (gps.location.isValid()) {
        // Print the location if it's valid
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6); // 6 decimal places
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6); // 6 decimal places
      } else {
        Serial.println("Waiting for valid GPS signal...");
      }
    }
  }
}
