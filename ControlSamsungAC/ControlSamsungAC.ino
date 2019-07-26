/* Copyright 2019 David Conran
*
* An IR LED circuit *MUST* be connected to the ESP8266 on a pin
* as specified by kIrLed below.
*
* TL;DR: The IR LED needs to be driven by a transistor for a good result.
*
* Suggested circuit:
*     https://github.com/crankyoldgit/IRremoteESP8266/wiki#ir-sending
*
* Common mistakes & tips:
*   * Don't just connect the IR LED directly to the pin, it won't
*     have enough current to drive the IR LED effectively.
*   * Make sure you have the IR LED polarity correct.
*     See: https://learn.sparkfun.com/tutorials/polarity/diode-and-led-polarity
*   * Typical digital camera/phones can be used to see if the IR LED is flashed.
*     Replace the IR LED with a normal LED if you don't have a digital camera
*     when debugging.
*   * Avoid using the following pins unless you really know what you are doing:
*     * Pin 0/D3: Can interfere with the boot/program mode & support circuits.
*     * Pin 1/TX/TXD0: Any serial transmissions from the ESP8266 will interfere.
*     * Pin 3/RX/RXD0: Any serial transmissions to the ESP8266 will interfere.
*   * ESP-01 modules are tricky. We suggest you use a module with more GPIOs
*     for your first time. e.g. ESP-12 etc.
*/
#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <ir_Samsung.h>

#if 0
#include "DHTesp.h"
#else
#include <DHT.h>
 
#define DHTPIN D4 //pin gpio 2 in sensor
#define DHTTYPE DHT11 // DHT 22 Change this if you have a DHT11
 
DHT dht(DHTPIN, DHTTYPE);
float t,h;
unsigned long previousMillis = 0; // will store last temp was read
const long interval = 2000; // interval at which to read sensor
#endif

const uint16_t kIrLed = 4;  // ESP8266 GPIO pin to use. Recommended: 4 (D2).
IRSamsungAc ac(kIrLed);     // Set the GPIO used for sending messages.

//DHTesp dht;

void printState() {
  // Display the settings.
  Serial.println("Samsung A/C remote is in the following state:");
  Serial.printf("  %s\n", ac.toString().c_str());
}

void setup() {
  ac.begin();
  Serial.begin(115200);
  delay(200);

  // Set up what we want to send. See ir_Samsung.cpp for all the options.
  Serial.println("Default state of the remote.");
  printState();
  Serial.println("Setting initial state for A/C.");
  ac.off();
  ac.setFan(kSamsungAcFanLow);
  ac.setMode(kSamsungAcCool);
  ac.setTemp(25);
  ac.setSwing(false);
  printState();

#if 0
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  String thisBoard= ARDUINO_BOARD;
  Serial.println(thisBoard);

  // Autodetect is not working reliable, don't use the following line
  // dht.setup(17);
  // use this instead: 
  dht.setup(D4, DHTesp::DHT22); // Connect DHT sensor to GPIO 17
#else
  dht.begin();
#endif
}

void loop() {
  getTempHumidity();
}

void controlAC() {
  // Turn the A/C unit on
  Serial.println("Turn on the A/C ...");
  ac.on();
  ac.send();
  printState();
  delay(15000);  // wait 15 seconds
  // and set to cooling mode.
  Serial.println("Set the A/C mode to cooling ...");
  ac.setMode(kSamsungAcCool);
  ac.send();
  printState();
  delay(15000);  // wait 15 seconds

  // Increase the fan speed.
  Serial.println("Set the fan to high and the swing on ...");
  ac.setFan(kSamsungAcFanHigh);
  ac.setSwing(true);
  ac.send();
  printState();
  delay(15000);

  // Change to Fan mode, lower the speed, and stop the swing.
  Serial.println("Set the A/C to fan only with a low speed, & no swing ...");
  ac.setSwing(false);
  ac.setMode(kSamsungAcFan);
  ac.setFan(kSamsungAcFanLow);
  ac.send();
  printState();
  delay(15000);

  // Turn the A/C unit off.
  Serial.println("Turn off the A/C ...");
  ac.off();
  ac.send();
  printState();
  delay(15000);  // wait 15 seconds
}

#if 0
void getTempHumidity() {
  delay(dht.getMinimumSamplingPeriod());

  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();

  Serial.print(dht.getStatusString());
  Serial.print("\t");
  Serial.print(humidity, 1);
  Serial.print("\t\t");
  Serial.print(temperature, 1);
  Serial.print("\t\t");
  Serial.print(dht.toFahrenheit(temperature), 1);
  Serial.print("\t\t");
  Serial.print(dht.computeHeatIndex(temperature, humidity, false), 1);
  Serial.print("\t\t");
  Serial.println(dht.computeHeatIndex(dht.toFahrenheit(temperature), humidity, true), 1);
  delay(2000);
}
#else
void getTempHumidity() {
  // Wait at least 2 seconds seconds between measurements.
  // if the difference between the current time and last time you read
  // the sensor is bigger than the interval you set, read the sensor
  // Works better than delay for things happening elsewhere also
  unsigned long currentMillis = millis();

  if(currentMillis - previousMillis >= interval) {
    // save the last time you read the sensor
    previousMillis = currentMillis;
 
    // Reading temperature for humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
    h = dht.readHumidity(); // Read humidity (percent)
    t = dht.readTemperature(); // Read temperature as
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }
    Serial.print("humidity : ");
    Serial.print(h, 1);
    Serial.print("\t\t");
    Serial.print("Temperature : ");
    Serial.println(t, 1);
  }
}
#endif
