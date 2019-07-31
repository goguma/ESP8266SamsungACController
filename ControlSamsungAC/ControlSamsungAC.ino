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
#include <DHT.h>

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

/*
 * Wifi Stuff
 */
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "goguma";
char pass[] = "csw13241324";

/*
 * DHT stuff
 */
#define DHTPIN D1
#define DHTTYPE DHT22 // DHT 22 Change this if you have a DHT11

DHT dht(DHTPIN, DHTTYPE);
float t, h;
unsigned long previousMillis = 0; // will store last temp was read
const long interval = 2000;       // interval at which to read sensor

/*
 * IR remote for SAMSUNG AC stuff
 */
const uint16_t kIrLed = 4; // ESP8266 GPIO pin to use. Recommended: 4 (D2).
IRSamsungAc ac(kIrLed);    // Set the GPIO used for sending messages.
#define AC_DEFAULT_TEMP_BY_LIM 27

/*
 * Blynk stuff
 */
// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
#define DEFAULT_AC_TEMPERATURE 28
#define DEFAULT_AC_TIMER_DURATION 30
char auth[] = "HjxenbGtzG8wjzzpA3BeTjNbcAgZl_Qh";
BlynkTimer timer;
int acTemperature = DEFAULT_AC_TEMPERATURE;                   //celsius
int acTimerDuration = DEFAULT_AC_TIMER_DURATION;                 //min
//unsigned long timerInterval = 60000L; //60sec
unsigned long timerInterval = 300000L; //300sec, 5min

BLYNK_WRITE(V0)
{
  acTemperature = param.asInt();
  Serial.print("V0 (temperatue )Slider value is: ");
  Serial.println(acTemperature);
}

BLYNK_WRITE(V1)
{
  acTimerDuration = param.asInt();
  Serial.print("V1 (timerDuration )Slider value is: ");
  Serial.println(acTimerDuration);
}

/*
 * Setup stuff
 */
void setup()
{
  Serial.begin(115200);
  delay(200);

  acSetup();

  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

  dht.begin();

  // Setup a function to be called every timerInterval
  timer.setInterval(timerInterval, timerCallback);
}

void loop()
{
  Blynk.run();
  timer.run();
}

void timerCallback()
{
  static int calledcnt = 0;
  int shouldcalled = (unsigned int)(acTimerDuration * 60) / (timerInterval / 1000);
  Serial.print("calledcnt : ");
  Serial.print(calledcnt);
  Serial.print(" shouldcalled : ");
  Serial.println(shouldcalled);

  getTempHumidity();
  sendSensor();

  /* TODO : Logic to control AC */
  if (ac.getPower() == true && shouldcalled <= calledcnt)
  {
    controlAC(false, 0);
    sendACPowerState(0);
    calledcnt = 0;
  }
  else if (ac.getPower() == false && t >= (float)acTemperature)
  {
    controlAC(true, AC_DEFAULT_TEMP_BY_LIM);
    sendACPowerState(1);
  }
  else
    calledcnt++;
}

/*
 * IR remote for SAMSUNG AC stuff
 */
void acSetup()
{
  ac.begin();

  // Set up what we want to send. See ir_Samsung.cpp for all the options.
  Serial.println("Default state of the remote.");
  printState();
  Serial.println("Setting initial state for A/C.");
  ac.off();
  ac.setFan(kSamsungAcFanLow);
  ac.setMode(kSamsungAcCool);
  ac.setTemp(27);
  ac.setSwing(false);
  printState();
}

void printState()
{
  // Display the settings.
  Serial.println("Samsung A/C remote is in the following state:");
  Serial.printf("  %s\n", ac.toString().c_str());
}

void controlAC(bool power, int temperature)
{
  if (power == false)
  {
    // Turn the A/C unit off.
    Serial.println("Turn off the A/C ...");
    ac.off();
    ac.send();
    printState();
    delay(15000); // wait 15 seconds
  }
  else
  {
    // Turn the A/C unit on
    Serial.println("Turn on the A/C ...");
    ac.on();
    ac.send();
    printState();
    delay(15000); // wait 15 seconds
    ac.setFan(kSamsungAcFanLow);
    ac.setMode(kSamsungAcCool);
    ac.setTemp(temperature);
    ac.setSwing(false);
    ac.send();
    printState();
    delay(15000); // wait 15 seconds
  }
  
#if 0
  // Turn the A/C unit on
  Serial.println("Turn on the A/C ...");
  ac.on();
  ac.send();
  printState();
  delay(15000); // wait 15 seconds
  // and set to cooling mode.
  Serial.println("Set the A/C mode to cooling ...");
  ac.setMode(kSamsungAcCool);
  ac.send();
  printState();
  delay(15000); // wait 15 seconds

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
  delay(15000); // wait 15 seconds
#endif
}
/*************************************************************************/

/*
 * DHT stuff
 */
void getTempHumidity()
{
  // Wait at least 2 seconds seconds between measurements.
  // if the difference between the current time and last time you read
  // the sensor is bigger than the interval you set, read the sensor
  // Works better than delay for things happening elsewhere also
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    // save the last time you read the sensor
    previousMillis = currentMillis;

    // Reading temperature for humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
    h = dht.readHumidity();    // Read humidity (percent)
    t = dht.readTemperature(); // Read temperature as
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t))
    {
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
/*************************************************************************/

/*
 * Blynk stuff
 * Timer callback
 */
void sendSensor()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V5, h);
  Blynk.virtualWrite(V6, t);
}

void sendACPowerState(int power)
{
  Blynk.virtualWrite(V7, power);
}