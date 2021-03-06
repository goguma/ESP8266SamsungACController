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

#include "my_secret.h"

/*
 * Wifi Stuff
 */
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = MY_SECRET_SSID;
char pass[] = MY_SECRET_PASSWORD;

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
#define AC_FAN_ONLY_TEMP_BY_LIM 27
#define AC_RE_COOL_TEMP_BY_LIM 27.5
#define AC_WORKING_DELAY 5000

/*
 * thingspeak stuff
 */
WiFiClient client;
String thingSpeakApiKey = MY_SECRET_THING_SPEAK_API_KEY;
const char* server = "api.thingspeak.com";

/*
 * Blynk stuff
 */
// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
#define DEFAULT_AC_TEMPERATURE 28
#define DEFAULT_AC_TIMER_DURATION 60
char auth[] = MY_SECRET_AUTH_KEY;

BlynkTimer timer;
int acTemperature = DEFAULT_AC_TEMPERATURE;                   //celsius
int acTimerDuration = DEFAULT_AC_TIMER_DURATION;                 //min
int acManualPower = 1;
int acAutomationEnabled = 1;
//unsigned long timerInterval = 60000L; //60sec
unsigned long timerInterval = 120000L; //120sec, 2min
int calledcnt = 0;

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

BLYNK_WRITE(V2)
{
	acManualPower = param.asInt();
	Serial.print("V2 (Manual Power )value is: ");
	Serial.println(acManualPower);

	if (acManualPower == 1)
	{
		sendACPowerState(1);
		controlAC(true, false, AC_DEFAULT_TEMP_BY_LIM);
		//controlAC(true, true, AC_DEFAULT_TEMP_BY_LIM);
	}
	else
	{
		sendACPowerState(0);
		calledcnt = 0;
		controlAC(false, false, 0);
	}
}

BLYNK_WRITE(V3)
{
	acAutomationEnabled = param.asInt();
	Serial.print("V3 (acAutomationEnabled ) value is: ");
	Serial.println(acAutomationEnabled);
}

/*
 * Setup stuff
 */
void setup()
{
	Serial.begin(115200);
	delay(200);

	acSetup();

	thingSpeakSetup();

	dht.begin();

	Blynk.begin(auth, ssid, pass);
	// You can also specify server:
	//Blynk.begin(auth, ssid, pass, "blynk-cloud.com", 80);
	//Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

	// Setup a function to be called every timerInterval
	timer.setInterval(timerInterval, timerCallback);

	// Initialize Blynk GUI
	getTempHumidity();
	sendSensor(); // V5, V6
	Blynk.virtualWrite(V7, -1); //AC Power Status
	Blynk.virtualWrite(V2, 0); //Power On/Off Button
	Blynk.virtualWrite(V0, acTemperature); // AC Action Temp
	Blynk.virtualWrite(V1, acTimerDuration); //AC Timer Duration
	Blynk.virtualWrite(V3, 1); // AC acAutomationEnabled is enabled
	sendDataToThingSpeak(t, h, -1, -1);
}

void loop()
{
	Blynk.run();
	timer.run();
}

void timerCallback()
{
	if (acAutomationEnabled == 0)
		return;

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
		sendACPowerState(0);
		sendDataToThingSpeak(t, h, 0, 0);
		calledcnt = 0;
		controlAC(false, false, 0);
	}
	else if (ac.getPower() == false && t >= (float)acTemperature)
	{
		sendACPowerState(1);
		sendDataToThingSpeak(t, h, 1, 2);
		controlAC(true, false, AC_DEFAULT_TEMP_BY_LIM);
	}
	else if (ac.getPower() == true && t <= (float)AC_FAN_ONLY_TEMP_BY_LIM)
	{
		sendACPowerState(1);
		sendDataToThingSpeak(t, h, 1, 1);
		controlAC(true, true, AC_DEFAULT_TEMP_BY_LIM);
		calledcnt++;
	}
	else if (ac.getPower() == true && t >= (float)AC_RE_COOL_TEMP_BY_LIM)
	{
		sendACPowerState(1);
		sendDataToThingSpeak(t, h, 1, 2);
		controlAC(true, false, AC_DEFAULT_TEMP_BY_LIM);
		calledcnt++;
	}
	else if (ac.getPower() == true )
	{
		sendDataToThingSpeak(t, h, 1, 2);
		calledcnt++;
	}
	else
	{
		Serial.println("temperature is good! I don't need A/C");
		sendACPowerState(0);
		sendDataToThingSpeak(t, h, 0, 0);
	}

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

void controlAC(bool power, bool fan_only, int temperature)
{
	if (power == false)
	{
		// Turn the A/C unit off.
		Serial.println("Turn off the A/C ...");
		ac.off();
		ac.send();
		printState();
		delay(AC_WORKING_DELAY); // default is 5 sec
	}
	else
	{
		// Turn the A/C unit on
		Serial.println("Turn on the A/C ...");
		ac.on();
		ac.send();
		printState();
		delay(AC_WORKING_DELAY); // default is 5 sec
		ac.setFan(kSamsungAcFanLow);
		if (fan_only == true)
			ac.setMode(kSamsungAcFan);
		else
			ac.setMode(kSamsungAcCool);
		ac.setTemp(temperature);
		ac.setSwing(false);
		ac.send();
		printState();
		delay(AC_WORKING_DELAY); //default is 5 sec
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
	Blynk.virtualWrite(V2, power);
}
/*************************************************************************/

/*
 * thingspeak stuff
 */

void thingSpeakSetup()
{
	WiFi.begin(ssid, pass);

	Serial.println("WiFi connecting ");
	while (WiFi.status() != WL_CONNECTED) {
		delay(1000);
		Serial.print(".");
	}
	Serial.println("");
	Serial.println("WiFi connected");
}

void sendDataToThingSpeak(float temperature, float humidity, int acPowerState, int acMode)
{
	if (client.connect(server, 80))
	{
		//"184.106.153.149" or api.thingspeak.com
		String postStr = thingSpeakApiKey;
			postStr +="&field1=";
			postStr += String(temperature);
			postStr +="&field2=";
			postStr += String(humidity);
			postStr +="&field3=";
			postStr += String(acPowerState);
			postStr +="&field4=";
			postStr += String(acMode);

		client.print("POST /update HTTP/1.1\n");
		client.print("Host: api.thingspeak.com\n");
		client.print("Connection: close\n");
		client.print("X-THINGSPEAKAPIKEY: "+thingSpeakApiKey+"\n");
		client.print("Content-Type: application/x-www-form-urlencoded\n");
		client.print("Content-Length: ");
		client.print(postStr.length());
		client.print("\n\n");
		client.print(postStr);

		Serial.print("Temperature: ");
		Serial.print(temperature);
		Serial.println(" Celsius");

		Serial.print("Humidity: ");
		Serial.print(humidity);
		Serial.println(" %");

		Serial.print("acPowerState: ");
		Serial.print(acPowerState);
		Serial.println(" %");

		Serial.print("acMode: ");
		Serial.println(acMode);
	}
	else
	{
		Serial.println("Failed to connect to thingspeak");
	}

	client.stop();
}
