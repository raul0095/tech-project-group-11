/*********************************************************************
  This is an example based on nRF51822 based Bluefruit LE modules

********************************************************************/
#include <Adafruit_NeoPixel.h> // LED RGB library 

#include "DHT.h" // humidity and temperature sensor
#define PIN 9 // rgb led
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, PIN, NEO_GRB + NEO_KHZ800);  // rgb led
#define DHTPIN 2     // what digital pin we're connected to
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif
  
#include <Servo.h>
Servo servoMain;

/*=========================================================================
       -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

uint8_t LED = 5; // LED PIN
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


int number;
int temp = 0;
int optimal = 0;
int light = 50;
bool rad = false;
void setup(void) // SEEEEEEEEEEEEEEEEEEEEEEEETTTTTTTTTTTTTTTTTTTTTTTTTUUUUUUUUUUUUUUUUUUUUUUUUUUUPPPPPPPPPPPPPPPPPPPPPPPPPPPP SETUP
{
servoMain.attach(10);
  
  Serial.begin(115200);
  dht.begin();

  strip.begin(); // RGB LED
  strip.show(); // Initialize all pixels to 'off'




  pinMode(LED, OUTPUT); // lED <<<<<<<<<<<<<<<<<<<
  while (!Serial)  // required for Flora & Micro
    delay(500);


  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();



  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  //Give module a new name
  ble.println("AT+GAPDEVNAME=group11"); // named LONE

  // Check response status
  ble.waitForOK();

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
}

void loop(void)
{

  float h = dht.readHumidity();          // humidity/temperature sensor code
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);         // might be unnecessary code, check and return to it






  // Check for user input
  char n, inputs[BUFSIZE + 1];
  //number = ble.read();
  char bt_input[50];

  if (Serial.available())
  {
    n = Serial.readBytes(inputs, BUFSIZE);
    inputs[n] = 0;
    // Send characters to Bluefruit
    Serial.print("Sending: ");
    Serial.println(inputs);

    // Send input data to host via Bluefruit
    ble.print(inputs);
  }

  if (ble.available()) {
    Serial.print("* "); Serial.print(ble.available()); Serial.println(F(" bytes available from BTLE"));
  }
  // Echo received data

  // Read BlueTooth commands
  int i = 0;
  while ( ble.available() )
  {
    int c = ble.read();
    bt_input[i++] = c;
    number = c;
    //Serial.print((char)c);
  }
  bt_input[i] = 0;
  Serial.print(bt_input);

  // Parse BlueTooth command <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  //ble.print(bt_input);
  if (bt_input[0] == 'g') { // Gets information and sends to Janis
    if (bt_input[1] == 'h') {
      ble.print("SH " + String(h)); // string (h) is the actual readings from the sensors
      //  Send humidity data back over bluetooth here...
    } else if (bt_input[1] == 't') {
      // Read temperature sensor here and send data back over bluetooth
      ble.print(" ST " + String(t));
    } else if (bt_input[1] == 'w') {     //working    //recently added to debug the A requests that don`t even reach arduino
      ble.print("ot 22");
      optimal = 22;
      delay(200);
      ble.print("oh 45");
      lig(0);
    }  else if (bt_input[1] == 's') { // Sleeping
      ble.print("ot 18");
      optimal = 18;
      delay(200);
      ble.print("oh 45");
      lig(100);
    } else if (bt_input[1] == 'b') {  // Baby in the room
      ble.print("ot 23");
      optimal = 23;
      delay(200);
      ble.print("oh 45");
      lig(50);
    } else if (bt_input[1] == 'e') { // Exercising
      ble.print("ot 19");
      optimal = 19;
      delay(200);
      ble.print("oh 50");
      lig(0);
    }
  } else if (bt_input[0] == 's') { //  Sets info, arduino must put new values
    if (bt_input[1] == 'h') {                             // humidity
      // set humidity a value and send back over bluetooth
      ble.print("SH " + String(bt_input).substring(3));
    } else if  (bt_input[1] == 't') {                     //temperature
      // set temperature and send back over bluetooth
      String strTemp = String(bt_input).substring(3);
      int tempNow = strTemp.toInt();
      ble.print("ST " + String(bt_input).substring(3));
      Serial.print("current temp is " + tempNow);
      temp = tempNow;

    } else if (bt_input[1] == 'l') {                     //light
      // set the light and send over bluetooth
      ble.print("SL");
      String nowL = String(bt_input).substring(3);
      light = nowL.toInt();
      lig(light);
    }
  }

  radiator();
  LEDon(rad);
}


void radiator() {

  if (temp < optimal) {  // checks if it's below or above
    LEDon(true);

    for (int i = optimal - temp; i > 0; i--) {
      temp++;
      ble.print("ST "  + String(temp));

      delay(2000);

    }
  } else if (temp > optimal) {
    LEDon(false);

    for (int i = temp - optimal; i >= 1; i--) {
      temp--;
      ble.print("ST " + String(temp));
      delay(2000);
      servoMain.write(95);
    }
    servoMain.write(0);
     LEDon(false);
  } 
}


void lig(int colors) {
  float r = 1.35;
  float g = 1.49;
  float b = 1.54;

  strip.setBrightness(20);
  strip.setPixelColor(0, 120 + r * colors, 209 - g * colors, 204 - b * colors);
  strip.show();
}


void LEDon(bool on) {
  if (on == true) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
}



// Commands needed
// set temperature 22
// get temperature
// set humidity 45
// get humidity
// set rgb FFFFFF


