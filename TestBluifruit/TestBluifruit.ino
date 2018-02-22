/*********************************************************************
  This is an example based on nRF51822 based Bluefruit LE modules

********************************************************************/
#include <Adafruit_NeoPixel.h> // LED RGB library 

#include "DHT.h" // humidity and temperature sensor
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
int temp;
int optimal;
void setup(void) // SEEEEEEEEEEEEEEEEEEEEEEEETTTTTTTTTTTTTTTTTTTTTTTTTUUUUUUUUUUUUUUUUUUUUUUUUUUUPPPPPPPPPPPPPPPPPPPPPPPPPPPP SETUP
{
  Serial.begin(115200);
  dht.begin();






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
    }
    else if (bt_input[1] == 't') {
      // Read temperature sensor here and send data back over bluetooth
      ble.print(" ST " + String(t));
    }

    else if (bt_input[2] == 'w') {        //recently added to debug the A requests that don`t even reach arduino
      
      ble.print("ot 22");
      delay(2000);
      ble.print("oh 45");
    }
  }



  else if (bt_input[0] == 's') { //  Sets info, arduino must put new values
    if (bt_input[1] == 'h') {                             // humidity
      // set humidity a value and send back over bluetooth
      ble.print("SH " + String(bt_input).substring(3));



    } else if  (bt_input[1] == 't') {                     //temperature
      // set temperature and send back over bluetooth
      String strTemp = String(bt_input).substring(3);
      int tempNow = strTemp.toInt();
      ble.print("ST " + String(bt_input).substring(3));
      Serial.print("current temp is " + tempNow);

    } else if (bt_input[1] == 'l') {                     //light
      // set the light and send over bluetooth
      ble.print("SL");
    }


  } else if (bt_input[0] == 'a') {    // arduino receives the activity selected and sends back recommended parameters
    if (bt_input[1] == 'w') { // Working
      ble.print("ot 22");
      ble.print("oh 45");
      //optimal
    }

    else if (bt_input[1] == 's') { // Sleeping
      ble.print("ot 18");
      ble.print("oh ");
    }

    else if (bt_input[1] == 'b') {  // Baby in the room
      ble.print("ot ");
      ble.print("oh ");
    }

    else if (bt_input[1] == 'e') { // Exercising
      ble.print("ot ");
      ble.print("oh ");
    }
  }


  if (number == 49) {
    digitalWrite(LED, HIGH);
    delay(5000);
    digitalWrite(LED, LOW);
    delay(1000);
  }
  delay(1000);
}

void radiator() {

  if (temp < optimal) {  // checks if it's below or above
    //  blinkfast();
    digitalWrite(LED, HIGH);
    delay(300);
    digitalWrite(LED, LOW);
    delay(300);
    digitalWrite(LED, HIGH);
    delay(300);
    digitalWrite(LED, LOW);
    delay(300);


    for (int i = optimal; i > 0; i--) {
      // make the LED blink until it reaches optimal
      ble.print("ST "  );
    }
    //if ( i == 0 ) {
    digitalWrite(LED, LOW);
    //}
  }
}


void LEDblink(int rate) {

}


// Commands needed
// set temperature 22
// get temperature
// set humidity 45
// get humidity
// set rgb FFFFFF


