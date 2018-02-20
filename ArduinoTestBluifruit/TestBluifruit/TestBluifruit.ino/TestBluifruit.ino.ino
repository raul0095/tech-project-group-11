/*********************************************************************
  This is an example based on nRF51822 based Bluefruit LE modules

********************************************************************/

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

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
int number;
int temp;
int optimal;
void setup(void) // SEEEEEEEEEEEEEEEEEEEEEEEETTTTTTTTTTTTTTTTTTTTTTTTTUUUUUUUUUUUUUUUUUUUUUUUUUUUPPPPPPPPPPPPPPPPPPPPPPPPPPPP SETUP
{

  pinMode(LED, OUTPUT); // lED <<<<
  temp = 14;
  optimal = 22;
  while (!Serial);  // required for Flora & Micro
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

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
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
  bool ble_data_received = false;
  while ( ble.available() )
  {
    int c = ble.read();
    bt_input[i++] = c;
    number = c;
    //Serial.print((char)c);
    ble_data_received = true;
  }
  bt_input[i] = 0;
  //Serial.print("Whole command" + String(bt_input));

  // Parse BlueTooth command
  if (ble_data_received) {
    ble_data_received = false;
    if (bt_input[0] == 'g' || bt_input[0] == 'G') {
      if (bt_input[1] == 'h' || bt_input[1] == 'H') {
        // read humidity sensor here and send data back over bluetooth using ble.print()
      }
      else if (bt_input[1] == 't' || bt_input[1] == 'T') {
        // read temperature sensor here and send data back over bluetooth
      }
      else {
        Serial.println("Unknown get command!");
      }
    } else if (bt_input[0] == 's' || bt_input[0] == 'S') {
      Serial.println("Reached S");
      if (bt_input[1] == 'h' || bt_input[1] == 'H') {
        Serial.println("Reached H");
        // set humidity a value and send back over bluetooth
        String str = String(bt_input);
        Serial.println("Humidity: " + str);
        int idx = str.indexOf(' ');
        Serial.println("Index: " + String(idx));
        Serial.println("length: " + String(str.length()));
        Serial.println("substr: " + str.substring(idx+1, str.length()));
        int humidity = (str.substring(idx+1, str.length())).toInt();
        Serial.println("Humidity: " + String(humidity));
      } else if  (bt_input[1] == 't' || bt_input[1] == 'T') {
        // set temperature and send back over bluetooth

      } else if (bt_input[1] == 'l' || bt_input[1] == 'L') {
        // set the light and send over bluetooth
      }
      else {
        Serial.println("Unknown set command!");
      }
    } else {
      Serial.println("Unknown command!");
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
    for (int i = optimal - temp; i > 0; i--) {
      // make the LED blink until it reaches optimal
    }
  } // FOLOSESTE WHILE LOOP CA SA FACI O LOGICA ceva gen while nu e optimal fa blinking si
}

// while (temp !=



void LEDblink(int rate) {

}


// Commands needed
// set temperature 22
// get temperature
// set humidity 45
// get humidity
// set rgb FFFFFF


