// TI Sensor Tag Master Logger
// 2014-04-17 by Sridhar Rajagopal <sridhar@upbeatlabs.com>

// Changelog:
//      2014-04-17 - Initial release

/* ============================================
Code is placed under the MIT license
Copyright (c) 2014 Sridhar Rajagopal

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <SoftwareSerial.h>
#include "BGLib.h"
#include <SD.h>
#include <Wire.h>
#include "DS1339.h"
#include "sensorTagUtil.h"
// #include <avr/wdt.h>

// BLE112 module connections:
// - BLE P0_4 -> Arduino Digital Pin 4 (BLE TX -> Arduino soft RX)
// - BLE P0_5 -> Arduino Digital Pin 5 (BLE RX -> Arduino soft TX)
//
// If using the *_hwake15 project firmware:
// - BLE P1_5 -> Arduino Digital Pin 3 (BLE host wake-up -> Arduino I/O 4)
//
// If useing the *_wake16 project firmware:
// - BLE P1_6 -> Arduino Digital Pin 6 (BLE module wake-up -> Arduino I/O 5)
//
// If using project with flow control enabled on the BLE:
// - BLE P0_2 -> GND (CTS tied to ground to bypass flow control)

// NOTE: this demo REQUIRES the BLE112 be programmed with the UART connected
// to the "api" endpoint in hardware.xml, have "mode" set to "packet", and be
// configured for 38400 baud, 8/N/1. This may change in the future, but be
// aware. The BLE SDK archive contains an /examples/uartdemo project which is
// a good starting point for this communication, though some changes are
// required to enable packet mode and change the baud rate. The BGLib
// repository also includes a project you can use for this in the folder
// /BLEFirmware/BGLib_U1A1P_38400_noflow_wake16_hwake15.
/*
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 10

*/


// Uncomment these if you want print statements and/or debug print statements
// #define DEBUGPRINT
// #define PRINT_ENABLED

#ifdef PRINT_ENABLED
  #define PRINTLN(x)  Serial.println (x)
  #define PRINT(x)  Serial.print (x)
  #define PRINTLN2(x, y) Serial.println(x, y)
  #define PRINT2(x, y) Serial.print(x, y)
  #define WRITE(x) Serial.write (x)

#else
  #define PRINTLN(x)
  #define PRINT(x) 
  #define PRINTLN2(x, y) 
  #define PRINT2(x, y) 
  #define WRITE(x) 
#endif

#ifdef DEBUGPRINT
  #define DEBUG_PRINTLN(x)  Serial.println (x)
  #define DEBUG_PRINT(x)  Serial.print (x)
  #define DEBUG_PRINTLN2(x, y) Serial.println(x, y)
  #define DEBUG_PRINT2(x, y) Serial.print(x, y)
 
#else
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN2(x, y)
  #define DEBUG_PRINT2(x, y)
#endif

#define RTC_INT_PIN 2
#define RTC_INT_NUM 0

DS1339 RTC = DS1339(RTC_INT_PIN, RTC_INT_NUM);

#define BLE_P04 4
#define BLE_P05 5

SoftwareSerial bleSerialPort(BLE_P04, BLE_P05); // RX, TX
BGLib ble112((HardwareSerial *)&bleSerialPort, 0, 1); // packet mode enabled  

// uint8_t isScanActive = 0;


// Perhaps use Digital 3 and/or A0 for Button1 and Shift register
// #define BUTTON1_PIN 
#define BUTTON2_PIN A1
#define BUTTON3_PIN A2
#define BUTTON4_PIN A3

#define CHIPSELECT_PIN 10

// Leds are on Shift register as follows
// 0 = LED8, 1 = LED7, 2 = LED6, 3=LED5, 4=LED4, 5=LED3, 6=LED2, 7=LED1 
#define SR_LED_TEMP 2 
#define SR_LED_ACCL 3 
#define SR_LED_HUMIDITY 4 
#define SR_LED_COMPASS 5 
#define SR_LED_BAROMETER 6 
#define SR_LED_GYROSCOPE 7 
#define SR_LED_PIN 0         // Arduino Uno LED
#define SR_LED_STATUS_PIN 1   // Connect an LED to this pin to get status information

#define BLE_HOST_WAKEUP_PIN 3 // Not used currently
#define BLE_WAKEUP_PIN 6    // Assuming digital pin 6 is connected to BLE wake-up pin (P1_6 in my firmware)
#define BLE_RESET_PIN 7

#define LATCH_PIN 9
#define CLOCK_PIN A0
#define DATA_PIN 8

#define BLE_BAUD_RATE 38400 
#define HSERIAL_BAUD_RATE 115200

// BLE state machine definitions
#define BLE_STATE_STANDBY                0
#define BLE_STATE_START_SCANNING         1
#define BLE_STATE_SCANNING               2
#define BLE_STATE_CONNECTING             3
#define BLE_STATE_CONNECTED              4
#define BLE_STATE_CONNECTED_MEASUREMENTS  5
#define BLE_STATE_CONNECTED_MEASUREMENTS_DONE 6

byte leds = 0; // Used with the shift register

// BLE state/link status tracker
uint8_t ble_state = BLE_STATE_STANDBY;
//uint8_t ble_encrypted = 0;  // 0 = not encrypted, otherwise = encrypted
//uint8_t ble_bonding = 0xFF; // 0xFF = no bonding, otherwise = bonding handle

uint8_t ble_state_index = 0; // 0 = temp, 1 = accl, 2 = humidity, 3 = magm, 4 = barometer, 5 = gyrascope


// Refer to http://processors.wiki.ti.com/images/a/a8/BLE_SensorTag_GATT_Server.pdf for handles
// peripheral_list = []

// ATT Handles in bglib are defined as uint16
// Sensor Tag handles all fit in uint8, so using uint8 to save space
// Change it if your handles are bigger

uint8 connection_handle = 0;
//uint16 att_handle_temp_start = 35;
//uint16 att_handle_temp_end = 45;
const uint8 att_handle_temp_measurement = 37;
//uint16 att_handle_temp_measurement_ccc = 38;
//uint16 att_handle_temp_measurement_switch = 41;
//uint16 att_handle_temp_user_desc = 39;
Measurement temp_measurement = { 41, 38, false};

//uint16 att_handle_accl_start = 46;
//uint16 att_handle_accl_end = 56;
const uint8 att_handle_accl_measurement = 48;
//uint16 att_handle_accl_measurement_ccc = 49;
//uint16 att_handle_accl_measurement_switch = 52;
//uint16 att_handle_accl_sample_rate = 55;
//uint16 att_handle_accl_user_desc = 50;
Measurement accl_measurement = {52, 49, false};

//uint16 att_handle_humidity_start = 57;
//uint16 att_handle_humidity_end = 67;
const uint8 att_handle_humidity_measurement = 59;
//uint16 att_handle_humidity_measurement_ccc = 60;
//uint16 att_handle_humidity_measurement_switch = 63;
//uint16 att_handle_humidity_user_desc = 61;
Measurement humidity_measurement = {63, 60, false};

//uint16 att_handle_magm_start = 68;
//uint16 att_handle_magm_end = 78;
const uint8 att_handle_magm_measurement = 70;
//uint16 att_handle_magm_measurement_ccc = 71;
//uint16 att_handle_magm_measurement_switch = 74;
//uint16 att_handle_magm_sample_rate = 77;
//uint16 att_handle_magm_user_desc = 72;
Measurement magm_measurement = {74, 71, false};

//uint16 att_handle_barometer_start = 79;
//uint16 att_handle_barometer_end = 93;
const uint8 att_handle_barometer_measurement = 81;
//uint16 att_handle_barometer_measurement_ccc = 82;
//uint16 att_handle_barometer_measurement_switch = 85;
//uint16 att_handle_barometer_sample_rate = 88; // If 0x44 is correct see hmmm below
//uint16 att_handle_barometer_user_desc = 83;
Measurement barometer_measurement = {85, 82, false};

//uint16 att_handle_gyroscope_start = 94;
//uint16 att_handle_gyroscope_end = 104;
const uint8 att_handle_gyroscope_measurement = 96;
//uint16 att_handle_gyroscope_measurement_ccc = 97;
//uint16 att_handle_gyroscope_measurement_switch = 100;
//uint16 att_handle_gyroscope_user_desc = 98;
Measurement gyroscope_measurement = {100, 97, false};

//uint16 att_handle_simplekey_start = 105;
//uint16 att_handle_simplekey_end = 109;
//uint16 att_handle_simplekey_measurement = 0;
//uint16 att_handle_simplekey_measurement_ccc = 0;
//uint16 att_handle_simplekey_measurement_switch = 0;
//uint16 att_handle_simplekey_user_desc = 0;


//const uint8_t uuid_service[] = {0x28, 0x00}; // 0x2800
//const uint8_t uuid_service_reversed[] = {0x00, 0x28}; // 0x2800
//
//const uint8_t uuid_client_characteristic_configuration[] = {0x29, 0x02}; // 0x2902
//
//const uint8_t uuid_characteristic_user_desc[] = {0x29, 0x01} ; // 0x2901
//
//const uint8_t uuid_sensor_common[] = {0xf0, 0x00, 0xaa};
//
//const uint8_t uuid_temp_service = 0x00; // 0xF000AA00))
//const uint8_t uuid_temp_characteristic = 0x01; // 0xF000AA01
//const uint8_t uuid_temp_characteristic_switch = 0x02; // 0xF000AA02
//
//const uint8_t uuid_accl_service = 0x10; // 0xF000AA10))
//const uint8_t uuid_accl_characteristic = 0x11; // 0xF000AA11
//const uint8_t uuid_accl_characteristic_switch = 0x12; // 0xF000AA12
////const uint8_t uuid_accl_sample_rate = 0x13; // 0xF000AA13
//
//const uint8_t uuid_humidity_service = 0x20; // 0xF000AA20))
//const uint8_t uuid_humidity_characteristic = 0x21; // 0xF000AA21
//const uint8_t uuid_humidity_characteristic_switch = 0x22; // 0xF000AA22
//
//const uint8_t uuid_magm_service = 0x30; // 0xF000AA30))
//const uint8_t uuid_magm_characteristic = 0x31; // 0xF000AA31
//const uint8_t uuid_magm_characteristic_switch = 0x32; // 0xF000AA32
////const uint8_t uuid_magm_sample_rate = 0x33; // 0xF000AA33
//
//const uint8_t uuid_barometer_service = 0x40; // 0xF000AA40))
//const uint8_t uuid_barometer_characteristic = 0x41; // 0xF000AA41
//const uint8_t uuid_barometer_characteristic_switch = 0x42; // 0xF000AA42
////const uint8_t uuid_barometer_sample_rate = 0x43; // 0xF000AA43 hmmm 0x44??????
//
//const uint8_t uuid_gyroscope_service = 0x50; // 0xF000AA50))
//const uint8_t uuid_gyroscope_characteristic = 0x51; // 0xF000AA51
//const uint8_t uuid_gyroscope_characteristic_switch = 0x52; // 0xF000AA52
//
//const uint8_t uuid_simplekey_service[] = {0xff, 0xe0}; // 0xFFE0))
//const uint8_t uuid_simplekey_characteristic[] = {0xff, 0xe1}; // 0xFFE1


int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// uint32_t startScanMillis;
bool sdInitialized = false;

const uint8 debounceDelay = 50;
uint32_t lastDebounceTime[] = {0, 0, 0, 0};

uint8 buttonState[] = {HIGH, HIGH, HIGH, HIGH};

uint8 lastButtonState[] = {HIGH, HIGH, HIGH, HIGH};

// Each button toggles a couple of sensors
// For example, button 1 toggles temp and accl 0, 1, 2, 3 (none, temp, accl, temp+accl)
uint8 buttonToggleState[] = {0, 0, 0, 0}; 

bool buttonToggle (int buttonPin, int index) {
  bool toggled = false;
  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPin);

  // check to see if you just pressed the button 
  // (i.e. the input went from LOW to HIGH),  and you've waited 
  // long enough since the last press to ignore any noise:  

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState[index]) {
    // reset the debouncing timer
    lastDebounceTime[index] = millis();
  } 
  
  if ((millis() - lastDebounceTime[index]) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState[index]) {
      buttonState[index] = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState[index] == HIGH) {
        toggled = true;
      }
    }
  }
  lastButtonState[index] = reading;
  return toggled;
}


void setup() {
    // wdt_disable();
  
    // open Arduino USB serial (and wait, if we're using Leonardo)
//    Serial.begin(HSERIAL_BAUD_RATE);
//    while (!Serial);
//    Serial.println(F("SensorTag Master"));
//    PRINTLN(F("SensorTag Master"));
    
    RTC.start(); // ensure RTC oscillator is running, if not already
  
    if(!RTC.time_is_set()) // set a time, if none set already...
    {
      PRINTLN(F("Clock not set"));
      // set_time();
    }
  
    PRINTLN(F("Init SD card..."));
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(CHIPSELECT_PIN, OUTPUT);
  
    // see if the card is present and can be initialized:
    if (!SD.begin(CHIPSELECT_PIN)) {
      PRINTLN(F("SD card error**"));
      sdInitialized = false;
    } else {
      sdInitialized = true;
    }
    
    // Initialize shift register
    pinMode(LATCH_PIN, OUTPUT);
    pinMode(DATA_PIN, OUTPUT);  
    pinMode(CLOCK_PIN, OUTPUT);
    
    // initialize buttons
    // pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
    pinMode(BUTTON3_PIN, INPUT_PULLUP);
    pinMode(BUTTON4_PIN, INPUT_PULLUP);
    
    
    // initialize LEDs
    leds = 0;
    updateShiftRegister(leds);
    
    // initialize BLE reset pin (active-low)
    // Cycle to reset BLE module
    pinMode(BLE_RESET_PIN, OUTPUT);
    digitalWrite(BLE_RESET_PIN, HIGH);
    delay(5);
    digitalWrite(BLE_RESET_PIN, LOW);
    delay(5); // wait 5ms
    digitalWrite(BLE_RESET_PIN, HIGH);


    // initialize BLE wake-up pin to allow (not force) sleep mode
    pinMode(BLE_WAKEUP_PIN, OUTPUT);
    digitalWrite(BLE_WAKEUP_PIN, LOW);

    // set up internal status handlers
    // (these are technically optional)
    // ble112.onBusy = onBusy;
    // ble112.onIdle = onIdle;
    // ble112.onTimeout = onTimeout;
    
    // ONLY enable these if you are using the <wakeup_pin> parameter in your firmware's hardware.xml file
    ble112.onBeforeTXCommand = onBeforeTXCommand;
    ble112.onTXCommandComplete = onTXCommandComplete;

    // set up BGLib response handlers (called almost immediately after sending commands)
    // (these are also technicaly optional)
    // ble112.ble_rsp_system_hello = my_rsp_system_hello;
    // ble112.ble_rsp_gap_set_scan_parameters = my_rsp_gap_set_scan_parameters;
    // ble112.ble_rsp_gap_discover = my_rsp_gap_discover;
    // ble112.ble_rsp_gap_end_procedure = my_rsp_gap_end_procedure;
    // ble112.ble_rsp_attclient_attribute_write = my_rsp_attclient_attribute_write;
    // ble112.ble_rsp_connection_disconnect = my_rsp_connection_disconnect;

    // set up BGLib event handlers (called at unknown times)
    // ble112.ble_evt_system_boot = my_evt_system_boot;
    ble112.ble_evt_gap_scan_response = my_evt_gap_scan_response;
    
    ble112.ble_evt_connection_status = my_ble_evt_connection_status;
    // ble112.ble_evt_attclient_group_found = my_ble_evt_attclient_group_found;
    // ble112.ble_evt_attclient_find_information_found = my_ble_evt_attclient_find_information_found;
    ble112.ble_evt_attclient_procedure_completed = my_ble_evt_attclient_procedure_completed;
    ble112.ble_evt_attclient_attribute_value = my_ble_evt_attclient_attribute_value;
    ble112.ble_evt_connection_disconnected = my_ble_evt_connection_disconnected;

    // set the data rate for the SoftwareSerial port
    bleSerialPort.begin(BLE_BAUD_RATE);
    
    // Turn on the watch dog - 8seconds for now to be conservative
    // wdt_enable(WDTO_8S);
}


void printHelp() {
    DEBUG_PRINTLN(F("Menu:"));
    DEBUG_PRINTLN(F("0) Reset"));
    DEBUG_PRINTLN(F("1) Hello"));
    DEBUG_PRINTLN(F("2) Memory left"));
    DEBUG_PRINTLN(F("c) Connect"));
    DEBUG_PRINTLN(F("t) Tog T"));
    DEBUG_PRINTLN(F("a) Tog Acc"));
    DEBUG_PRINTLN(F("h) Tog H"));
    DEBUG_PRINTLN(F("m) Tog Comp"));
    DEBUG_PRINTLN(F("g) Tog Gyro"));
    DEBUG_PRINTLN(F("b) Tog Bar"));    
    DEBUG_PRINTLN(F("s) Tog SKS"));
    DEBUG_PRINTLN(F("d) Disconnect"));
    DEBUG_PRINTLN(F("?) Help"));  
}

void loop() {
    printHelp();
    DEBUG_PRINTLN(F("Command?"));
    connect();
    while (1) {
        // keep polling for new data from BLE
        ble112.checkActivity();
        
//        if (buttonToggle(BUTTON1_PIN, 0)) {
//          buttonToggleState[0] = (buttonToggleState[0]+1) % 2;
//          Serial.println(buttonToggleState[0]);
//          if (buttonToggleState[0] == 0) {
//            disconnect();  
//          } else {
//            connect();
//          }
//          
//        }
        if (buttonToggle(BUTTON2_PIN, 1)) {
          buttonToggleState[1] = (buttonToggleState[1]+1) % 4;
          PRINTLN(buttonToggleState[1]);
          switch(buttonToggleState[1]) {
            case 0:
              bitClear(leds, SR_LED_ACCL);
              turnAccl(false);
              break;
            case 1:
              turnTemp(true);
              bitSet(leds, SR_LED_TEMP);
              break;
            case 2:
              turnAccl(true);
              bitSet(leds, SR_LED_ACCL);
              break;
            case 3:
              bitClear(leds, SR_LED_TEMP);
              turnTemp(false);
              break;
            default:
              break;  
          }
          updateShiftRegister(leds);            

        }
        if (buttonToggle(BUTTON3_PIN, 2)) {
          buttonToggleState[2] = (buttonToggleState[2]+1) % 4;
          PRINTLN(buttonToggleState[2]);
          switch(buttonToggleState[2]) {
            case 0:
              bitClear(leds, SR_LED_COMPASS);
              turnCompass(false);
              break;
            case 1:
              bitSet(leds, SR_LED_HUMIDITY);
              turnHumidity(true);
              break;
            case 2:
              bitSet(leds, SR_LED_COMPASS);
              turnCompass(true);
              break;
            case 3:
              bitClear(leds, SR_LED_HUMIDITY);
              turnHumidity(false);
              break;
            default:
              break;  
          }     
          updateShiftRegister(leds);          
          
        }
        if (buttonToggle(BUTTON4_PIN, 3)) {
          buttonToggleState[3] = (buttonToggleState[3]+1) % 4;
          PRINTLN(buttonToggleState[3]);
          switch(buttonToggleState[3]) {
            case 0:
              bitClear(leds, SR_LED_GYROSCOPE);
              turnGyroscope(false);
              break;
            case 1:
              bitSet(leds, SR_LED_BAROMETER);
              turnBarometer(true);
              break;
            case 2:
              bitSet(leds, SR_LED_GYROSCOPE);
              turnGyroscope(true);
              break;
            case 3:
              bitClear(leds, SR_LED_BAROMETER);
              turnBarometer(false);
              break;
            default:
              break;  
          }         
          updateShiftRegister(leds);                    
        }
        
        
        // blink Arduino LED based on state:
        //  - solid = STANDBY
        //  - 1 pulse per second = START_SCANNING
        //  - 2 pulses per second = SCANNING (some advertiser is out there)
        //  - 3 pulses per second = CONNECTED 
        uint16_t slice = millis() % 1000;
        if (ble_state == BLE_STATE_STANDBY) {
            bitSet(leds, SR_LED_STATUS_PIN);
            // digitalWrite(LED_STATUS_PIN, HIGH);
        } else if (ble_state == BLE_STATE_START_SCANNING) {
            bitWrite(leds, SR_LED_STATUS_PIN, slice < 100);
            // digitalWrite(LED_STATUS_PIN, slice < 100);
        } else if (ble_state >= BLE_STATE_SCANNING && ble_state < BLE_STATE_CONNECTED) {
            bitWrite(leds, SR_LED_STATUS_PIN, slice < 100 || (slice > 200 && slice < 300));
            // digitalWrite(LED_STATUS_PIN, slice < 100 || (slice > 200 && slice < 300));
        } else if (ble_state >= BLE_STATE_CONNECTED) {
            bitWrite(leds, SR_LED_STATUS_PIN, slice < 100 || (slice > 200 && slice < 300) || (slice > 400 && slice < 500));
            // digitalWrite(LED_STATUS_PIN, slice < 100 || (slice > 200 && slice < 300) || (slice > 400 && slice < 500));
        }
        updateShiftRegister(leds);   

        
        // Manage automatic state transitions based on timeouts here
//        if (ble_state == BLE_STATE_START_SCANNING || ble_state == BLE_STATE_SCANNING) {
//            uint32_t timeElapsed = millis() - startScanMillis;
//            if (timeElapsed > 5000) {
//                DEBUG_PRINT(F("No response in:")); DEBUG_PRINT(timeElapsed/1000); DEBUG_PRINTLN(F(" secs"));
//                stopScanning();  
//                
//            }
//        }
        
        // check for input from the user
//        if (Serial.available()) {
//            uint8_t ch = Serial.read();
//            uint8_t status;
//            switch(ch) {
//                case '0':
//                    // Reset BLE112 module
//                    ble_state = BLE_STATE_STANDBY;
//                    DEBUG_PRINTLN(F("-->\tsys_reset:"));
//                    ble112.ble_cmd_system_reset(0);
//                    while ((status = ble112.checkActivity(1000)));
//                    // system_reset doesn't have a response, but this BGLib
//                    // implementation allows the system_boot event specially to
//                    // set the "busy" flag to false for this particular case
//                    break;
//                case '1':
//                    // Say hello to the BLE112 and wait for response
//                    Serial.println(F("-->\tsys_hello"));
//                    ble112.ble_cmd_system_hello();
//                    while ((status = ble112.checkActivity(1000)));
//                    // response should come back within milliseconds
//                    break;
//                case 'c':
//                    connect();
//                    break;
//                case 't':
//                    toggleTemperature();
//                    break;
//                case 'a':
//                    toggleAccelerometer();
//                    break;
//                case 'h':
//                    toggleHumiditySensor();
//                    break;
//                case 'm':
//                    toggleCompass();
//                    break;
//                case 'g':
//                    toggleGyroscope();
//                    break;
//                case 'b':
//                    toggleBarometricPressure();
//                    break;
//                case 's':
//                    toggleSimpleKeyService();
//                    break;
//                case 'd':
//                    disconnect();
//                    break; 
//                case '2':
//                    PRINT(F("Free memory: "));
//                    PRINTLN(freeRam());
//                    break; 
//                case '?':
//                    printHelp();
//                    break;
//                default:
//                    break;                 
//            }
//        }
        // Reset watchdog
        // wdt_reset();
    }
}

int connect() {
    if (ble_state > BLE_STATE_STANDBY && ble_state < BLE_STATE_CONNECTED) {
        PRINTLN(F("Op in progress! Try later"));
    } else if (ble_state > BLE_STATE_STANDBY && ble_state >= BLE_STATE_CONNECTED) {
        PRINTLN(F("Already connected!"));      
    } else {
        uint8_t status;
        ble_state = BLE_STATE_START_SCANNING;
        // startScanMillis = millis();
        ble112.ble_cmd_gap_set_scan_parameters(0xC8, 0xC8, 1);
        while ((status = ble112.checkActivity(1000)));
        // response should come back within milliseconds
        ble112.ble_cmd_gap_discover(BGLIB_GAP_DISCOVER_GENERIC);
        while ((status = ble112.checkActivity(1000)));
        // response should come back within milliseconds
        // scan response events may happen at any time after this        
    }

}

void stopScanning() {
    PRINTLN(F("stopScanning"));
    if (ble_state == BLE_STATE_SCANNING | ble_state == BLE_STATE_START_SCANNING) {
        uint8_t status;
        DEBUG_PRINTLN(F("-->\tend_procedure"));
        ble112.ble_cmd_gap_end_procedure();
        while ((status = ble112.checkActivity(1000)));
        // response should come back within milliseconds
        ble_state = BLE_STATE_STANDBY;    
        buttonToggleState[0] = 0;
    }  
}

void turnOnMeasurement(Measurement myMeasurement) {
    if (ble_state < BLE_STATE_CONNECTED) {
      DEBUG_PRINTLN(F("measure: Not ready."));
      return;
    }
    // uint8 data[1] = {0x01}; 
    uint8 data[1] = { myMeasurement.measurementOn};
    uint8_t status;
    ble112.ble_cmd_attclient_attribute_write(connection_handle, myMeasurement.measurement_switch, 1, (const uint8*)&data);
    while ((status = ble112.checkActivity(1000))); 
}

void turnOnMeasurementCCC(Measurement myMeasurement) {
    if (ble_state < BLE_STATE_CONNECTED_MEASUREMENTS) {
      DEBUG_PRINTLN(F("measureCCC: Not ready."));
      return;
    }
    uint8_t status;
    uint8 data2[2] = {myMeasurement.measurementOn, 0x00};
    ble112.ble_cmd_attclient_attribute_write(connection_handle, myMeasurement.measurement_ccc, 2, (const uint8*)&data2);
    while ((status = ble112.checkActivity(1000)));
}

//void toggleTemperature() {
//    temp_measurement.measurementOn = !temp_measurement.measurementOn;
//    turnOnMeasurement(temp_measurement);  
//}
//
//void toggleAccelerometer() {
//    accl_measurement.measurementOn = !accl_measurement.measurementOn;
//    turnOnMeasurement(accl_measurement);  
//  
//}
//
//void toggleCompass() {
//    magm_measurement.measurementOn = !magm_measurement.measurementOn;
//    turnOnMeasurement(magm_measurement);  
//  
//}
//
//void toggleGyroscope() {
//    gyroscope_measurement.measurementOn = !gyroscope_measurement.measurementOn;
//    turnOnMeasurement(gyroscope_measurement);  
//  
//}
//
//void toggleBarometricPressure() {
//    barometer_measurement.measurementOn = !barometer_measurement.measurementOn;
//    turnOnMeasurement(barometer_measurement);  
//  
//}
//
//void toggleHumiditySensor() {
//    humidity_measurement.measurementOn = !humidity_measurement.measurementOn;
//    turnOnMeasurement(humidity_measurement);  
//  
//}

void turnTemp(bool on) {
   temp_measurement.measurementOn = on;
   turnOnMeasurement(temp_measurement);
}


void turnAccl(bool on) {
   accl_measurement.measurementOn = on;
   turnOnMeasurement(accl_measurement);
}

void turnCompass(bool on) {
   magm_measurement.measurementOn = on;
   turnOnMeasurement(magm_measurement);
}

void turnGyroscope(bool on) {
   gyroscope_measurement.measurementOn = on;
   turnOnMeasurement(gyroscope_measurement);
}

void turnBarometer(bool on) {
   barometer_measurement.measurementOn = on;
   turnOnMeasurement(barometer_measurement);
}

void turnHumidity(bool on) {
   humidity_measurement.measurementOn = on;
   turnOnMeasurement(humidity_measurement);
}


//void toggleSimpleKeyService() {
//
//}

void disconnect() {
    if (ble_state >= BLE_STATE_CONNECTED) {
        PRINTLN(F("Disconnecting..."));
        ble112.ble_cmd_connection_disconnect(connection_handle);
        uint8_t status;
        while ((status = ble112.checkActivity(1000)));
    } else {
        PRINTLN(F("Not connected!"));
    }  
}

// ================================================================
// INTERNAL BGLIB CLASS CALLBACK FUNCTIONS
// ================================================================

void onBusy() {
    // turn LED on when we're busy
    bitSet(leds, SR_LED_PIN);
    updateShiftRegister(leds);
    // digitalWrite(LED_PIN, HIGH);
}

void onIdle() {
    // turn LED off when we're no longer busy
    bitClear(leds, SR_LED_PIN);
    updateShiftRegister(leds);
    // digitalWrite(LED_PIN, LOW);
}

void onTimeout() {
    DEBUG_PRINTLN(F("!!!\tTimeout occurred!"));
}

void onBeforeTXCommand() {
    // wake module up (assuming here that digital pin 5 is connected to the BLE wake-up pin)
    digitalWrite(BLE_WAKEUP_PIN, HIGH);

    // wait for "hardware_io_port_status" event to come through, and parse it (and otherwise ignore it)
    uint8_t *last;
    // Email exchange with Jeff Rowberg
    while (1) {
        ble112.checkActivity();
        last = ble112.getLastEvent();
        if (last[0] == 0x07 && last[1] == 0x00) break;
    }
    
    // give a bit of a gap between parsing the wake-up event and allowing the command to go out
    delayMicroseconds(1000);
}

void onTXCommandComplete() {
    // allow module to return to sleep (assuming here that digital pin 5 is connected to the BLE wake-up pin)
    digitalWrite(BLE_WAKEUP_PIN, LOW);
}

// ================================================================
// USER-DEFINED BGLIB RESPONSE CALLBACKS
// ================================================================

void my_rsp_system_hello(const ble_msg_system_hello_rsp_t *msg) {
    PRINTLN(F("<--\tsys_hello"));
}

void my_rsp_gap_set_scan_parameters(const ble_msg_gap_set_scan_parameters_rsp_t *msg) {
    DEBUG_PRINT(F("<--\tset_scan_parameters: { "));
    DEBUG_PRINT(F("res: ")); PRINT2((uint16_t)msg -> result, HEX);
    DEBUG_PRINTLN(F(" }"));
}

void my_rsp_gap_discover(const ble_msg_gap_discover_rsp_t *msg) {
    DEBUG_PRINT(F("<--\tdiscover: { "));
    DEBUG_PRINT(F("res: ")); DEBUG_PRINT2((uint16_t)msg -> result, HEX);
    DEBUG_PRINTLN(F(" }"));
}

void my_rsp_gap_end_procedure(const ble_msg_gap_end_procedure_rsp_t *msg) {
    ble_state = BLE_STATE_STANDBY;
    DEBUG_PRINT(F("<--\tend_procedure: { "));
    DEBUG_PRINT(F("res: ")); DEBUG_PRINT2((uint16_t)msg -> result, HEX);
    DEBUG_PRINTLN(F(" }"));
}

void my_rsp_attclient_attribute_write(const ble_msg_attclient_attribute_write_rsp_t *msg) {
    DEBUG_PRINT(F("<--\tattribute_write: { "));
    DEBUG_PRINT(F("res: ")); DEBUG_PRINT2((uint16_t)msg -> result, HEX);
    DEBUG_PRINTLN(F(" }"));
}

void my_rsp_connection_disconnect(const ble_msg_connection_disconnect_rsp_t *msg) {
    DEBUG_PRINT(F("<--\tconnection_disconnect"));
    DEBUG_PRINT(F("res: ")); DEBUG_PRINT2((uint16_t)msg -> result, HEX);
    DEBUG_PRINTLN(F(" }"));
    // ble_state = BLE_STATE_STANDBY; 
}

// ================================================================
// USER-DEFINED BGLIB EVENT CALLBACKS
// ================================================================

void my_evt_system_boot(const ble_msg_system_boot_evt_t *msg) {
    DEBUG_PRINT(F("###\tsys_boot: { "));
    DEBUG_PRINT(F("maj: ")); DEBUG_PRINT2(msg -> major, HEX);
    DEBUG_PRINT(F(", min: ")); DEBUG_PRINT2(msg -> minor, HEX);
    DEBUG_PRINT(F(", pat: ")); DEBUG_PRINT2(msg -> patch, HEX);
    DEBUG_PRINT(F(", bld: ")); DEBUG_PRINT2(msg -> build, HEX);
    DEBUG_PRINT(F(", ll_ver: ")); DEBUG_PRINT2(msg -> ll_version, HEX);
    DEBUG_PRINT(F(", prot_ver: ")); DEBUG_PRINT2(msg -> protocol_version, HEX);
    DEBUG_PRINT(F(", hw: ")); DEBUG_PRINT2(msg -> hw, HEX);
    DEBUG_PRINTLN(F(" }"));
}

void my_evt_gap_scan_response(const ble_msg_gap_scan_response_evt_t *msg) {
  #ifdef DEBUG
    Serial.print(F("###\tscan_response: { "));
    Serial.print(F("rssi: ")); Serial.print(msg -> rssi);
    Serial.print(F(", packet_type: ")); Serial.print((uint8_t)msg -> packet_type, HEX);
    Serial.print(F(", sender: "));
    // this is a "bd_addr" data type, which is a 6-byte uint8_t array
    for (uint8_t i = 0; i < 6; i++) {
        if (msg -> sender.addr[i] < 16) Serial.write('0');
        Serial.print(msg -> sender.addr[i], HEX);
    }
    Serial.print(F(", add_type: ")); Serial.print(msg -> address_type, HEX);
    Serial.print(F(", bond: ")); Serial.print(msg -> bond, HEX);
    Serial.print(F(", data: "));
    // this is a "uint8array" data type, which is a length byte and a uint8_t* pointer
    for (uint8_t i = 0; i < msg -> data.len; i++) {
        if (msg -> data.data[i] < 16) Serial.write('0');
        Serial.print(msg -> data.data[i], HEX);
    }
    Serial.println(F(" }"));
   #endif
   
    int i = 0;
    bool found = 0;
    char sensorTagLocalName[] = "SensorTag";
    while (i < msg->data.len) {
      int length = msg->data.data[i] - 1; // since 1 octet is for type of data
      int type = msg->data.data[i+1];
      if (type == 0x09 && length == 9) { // matches length of string SensorTag
          // Serial.println(F("Matched type and length"));
          // assume found unless proven otherwise
          found = 1;
          for (int j = i+2, k=0; j < i+2+length, k<9; j++, k++) {
              char c = msg->data.data[j];
              if (c != sensorTagLocalName[k]) {
                  found = 0;
                  break;  
              }
          }
      }
      i = i+(length+1);
    }
    if (found) { 
      PRINTLN(F("Found SensorTag..."));
          
      if (msg->address_type == 0 && ble_state < BLE_STATE_CONNECTING) {    
          ble_state = BLE_STATE_CONNECTING; 
        #ifdef DEBUG
          Serial.print(F("-->\tconnect_direct: {"));
          Serial.print(F("addr: "));
          // this is a "bd_addr" data type, which is a 6-byte uint8_t array
          for (uint8_t i = 0; i < 6; i++) {
              if (msg -> sender.addr[i] < 16) Serial.write('0');
              Serial.print(msg -> sender.addr[i], HEX);
          }
      
          Serial.println(F(" }"));
        #endif
          uint8_t status;
          ble112.ble_cmd_gap_connect_direct(msg->sender, msg->address_type, 0x20, 0x30, 0x100, 0);
          // while ((status = ble112.checkActivity(1000)));
          PRINTLN(F("Connecting..."));
      }
    }
}

void my_ble_evt_connection_status(const ble_msg_connection_status_evt_t *msg) {
  DEBUG_PRINTLN(F("###\tconnect_direct_response: {"));
  if ((msg->flags & 0x05) == 0x05)  {
    ble_state = BLE_STATE_CONNECTED;
  #ifdef DEBUG
    for (uint8_t i = 0; i < 6; i++) {
      if (msg -> address.addr[i] < 16) Serial.write('0');
      Serial.print(msg -> address.addr[i], HEX);
    }
    Serial.println();
  #endif
    connection_handle = msg->connection;
    PRINTLN(F("Connected!"));
    // Toggle all necessary readings
    // temp_measurement starts the process, to be continued in procedure_completed
    turnOnMeasurement(temp_measurement);
    
  } else {
    PRINTLN(F("Connection failed")); 
  }
}

void my_ble_evt_attclient_procedure_completed(const ble_msg_attclient_procedure_completed_evt_t *msg) {
    DEBUG_PRINT(F("proc_completed--> ble_state :")); DEBUG_PRINT(ble_state); DEBUG_PRINT(F(" ble_state_index: ")); DEBUG_PRINTLN(ble_state_index);
    DEBUG_PRINT(F("msg->chrhandle is: ")); DEBUG_PRINTLN(msg->chrhandle);
    // 0 = temp, 1 = accl, 2 = humidity, 3 = magm, 4 = barometer, 5 = gyrascope
    if (ble_state == BLE_STATE_CONNECTED) {
      if (ble_state_index == 0) {
        ble_state_index = 1;
        turnOnMeasurement(accl_measurement); 
      } else if (ble_state_index == 1) {
        ble_state_index = 2;
        turnOnMeasurement(humidity_measurement); 
      } else if (ble_state_index == 2) {
        ble_state_index = 3;
        turnOnMeasurement(magm_measurement);
      } else if (ble_state_index == 3) {
        ble_state_index = 4;
        turnOnMeasurement(barometer_measurement);
      } else if (ble_state_index == 4) {
        ble_state_index = 5;
        turnOnMeasurement(gyroscope_measurement);
      } else if (ble_state_index == 5) {
        ble_state = BLE_STATE_CONNECTED_MEASUREMENTS;
        ble_state_index = 0; 
        // do something else here
        turnOnMeasurementCCC(temp_measurement);
      }
   
      
    } else if (ble_state == BLE_STATE_CONNECTED_MEASUREMENTS) {
       if (ble_state_index == 0) {
        ble_state_index = 1;
        turnOnMeasurementCCC(accl_measurement); 
      } else if (ble_state_index == 1) {
        ble_state_index = 2;
        turnOnMeasurementCCC(humidity_measurement); 
      } else if (ble_state_index == 2) {
        ble_state_index = 3;
        turnOnMeasurementCCC(magm_measurement);
      } else if (ble_state_index == 3) {
        ble_state_index = 4;
        turnOnMeasurementCCC(barometer_measurement);
      } else if (ble_state_index == 4) {
        ble_state_index = 5;
        turnOnMeasurementCCC(gyroscope_measurement);
      } else if (ble_state_index == 5) {
        ble_state = BLE_STATE_CONNECTED_MEASUREMENTS_DONE;
        ble_state_index = 0;
      }     
    } else if (ble_state == BLE_STATE_CONNECTED_MEASUREMENTS_DONE) {
    
    Measurement *myMeasurement;
    switch(msg->chrhandle) {
      case 38:
      case 41:
        myMeasurement = &temp_measurement;  
        break;
      case 49:
      case 52:
        myMeasurement = &accl_measurement;  
        break;
      case 60:
      case 63:
        myMeasurement = &humidity_measurement;  
        break;
      case 71:
      case 74:
        myMeasurement = &magm_measurement;  
        break;
      case 82:
      case 85:
        myMeasurement = &barometer_measurement; 
        break;
      case 97:
      case 100:
        myMeasurement = &gyroscope_measurement;  
        break;
    }
    
    if (myMeasurement->measurementOn && msg->chrhandle == myMeasurement->measurement_switch) {
      uint8_t status;
      uint8 data2[2] = {myMeasurement->measurementOn, 0x00};
      ble112.ble_cmd_attclient_attribute_write(connection_handle, myMeasurement->measurement_ccc, 2, (const uint8*)&data2);
      while ((status = ble112.checkActivity(1000)));
    }
    
    if (!myMeasurement->measurementOn && msg->chrhandle == myMeasurement->measurement_ccc) {
      uint8 data[1] = { myMeasurement->measurementOn};
      uint8_t status;
      ble112.ble_cmd_attclient_attribute_write(connection_handle, myMeasurement->measurement_switch, 1, (const uint8*)&data);
      while ((status = ble112.checkActivity(1000)));       
    }

    }
    DEBUG_PRINTLN(F("proc_completed<--"));
}


void my_ble_evt_attclient_attribute_value(const ble_msg_attclient_attribute_value_evt_t *msg) {
  DEBUG_PRINTLN(F("attr_val"));
  if (msg->connection == connection_handle && msg->atthandle == att_handle_temp_measurement) {
#ifdef DEBUG
    Serial.print(F("temp raw val: "));
    for(int i=msg->value.len-1; i >= 0; i--) {
      Serial.print(msg->value.data[i], HEX);
      Serial.print(F(" "));
    }    
    Serial.println();
#endif    
    writeToSDCard("temp", &msg->value);
//    if (msg->value.len == 4) {
//        uint16 rawTdie, rawTtarget;
//        rawTdie = msg->value.data[3]<<8 | msg->value.data[2];
//        rawTtarget = msg->value.data[1]<<8 | msg->value.data[0];
//        double dieT = calcTmpLocal(rawTdie);
//        double targetT = calcTmpTarget(rawTtarget, dieT);
//        Serial.print(F("Ambient Temp:")); Serial.println(dieT);
//        Serial.print(F("IR Temp")); Serial.println(targetT);        
//        double args[] = { dieT, targetT};
//        writeToSDCard("temp", 0, args);
//        Serial.print(F("Ambient temperature raw: "));
//        Serial.print(rawTdie);
//        Serial.print(F(" IR temperature raw: "));
//        Serial.print(rawTtarget);        
//        Serial.print(F(" Ambient temperature: "));
//        Serial.print(dieT);
//        Serial.print(F(" IR temperature: "));  
//        Serial.println(targetT);      
//    } else {
//        Serial.println(F("Data not in expected format"));  
//    }

  } else   if (msg->connection == connection_handle && msg->atthandle == att_handle_accl_measurement) {
#ifdef DEBUG
    Serial.print(F("accl raw val: "));
    for(int i=msg->value.len-1; i >= 0; i--) {
      Serial.print(msg->value.data[i], HEX);
      Serial.print(F(" "));
    }    
    Serial.println();
#endif    
    writeToSDCard("accl", &msg->value);

  } else if (msg->connection == connection_handle && msg->atthandle == att_handle_humidity_measurement) {
#ifdef DEBUG    
    Serial.print(F("humidity raw val: "));
    for(int i=msg->value.len-1; i >= 0; i--) {
      Serial.print(msg->value.data[i], HEX);
      Serial.print(F(" "));
    }    
    Serial.println();
#endif
    writeToSDCard("humi", &msg->value);

  } else   if (msg->connection == connection_handle && msg->atthandle == att_handle_magm_measurement) {
#ifdef DEBUG
    Serial.print(F("magm raw val: "));
    for(int i=msg->value.len-1; i >= 0; i--) {
      Serial.print(msg->value.data[i], HEX);
      Serial.print(F(" "));
    }    
    Serial.println();
#endif
    writeToSDCard("magm", &msg->value);

  } else   if (msg->connection == connection_handle && msg->atthandle == att_handle_barometer_measurement) {
#ifdef DEBUG
    Serial.print(F("barometer raw val: "));
    for(int i=msg->value.len-1; i >= 0; i--) {
      Serial.print(msg->value.data[i], HEX);
      Serial.print(F(" "));
    }    
    Serial.println();
#endif    
    writeToSDCard("barom", &msg->value);

  } else   if (msg->connection == connection_handle && msg->atthandle == att_handle_gyroscope_measurement) {
#ifdef DEBUG
    Serial.print(F("gyroscope raw val: "));
    for(int i=msg->value.len-1; i >= 0; i--) {
      Serial.print(msg->value.data[i], HEX);
      Serial.print(F(" "));
    }    
    Serial.println();
#endif    
    writeToSDCard("gyro", &msg->value);   
  } else {
      DEBUG_PRINTLN(F("attr val for unknown handle")); 
  }

}

void writeTimeToFile(File *dataFile) 
{
    dataFile->print(int(RTC.getMonths()));
    dataFile->print("/");  
    dataFile->print(int(RTC.getDays()));
    dataFile->print("/");  
    dataFile->print(RTC.getYears());
    dataFile->print("  ");
    dataFile->print(int(RTC.getHours()));
    dataFile->print(":");
    dataFile->print(int(RTC.getMinutes()));
    dataFile->print(":");
    dataFile->print(int(RTC.getSeconds()));   
}

void writeToSDCard(const char *sensor, const uint8array *value)
{
    if (!sdInitialized) {
        PRINT(F("No SD!: Data : ")); 
        PRINT(sensor); PRINT(F(" ")); 
        for(int i=value->len-1; i >= 0; i--) {
            if (value->data[i] < 16) WRITE('0');
            PRINT2(value->data[i], HEX);
            PRINT(F(", "));
        }
        PRINTLN();
        return;  
    }
    
    RTC.readTime();
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    // this opens the file and appends to the end of file
    // if the file does not exist, this will create a new file.
    File dataFile = SD.open(sensor, FILE_WRITE);
    
    // if the file is available, write to it:
    if (dataFile) 
    {  
      //int timeStamp = millis();
      //dataFile.print(timeStamp);
      writeTimeToFile(&dataFile);   
      
      dataFile.print(F(", "));
      // Serial.print(timeStamp);
      // Serial.print(", ");
      dataFile.print(sensor);
      dataFile.print(F(", "));      
      for(int i=value->len-1; i >= 0; i--) {
        if (value->data[i] < 16) dataFile.write('0');
        dataFile.print(value->data[i], HEX);
        dataFile.print(F(", "));
      }    
      dataFile.println();
      dataFile.close();
      // Serial.println();
      // print to the serial port too:
    }  
    // if the file isn't open, pop up an error:
    else
    {
      PRINT(F("err opening "));
      PRINTLN(sensor);
    } 
}

void writeToSDCard(const char *sensor, int numArgs, double *argArray)
{
    if (!sdInitialized) {
        PRINT(F("No SD!: Data : ")); 
        PRINT(sensor); PRINT(F(" ")); 
        for(int i=0; i < numArgs; i++) {
            PRINT(argArray[i]);
            PRINT(F(", "));
        }
        PRINTLN();
        return;  
    }
    
    RTC.readTime();
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    // this opens the file and appends to the end of file
    // if the file does not exist, this will create a new file.
    File dataFile = SD.open(sensor, FILE_WRITE);
    
    // if the file is available, write to it:
    if (dataFile) 
    {  
      //int timeStamp = millis();
      //dataFile.print(timeStamp);
      writeTimeToFile(&dataFile);   
      
      dataFile.print(F(", "));
      // Serial.print(timeStamp);
      // Serial.print(", ");
      dataFile.print(sensor);
      dataFile.print(F(", "));      
      for(int i=0; i < numArgs; i++) {
        dataFile.print(argArray[i]);
        dataFile.print(F(", "));
      }    
      dataFile.println();
      dataFile.close();
      // Serial.println();
      // print to the serial port too:
    }  
    // if the file isn't open, pop up an error:
    else
    {
      PRINT(F("err opening "));
      PRINTLN(sensor);
    } 
}


void updateShiftRegister(byte myLeds)
{
   digitalWrite(LATCH_PIN, LOW);
   shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, myLeds);
   digitalWrite(LATCH_PIN, HIGH);
}

void my_ble_evt_connection_disconnected(const struct ble_msg_connection_disconnected_evt_t *msg) {
    DEBUG_PRINT(F("###\tconn disconnected: "));
    DEBUG_PRINT(F("reason: ")); DEBUG_PRINT2((uint16_t)msg -> reason, HEX);
    DEBUG_PRINTLN(F(" }"));
    ble_state = BLE_STATE_STANDBY; 
    connect();  
}

/*  Conversion algorithm for die temperature */
double calcTmpLocal(uint16 rawT)
{
  //-- calculate die temperature [°C] --
  return (double)((uint16)rawT)/128.0; // Used in also in the calc. below;
}

/* Conversion algorithm for target temperature */
double calcTmpTarget(uint16 rawT, double ambientT)
{
  //-- calculate target temperature [°C] -
  double Vobj2 = (double)(uint16)rawT;
  Vobj2 *= 0.00000015625;

  double Tdie2 = ambientT + 273.15;
  const double S0 = 6.4E-14;            // Calibration factor

  const double a1 = 1.75E-3;
  const double a2 = -1.678E-5;
  const double b0 = -2.94E-5;
  const double b1 = -5.7E-7;
  const double b2 = 4.63E-9;
  const double c2 = 13.4;
  const double Tref = 298.15;
  double S = S0*(1+a1*(Tdie2 - Tref)+a2*pow((Tdie2 - Tref),2));
  double Vos = b0 + b1*(Tdie2 - Tref) + b2*pow((Tdie2 - Tref),2);
  double fObj = (Vobj2 - Vos) + c2*pow((Vobj2 - Vos),2);
  double tObj = pow(pow(Tdie2,4) + (fObj/S),.25);
  tObj = (tObj - 273.15);

  return tObj;
}



