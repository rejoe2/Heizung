/**
   The MySensors Arduino library handles the wireless radio link and protocol
   between your home built sensors/actuators and HA controller of choice.
   The sensors forms a self healing radio network with optional repeaters. Each
   repeater and gateway builds a routing tables in EEPROM which keeps track of the
   network topology allowing messages to be routed to nodes.

   Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
   Copyright (C) 2013-2015 Sensnology AB
   Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors

   Documentation: http://www.mysensors.org
   Support Forum: http://forum.mysensors.org

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   version 2 as published by the Free Software Foundation.

 *******************************

   REVISION HISTORY
   Version 1.0 - Henrik Ekblad
   Contribution by: Derek Macias

   DESCRIPTION
   Example showing how to create an atuator for a servo.
   Connect red to +5V, Black or brown to GND and the last cable to Digital pin 3.
   The servo consumes much power and should probably have its own powersource.'
   The arduino might spontanally restart if too much power is used (happend
   to me when servo tried to pass the extreme positions = full load).
   http://www.mysensors.org/build/servo
*/

// Enable debug prints to serial monitor
//#define MY_DEBUG
//#define MY_DEBUG_LOCAL

// Enable and select radio type attached
#define MY_RADIO_NRF24
#define MY_RF24_PA_LEVEL RF24_PA_MAX
//#define MY_RADIO_RFM69

// Enabled repeater feature for this node
#define MY_REPEATER_FEATURE
#define MY_NODE_ID 102
#define MY_TRANSPORT_RELAXED //activate as soon as pull request is merged use the following two lines:
//#define MY_TRANSPORT_DONT_CARE_MODE
//#define MY_PARENT_NODE_ID 0

#include <MySensors.h>
#include <SPI.h>
#include <Servo.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#define ONE_WIRE_BUS 5 // Pin where dallas sensor is connected 
#define MAX_ATTACHED_DS18B20 8
unsigned long SLEEP_TIME = 300000; // Sleep time between reads (in milliseconds)
unsigned long tempDelayShort = 10000; //Check cyclus for fast temperature rise
unsigned long tempDelayPump = 30000; //Check cyclus for switching off warmwater circle pump
unsigned long tempDelayHeatingPump = 45000; //Check cyclus for switching internal heating water pump

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature.
float lastTemperature[MAX_ATTACHED_DS18B20];
//int numSensors = 0;
boolean receivedConfig = false;
boolean metric = true;
// Initialize temperature message
MyMessage DallasMsg(0, V_TEMP);
int  resolution = 10;
int  conversionTime = 1000;

// arrays to hold device addresses
DeviceAddress dallasAddresses[] = {
  {0x28, 0xFF, 0x36, 0x98, 0x54, 0x14, 0x1, 0xC1}, // Sensor Internal Heating Pump
  {0x28, 0xF8, 0x24, 0xE5, 0x5, 0x0, 0x0, 0xD7}, // Warmwasser 28.F824E5050000.D7
  {0x28, 0xF9, 0x61, 0xE7, 0x5, 0x0, 0x0, 0xDC}, //{0x28, 0xF8, 0x24, 0xE5, 0x05, 0x0, 0x0, 0xDC}, // Warmwasser an der Umwälzpumpe 28.F961E7050000.DC
  {0x28, 0xFF, 0x0, 0x4A, 0x54, 0x14, 0x1, 0xDF}, // Heizung Rücklauf 28.FF004A541401.DF
  {0x28, 0xFF, 0xCE, 0x69, 0x36, 0x16, 0x4, 0xE3}, // Heizung Vorlauf 28FFCE69361604E3
  {0x28, 0xDC, 0x1A, 0xE6, 0x5, 0x0, 0x0, 0x13}, // Aussentemperatur Nord DC1AE6050000.13
  {0x28, 0x28, 0x6F, 0xE5, 0x5, 0x0, 0x0, 0x8A}, // Aussentemperatur Süd 28.286FE5050000.8A
  {0x28, 0x42, 0x6F, 0xE5, 0x5, 0x0, 0x0, 0x86} // Schildkröten 28.426FE5050000.86*/
};

int HP = 0; //Internal Heating Pump
int WW = 1; //binary # of warmwater sensor
int WP = 2; //WarmWater Pump
int RL = 3; //Rücklauf
int VL = 4; //Vorlauf
int AN = 5; //Aussentemperatur Nord
int AS = 6; //Aussentemperatur Sued
int TS = 7; //Schidkroeten

#define CHILD_ID_CONFIG 102   // Id for the temp-settings
#define CHILD_ID_CONFIG0 0   // Id for automatic mode
bool autoMode = 0;

#define SERVO_DIGITAL_OUT_PIN 4
#define SERVO_MIN 0 // Fine tune your servos min. 0-180
#define SERVO_MAX 180  // Fine tune your servos max. 0-180
#define DETACH_DELAY 900 // Tune this to let your movement finish before detaching the servo
#define CHILD_ID_SERVO 10   // Id of the sensor child

#define DIGITAL_INPUT_SENSOR 3  // The digital input you attached your light sensor.  (Only 2 and 3 generates interrupt!)
#define PULSE_FACTOR 100       // Nummber of blinks per m3 of your meter (One rotation/10 liter)
#define SLEEP_MODE false        // Watt-value can only be reported when sleep mode is false.
#define MAX_FLOW 100          // Max volume value to report. This filetrs outliers.
#define SENSOR_INTERRUPT DIGITAL_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)
#define CHILD_ID_GAS 11              // Id of the sensor child
unsigned long SEND_FREQUENCY = 300000; // Minimum time between send (in milliseconds). We don't want to spam the gateway.

#define RELAY_PIN  6  // Arduino Digital I/O pin number for relay 
#define CHILD_ID_RELAY 1   // Id of the sensor child
#define RELAY_ON 0
#define RELAY_OFF 1

//MySensor gw;
MyMessage ServoMsg(CHILD_ID_SERVO, V_DIMMER);
Servo myservo;  // create servo object to control a servo
// a maximum of eight servo objects can be created Sensor gw(9,10);
unsigned long timeOfLastChange = 0;
bool attachedServo = false;

MyMessage flowMsg(CHILD_ID_GAS, V_FLOW);
MyMessage volumeMsg(CHILD_ID_GAS, V_VOLUME);
MyMessage lastCounterMsg(CHILD_ID_GAS, V_VAR1);

double ppl = ((double)PULSE_FACTOR) / 1000;      // Pulses per liter

volatile unsigned long pulseCount = 0;
volatile unsigned long lastBlink = 0;
volatile double flow = 0;
boolean pcReceived = false;
unsigned long oldPulseCount = 0;
unsigned long newBlink = 0;
double oldflow = 0;
double volume = 0;
double oldvolume = 0;
unsigned long lastSend = 0;
unsigned long lastPulse = 0;

unsigned long lastCheckWater = 0;
unsigned long lastCheckPump = 0;
unsigned long lastCheckHeatingPump = 0;
unsigned long lastTempAll = 0;
int DeltaL3 = 14; //Temperature Delta for switching to internal pump level 3
int DeltaL2 = 11; //Temperature Delta for switching to internal pump level 3
unsigned long waitTimePumpSwitch = 600000; //10 min as minimum time to switch internal pump
unsigned long lastPumpSwitch = 0; //use this also for heartbeat?

//set defaults
int tempMaxPump = 45; //upper temp level at warmwater circle pump
int tempMaxHeatingPump = 65; //temperature to switch internal heating pump to highest level
int tempLowExtToLevelIII = -8; //External low temperature to switch internal heating pump to highest level
int tempLowExtToLevelII = 7; ////External highest temperature to switch internal heating pump to medium level
int val = 60;


int oldValue = 0;
bool state;
MyMessage RelayMsg(CHILD_ID_RELAY, V_LIGHT);

void before() {

  // initialize our digital pins internal pullup resistor so one pulse switches from high to low (less distortion)
  pinMode(DIGITAL_INPUT_SENSOR, INPUT_PULLUP);
  digitalWrite(DIGITAL_INPUT_SENSOR, HIGH);
  pulseCount = oldPulseCount = 0;
  attachInterrupt(SENSOR_INTERRUPT, onPulse, FALLING);

  // Then set relay pins in output mode
  pinMode(RELAY_PIN, OUTPUT);
  // Switch water pump on when starting up
  //digitalWrite(RELAY_PIN, RELAY_ON);

  conversionTime = 750 / (1 << (12 - resolution));
  // Startup up the OneWire library
  sensors.begin();
  // requestTemperatures() will not block current thread
  sensors.setWaitForConversion(false);
  // Fetch the number of attached temperature sensors
  //numSensors = sensors.getDeviceCount();
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Heating Environment", "0.96");
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_SERVO, S_COVER);
  present(CHILD_ID_GAS, S_WATER);
  present(CHILD_ID_RELAY, S_LIGHT);
  present(CHILD_ID_CONFIG, S_CUSTOM);
  present(CHILD_ID_CONFIG0, S_CUSTOM); //for automatic mode


  // Present all sensors to controller
  for (int i = 0; i < MAX_ATTACHED_DS18B20; i++) { //i < numSensors &&
    present(20 + i, S_TEMP);
    sensors.setResolution(dallasAddresses[i], resolution);
    wait(20);
  }
}

void setup() {
  // Request last servo state at startup
  request(CHILD_ID_SERVO, V_DIMMER);
  // Fetch last known pulse count value from gw
  request(CHILD_ID_GAS, V_VAR1);
  request(CHILD_ID_CONFIG, V_VAR1);
  request(CHILD_ID_CONFIG, V_VAR2);
  request(CHILD_ID_CONFIG, V_VAR3);
  request(CHILD_ID_CONFIG, V_VAR4);
  request(CHILD_ID_CONFIG0, V_VAR1);
  lastSend = lastPulse = lastCheckWater = lastCheckPump = lastCheckHeatingPump = lastTempAll = millis();
}

void loop()
{
  if (attachedServo && millis() - timeOfLastChange > DETACH_DELAY) {
    myservo.detach();
    attachedServo = false;
  }

  unsigned long currentTime = millis();

  // Only send values at a maximum frequency or woken up from sleep
  if (currentTime - lastSend > SEND_FREQUENCY)
  {
    lastSend = currentTime;
    if (flow != oldflow) {
      oldflow = flow;
#ifdef MY_DEBUG_LOCAL
      Serial.print("l/min:");
      Serial.println(flow);
#endif
      // Check that we dont get unresonable large flow value.
      // could hapen when long wraps or false interrupt triggered
      if (flow < ((unsigned long)MAX_FLOW)) {
        send(flowMsg.set(flow, 2));                   // Send flow value to gw
      }
    }

    // No Pulse count received in 2min
    if (currentTime - lastPulse > 120000) {
      flow = 0;
    }

    // Pulse count has changed
    if (pulseCount != oldPulseCount) {
      oldPulseCount = pulseCount;
#ifdef MY_DEBUG_LOCAL
      Serial.print("pulsecnt:");
      Serial.println(pulseCount);
#endif
      send(lastCounterMsg.set(pulseCount));                  // Send  pulsecount value to gw in VAR1

      double volume = ((double)pulseCount / ((double)PULSE_FACTOR));
      if (volume != oldvolume) {
        oldvolume = volume;
#ifdef MY_DEBUG_LOCAL
        Serial.print("vol:");
        Serial.println(volume, 3);
#endif
        send(volumeMsg.set(volume, 3));               // Send volume value to gw
      }
    }
  }

  //Loop for quick temperature rise hot water and check for higher temperaturess of water exiting burner
  unsigned long currentMillisShort = millis();
  if (currentMillisShort - lastCheckWater > tempDelayShort) {
    // Fetch temperatures from Dallas sensors
    sensors.requestTemperatures();
    wait(conversionTime);

    // Fetch and round temperature to one decimal
    float temperature = static_cast<float>(static_cast<int>((getConfig().isMetric ? sensors.getTempC(dallasAddresses[WW]) : sensors.getTempF(dallasAddresses[WW])) * 10.)) / 10.;
    // switch Relay on, if temperature on warmwater pipe rises fast and temperature at circlepump is not already at upper level
    if (digitalRead(RELAY_PIN) != RELAY_ON && lastTemperature[WW] + 0.2 < temperature && temperature != -127.00 && lastTemperature[WP] < tempMaxPump) {
      // Send in the new temperature
      send(DallasMsg.setSensor(WW + 20).set(temperature, 1));
      digitalWrite(RELAY_PIN, RELAY_ON);
      send(RelayMsg.set(true), true); // Send new state and request ack back
      // Write some debug info
#ifdef MY_DEBUG
      Serial.print("Int. change:");
      Serial.print(temperature);
      Serial.print("C, Rel. status: ON");
      Serial.println();
#endif
    }
    lastTemperature[WW] = temperature;
    lastCheckWater = currentMillisShort;

    // Fetch and round temperature to one decimal
    temperature = static_cast<float>(static_cast<int>((getConfig().isMetric ? sensors.getTempC(dallasAddresses[HP]) : sensors.getTempF(dallasAddresses[HP])) * 10.)) / 10.;

    //Emergency switching function heating pump
    //switch heating pump to max, if temperature at exit of burner is above warning level
    if (val < 3 && temperature != -127.00 && temperature > tempMaxHeatingPump) {
      // Send in the new temperature
      send(DallasMsg.setSensor(HP + 20).set(temperature, 1));
      val = 3;
      internalServo(val);
      // Write some debug info
#ifdef MY_DEBUG_LOCAL
      Serial.print("Int. change:");
      Serial.print(temperature);
      Serial.print("C, Servo. status: ");
      Serial.print(val);
      Serial.println();
#endif
    }
    lastTemperature[HP] = temperature;
  }

  //Loop for switching off circle pump
  unsigned long currentMillisPump = millis();
  if (currentMillisPump - lastCheckPump > tempDelayPump) {
    float temperature = static_cast<float>(static_cast<int>((getConfig().isMetric ? sensors.getTempC(dallasAddresses[WP]) : sensors.getTempF(dallasAddresses[WP])) * 10.)) / 10.;
    // switch Relay off, if temperature on warmwater is at or higher upper level
    if (digitalRead(RELAY_PIN) != RELAY_OFF && tempMaxPump < temperature && temperature != -127.00 ) {
      // Send in the new temperature
      send(DallasMsg.setSensor(WP + 20).set(temperature, 1));
      digitalWrite(RELAY_PIN, RELAY_OFF);
      send(RelayMsg.set(false), true); // Send new state and request ack back
#ifdef MY_DEBUG_LOCAL
      // Write some debug info
      Serial.print("Int. change:");
      Serial.print(temperature);
      Serial.print("C, Rel. status: OFF");
      Serial.println();
#endif
    }
    lastTemperature[WP] = temperature;
    lastCheckPump = currentMillisPump;
  }

  //Regular Loop for switching internal heating pump
  unsigned long currentMillisHeatingPump = millis();
  if (currentMillisHeatingPump - lastPumpSwitch > tempDelayHeatingPump && autoMode) {
    float temperature = static_cast<float>(static_cast<int>((getConfig().isMetric ? sensors.getTempC(dallasAddresses[AN]) : sensors.getTempF(dallasAddresses[AN])) * 10.)) / 10.;
    // switch pump to Level 3, if external temperature is below minimum level

    if (val < 3 && tempLowExtToLevelIII > temperature && temperature != -127.00 ) {
      val = 3;
      internalServo(val);
      // Send in the new temperature
      send(DallasMsg.setSensor(AN + 20).set(temperature, 1));
#ifdef MY_DEBUG
      // Write some debug info
      Serial.print("Int. change:");
      Serial.println(temperature);
#endif
      lastTemperature[AN] = temperature;
      lastPumpSwitch = currentMillisHeatingPump;

    } else if (val != 2 && tempLowExtToLevelII - 0.5 > temperature && lastTemperature[HP] < tempMaxHeatingPump - 5 && temperature != -127.00 ) {
      val = 2;
      internalServo(val);
      // Send in the new temperature
      send(DallasMsg.setSensor(AN + 20).set(temperature, 1));
#ifdef MY_DEBUG
      // Write some debug info
      Serial.print("Int. change:");
      Serial.println(temperature);
#endif
      lastTemperature[AN] = temperature;
      lastPumpSwitch = currentMillisHeatingPump;

    } else if (val > 1 && tempLowExtToLevelII + 0.5 < temperature && lastTemperature[HP] < tempMaxHeatingPump - 5 && temperature != -127.00 ) {
      val = 1;
      internalServo(val);
      // Send in the new temperature
      send(DallasMsg.setSensor(AN + 20).set(temperature, 1));
#ifdef MY_DEBUG
      // Write some debug info
      Serial.print("Int. change:");
      Serial.println(temperature);
#endif
      lastTemperature[AN] = temperature;
      lastPumpSwitch = currentMillisHeatingPump;

    }

    /*Ideas for advanced features

      (lastTemperature[VL] > 30 && lastTemperature[VL] - lastTemperature[RL] > DeltaL3 && lastCheckHeatingPump - lastPumpSwitch > waitTimePumpSwitch) {
      myservo.attach(SERVO_DIGITAL_OUT_PIN);
      attachedServo = true;
      val = 20;
      myservo.write(SERVO_MAX + (SERVO_MIN - SERVO_MAX) / 100 * val); // sets the servo position 0-180
      send(ServoMsg.set(val), true); // Send new state and request ack back
      // Write some debug info
      Serial.print("Servo change; state: ");
      Serial.println(val);
      lastPumpSwitch = currentMillisHeatingPump;
      timeOfLastChange = currentMillisHeatingPump;
      } else if (lastTemperature[VL] - lastTemperature[RL] > DeltaL2 && lastCheckHeatingPump - lastPumpSwitch > waitTimePumpSwitch) {
      myservo.attach(SERVO_DIGITAL_OUT_PIN);
      attachedServo = true;
      val = 60;
      myservo.write(SERVO_MAX + (SERVO_MIN - SERVO_MAX) / 100 * val); // sets the servo position 0-180
      send(ServoMsg.set(val), true); // Send new state and request ack back
      // Write some debug info
      Serial.print("Servo change; state: ");
      Serial.println(val);
      lastPumpSwitch = currentMillisHeatingPump;
      timeOfLastChange = currentMillisHeatingPump;

      } else if (lastTemperature[VL] > 30 && lastTemperature[VL] - lastTemperature[RL] < DeltaL2 && lastCheckHeatingPump - lastPumpSwitch > waitTimePumpSwitch) {
      myservo.attach(SERVO_DIGITAL_OUT_PIN);
      attachedServo = true;
      val = 100;
      myservo.write(SERVO_MAX + (SERVO_MIN - SERVO_MAX) / 100 * val); // sets the servo position 0-180
      send(ServoMsg.set(val), true); // Send new state and request ack back
      // Write some debug info
      Serial.print("Servo change; state: ");
      Serial.println(val);
      lastPumpSwitch = currentMillisHeatingPump;
      timeOfLastChange = currentMillisHeatingPump;

      }

      lastCheckHeatingPump = currentMillisHeatingPump;
      }
    */
    //Loop for regular temperature sensing
    unsigned long currentMillis = millis();
    if (currentMillis - lastTempAll > SLEEP_TIME) {
      // Read temperatures and send them to controller
      for (int i = 0; i < MAX_ATTACHED_DS18B20; i++) { //i < numSensors &&

        // Fetch and round temperature to one decimal
        float temperature = static_cast<float>(static_cast<int>((getConfig().isMetric ? sensors.getTempC(dallasAddresses[i]) : sensors.getTempF(dallasAddresses[i])) * 10.)) / 10.;
        //float temperature = static_cast<float>(static_cast<int>((getConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;

        // Only send data if temperature has no error
        if ( temperature != -127.00 && temperature != 85.00) {
          // Send in the new temperature
          send(DallasMsg.setSensor(i + 20).set(temperature, 1));
          // Save new temperatures for next compare
          lastTemperature[i] = temperature;
          wait(30);
#ifdef MY_DEBUG_LOCAL
          // Write some debug info
          Serial.print("Temperature ");
          Serial.print(i);
          Serial.print(" : ");
          Serial.println(temperature);
#endif

        }
      }
      lastTempAll = currentMillis;
    }
  }
}
void receive(const MyMessage & message) {
  if (message.sensor == CHILD_ID_SERVO) {
    myservo.attach(SERVO_DIGITAL_OUT_PIN);
    attachedServo = true;
    if (message.isAck()) {
      Serial.println("Ack from gw rec.");
    }
    if (message.type == V_DIMMER) { // This could be M_ACK_VARIABLE or M_SET_VARIABLE
      int val = message.getInt();
      if (val > 3) {
        val = 3;
      };
      //myservo.write(SERVO_MAX + (SERVO_MIN - SERVO_MAX) / 100 * val); // sets the servo position 0-180
      myservo.write(SERVO_MAX + (SERVO_MIN - SERVO_MAX) / 100 * (130 - val * 40)); // sets the servo position 0-180
      // Write some debug info
      //Serial.print("Servo change; state: ");
      //Serial.println(val);
    }
    timeOfLastChange = millis();
  }
  else if (message.sensor == CHILD_ID_GAS) {
    if (message.type == V_VAR1) {
      pulseCount = message.getULong();
      flow = oldflow = 0;
      //Serial.print("Rec. last pulse count from gw:");
      //Serial.println(pulseCount);
      pcReceived = true;
    }
  }
  else if (message.sensor == CHILD_ID_RELAY) {

    if (message.type == V_LIGHT) {
      // Change relay state
      state = message.getBool();
      digitalWrite(RELAY_PIN, state ? RELAY_ON : RELAY_OFF);
#ifdef MY_DEBUG
      // Write some debug info
      Serial.print("Gw change relay:");
      Serial.print(message.sensor);
      Serial.print(", New status: ");
      Serial.println(message.getBool());
#endif
    }
  }
  else if (message.sensor == CHILD_ID_CONFIG) {
    if (message.type == V_VAR1) {
      int tempMaxPump = message.getInt(); //upper temp level at warmwater circle pump
    }
    else if (message.type == V_VAR2) {
      int tempMaxHeatingPump = message.getInt(); //temperature to switch internal heating pump to highest level
    }
    else if (message.type == V_VAR3) {
      int tempLowExtToLevelIII = message.getInt(); //External low temperature to switch internal heating pump to highest level
    }
    else if (message.type == V_VAR4) {
      int tempLowExtToLevelII = message.getInt(); //External highest temperature to switch internal heating pump to medium level
    }
    else if (message.type == V_VAR5) {
      if (message.getBool()) {
        lastPumpSwitch = millis(); //if true: Reset timer (Heartbeat functionality)
#ifdef MY_DEBUG_LOCAL
        // Write some debug info
        Serial.print("Timer reset to ");
        Serial.print(lastPumpSwitch);
#endif
      } else {
        lastPumpSwitch = 0; //if false: switch immediately if necessary
#ifdef MY_DEBUG_LOCAL
        // Write some debug info
        Serial.print("Timer deleted, lastPumpSwitch=");
        Serial.print(lastPumpSwitch);
#endif
      }
    }
  }
  else if (message.sensor == CHILD_ID_CONFIG0) {
    if (message.type == V_VAR1) {
      int autoMode = message.getBool(); //enable autoMod
    }
  }
}

void onPulse()
{
  if (!SLEEP_MODE)
  {
    unsigned long newBlink = micros();
    unsigned long interval = newBlink - lastBlink;

    if (interval != 0)
    {
      lastPulse = millis();
      if (interval < 1000000L) {
        // Sometimes we get interrupt on RISING,  1000000 = 1sek debounce ( max 60 l/min)
        return;
      }
      flow = (60000000.0 / interval) / ppl;
    }
    lastBlink = newBlink;
  }
  pulseCount++;
}

void internalServo(int val1)
{
  myservo.attach(SERVO_DIGITAL_OUT_PIN);
  attachedServo = true;
  //myservo.write(SERVO_MAX + (SERVO_MIN - SERVO_MAX) / 100 * val1); // sets the servo position 0-180
  myservo.write(SERVO_MAX + (SERVO_MIN - SERVO_MAX) / 100 * (130 - val1 * 40)); // sets the servo position 0-180
  send(ServoMsg.set(val1), true);
  timeOfLastChange = millis();
}
