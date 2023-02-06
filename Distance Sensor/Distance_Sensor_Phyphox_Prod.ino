#include <phyphoxBle.h>
#include <Adafruit_NeoPixel.h>

#include <VL53L1X.h>
#include <Wire.h>

#define NUMPIXELS        1
#define VBATPIN A13

VL53L1X sensor;
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

float last_t = 0;
float last_distance = 0;
float last_velocity = 0;

const int buttonPin = 38;
int buttonState = 0;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 2000;

const int numReadings = 3;      // averaging distance over 3 readings for v and a
float readings[numReadings];    // array to store distance
int readIndex = 0;              // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average

void setup() {
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_38, 0); // how we wake from deep sleep
  pinMode(buttonPin, INPUT);

  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(38400);

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

#if defined(NEOPIXEL_POWER) // ensure that when reawoken power is restored to neopixel and QT Stemma port
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

#if defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
#endif

  pixels.begin(); // INITIALIZE NeoPixel strip object
  pixels.setBrightness(5);
  pixels.fill(0xfc9905); // yellow setup light
  pixels.show();

  //####### Sensor setup ##################################################

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Sensor not found");
    pixels.fill(0xfc0505); //red warning LED
    pixels.show();
    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous(20);

  //####### phyphox experiment setup #####################################

  PhyphoxBLE::start("Distance Sensor");
  PhyphoxBleExperiment experiment;

  experiment.setTitle("ToF Distance Sensor");
  experiment.setCategory("Arduino Experiments");
  experiment.setDescription("Plot Distance, Velocity and Accel against time");

  //View
  PhyphoxBleExperiment::View firstview;
  firstview.setLabel("Distance Time");
  PhyphoxBleExperiment::View secondview;
  secondview.setLabel("Velocity Time");
  PhyphoxBleExperiment::View thirdview;
  thirdview.setLabel("All graphs");
  PhyphoxBleExperiment::View fourthview;
  fourthview.setLabel("Raw Values");

  //Graphs
  PhyphoxBleExperiment::Graph graph;
  graph.setLabel("Distance");
  graph.setUnitX("s");
  graph.setUnitY("mm");
  graph.setLabelX("time");
  graph.setLabelY("distance");

  graph.setChannel(3, 1);

  PhyphoxBleExperiment::Graph secondGraph;
  secondGraph.setLabel("Velocity");
  secondGraph.setUnitX("s");
  secondGraph.setUnitY("mms-1");
  secondGraph.setLabelX("time");
  secondGraph.setLabelY("velocity");
  secondGraph.setColor("318E2E");

  secondGraph.setChannel(3, 4);

  PhyphoxBleExperiment::Graph thirdGraph;
  thirdGraph.setLabel("Acceleration");
  thirdGraph.setUnitX("s");
  thirdGraph.setUnitY("ms-2");
  thirdGraph.setLabelX("time");
  thirdGraph.setLabelY("accel");
  thirdGraph.setColor("2E728E");

  thirdGraph.setChannel(3, 5);

  //Separator
  PhyphoxBleExperiment::Separator mySeparator;
  mySeparator.setHeight(0.3);
  mySeparator.setColor("404040");

  //Value
  PhyphoxBleExperiment::Value myValue;
  myValue.setLabel("Distance");
  myValue.setPrecision(2);
  myValue.setUnit("mm");
  myValue.setColor("FFFFFF");
  myValue.setXMLAttribute("size=\"2\"");
  myValue.setChannel(1);

  PhyphoxBleExperiment::Value myValue2;
  myValue2.setLabel("Velocity");
  myValue2.setPrecision(2);
  myValue2.setUnit("mms-1");
  myValue2.setColor("FFFFFF");
  myValue2.setXMLAttribute("size=\"2\"");
  myValue2.setChannel(4);

  PhyphoxBleExperiment::Value myValue3;
  myValue3.setLabel("Accel");
  myValue3.setPrecision(2);
  myValue3.setUnit("mms-2");
  myValue3.setColor("FFFFFF");
  myValue3.setXMLAttribute("size=\"2\"");
  myValue3.setChannel(5);

  PhyphoxBleExperiment::Value myValue4;
  myValue4.setLabel("Battery Level");
  myValue4.setPrecision(2);
  myValue4.setUnit("V");
  myValue4.setColor("FFFFFF");
  myValue4.setChannel(2);

  //Populate Views
  firstview.addElement(graph);                 
  firstview.addElement(myValue); 
  secondview.addElement(secondGraph);
  secondview.addElement(myValue2);
  thirdview.addElement(graph);
  thirdview.addElement(mySeparator);
  thirdview.addElement(secondGraph);
  thirdview.addElement(mySeparator);
  thirdview.addElement(thirdGraph);
  fourthview.addElement(myValue);
  fourthview.addElement(myValue2);
  fourthview.addElement(myValue3);
  fourthview.addElement(myValue4);
  experiment.addView(firstview);
  experiment.addView(secondview);
  experiment.addView(thirdview);
  experiment.addView(fourthview);

  PhyphoxBLE::addExperiment(experiment);

  //####### Setup complete #####################################
  pixels.setBrightness(1);
  pixels.fill(0x05fc15); //Green status LED
  pixels.show();
}

void disableInternalPower() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, LOW);
#endif

#if defined(NEOPIXEL_I2C_POWER)
  pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_I2C_POWER, LOW);
#endif
}

void LEDoff() {
#if defined(PIN_NEOPIXEL)
  pixels.setPixelColor(0, 0x0);
  pixels.show();
#endif
}

void loop() {
  // ################## Deep sleep button setup, 2 sec push to put to sleep ############
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        pixels.fill(0xfc0505); // red warning LED, going to sleep
        pixels.setBrightness(5);
        pixels.show();
        delay(3000);
        disableInternalPower();
        LEDoff();
        esp_deep_sleep_start();
      }
    }
  }

  if (reading == LOW) {
    pixels.fill(0xfc9905);  // yellow warning LED that button is held
    pixels.setBrightness(5);
    pixels.show();
  } else {
    pixels.fill(0x05fc15);
    pixels.setBrightness(1);
    pixels.show();
  }

  lastButtonState = reading;

  // ################## Collect data and broadcast to phyphox ############

  float measuredvbat = analogReadMilliVolts(VBATPIN);
  measuredvbat *= 2;
  measuredvbat /= 1000;

  float t = 0.001 * (float)millis();

  sensor.read();
  float range = sensor.ranging_data.range_mm ;

  total = total - readings[readIndex];
  readings[readIndex] = range ;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  // calculate the average:
  float distance = total / numReadings;
  float velocity = ((distance - last_distance) / (t - last_t)) ;
  float accel = ((velocity - last_velocity) / (t - last_t)) ;

  PhyphoxBLE::write(range, measuredvbat, t,  velocity, accel);

  //Serial.print(t);
  // Serial.print(",");
  //Serial.print(sensor.ranging_data.range_mm);
  //Serial.println(",");

  last_t = t;
  last_distance = distance;
  last_velocity = velocity;
}
