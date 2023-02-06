#include <phyphoxBle.h>
#include <Adafruit_NeoPixel.h>

#include "Adafruit_VEML7700.h"

#define NUMPIXELS        1
#define VBATPIN A13

Adafruit_VEML7700 veml = Adafruit_VEML7700();
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

const int buttonPin = 38;
int buttonState = 1;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 2000;

void setup() {
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_38, 0); // how we wake from deep sleep
  pinMode(buttonPin, INPUT);

  Serial.begin(38400);

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

  if (!veml.begin()) {
    Serial.println("Sensor not found");
    pixels.fill(0xfc0505); //red warning LED
    pixels.show();
    while (1) ;
  }

  veml.setGain(VEML7700_GAIN_1_4);
  veml.setIntegrationTime(VEML7700_IT_50MS);

  //####### phyphox experiment setup #####################################

  PhyphoxBLE::start("Light Sensor");
  PhyphoxBleExperiment experiment;

  experiment.setTitle("Light Intensity");
  experiment.setCategory("Arduino Experiments");
  experiment.setDescription("Plot Light Intensity against Time");

  //View
  PhyphoxBleExperiment::View firstview;
  firstview.setLabel("Graphs");  //Create a "view"
  PhyphoxBleExperiment::View secondview;
  secondview.setLabel("Raw Values");

  //Graphs
  PhyphoxBleExperiment::Graph graph;
  graph.setLabel("Light Intensity");
  graph.setUnitX("s");
  graph.setUnitY("lux");
  graph.setLabelX("time");
  graph.setLabelY("light intensity");

  graph.setChannel(1, 2);

  //Value
  PhyphoxBleExperiment::Value myValue;
  myValue.setLabel("Light Intensity");
  myValue.setPrecision(2);
  myValue.setUnit("lux");
  myValue.setColor("FFFFFF");
  myValue.setChannel(2);

  PhyphoxBleExperiment::Value myValue2;
  myValue2.setLabel("Battery Level");
  myValue2.setPrecision(2);
  myValue2.setUnit("V");
  myValue2.setColor("FFFFFF");
  myValue2.setChannel(3);

  //Populate Views
  firstview.addElement(graph);
  secondview.addElement(myValue);
  secondview.addElement(myValue2);
  experiment.addView(firstview);
  experiment.addView(secondview);

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
  
  float intensity = veml.readLux();

  //Serial.print("lux: ");
  //Serial.println(intensity);

  PhyphoxBLE::write(t, intensity, measuredvbat);
}
