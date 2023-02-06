#include <phyphoxBle.h>
#include <Adafruit_NeoPixel.h>

#include <Adafruit_ADS1X15.h>
#include <Adafruit_INA219.h>
#include <Wire.h>

#define NUMPIXELS        1
#define VBATPIN A13

Adafruit_ADS1015 ads;
Adafruit_INA219 ina219;
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

float current_mA = 0;
float last_t = 0;
float last_current = 0;
float total_charge = 0;

const int buttonPin = 38;
int buttonState = 0;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 2000;

void setup(void) {
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_38, 0); // how we wake from deep sleep
  pinMode(buttonPin, INPUT);

  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(38400);

#if defined(NEOPIXEL_POWER)   // ensure that when reawoken power is restored to neopixel and QT Stemma port
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

  if (!ads.begin()) {  // use !ads.begin(0x49) if jumper set
    Serial.println("Failed to find ADS");
    pixels.fill(0xfc0505); //red warning LED
    pixels.show();
    while (1);
  }

  //Getting differential reading from AIN0 (P) and AIN1 (N)
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!

  // Note this is in conjunction with the potential divider on the Pimoroni board we use

  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  if (! ina219.begin()) {
    Serial.println("Failed to find INA219");
    pixels.fill(0xfc0505); //red warning LED
    pixels.show();
    while (1);
  }

  //####### phyphox experiment setup #####################################

  PhyphoxBLE::start("Voltmeter - Ammeter");
  PhyphoxBleExperiment experiment;

  experiment.setTitle("Voltmeter - Ammeter");
  experiment.setCategory("Arduino Experiments");
  experiment.setDescription("Plot Voltage and Current against time, I/V characteristics and Capacitor Graphs");

  //View
  PhyphoxBleExperiment::View firstview;
  firstview.setLabel("Time Graphs");
  PhyphoxBleExperiment::View secondview;
  secondview.setLabel("I/V Graph");
  PhyphoxBleExperiment::View thirdview;
  thirdview.setLabel("Raw Values");
  PhyphoxBleExperiment::View fourthview;
  fourthview.setLabel("Capacitors");

  //Graphs
  PhyphoxBleExperiment::Graph graph;
  graph.setLabel("Voltage");
  graph.setUnitX("s");
  graph.setUnitY("V");
  graph.setLabelX("time");
  graph.setLabelY("Voltage");

  graph.setChannel(1, 2);

  PhyphoxBleExperiment::Graph secondGraph;
  secondGraph.setLabel("Current");
  secondGraph.setUnitX("s");
  secondGraph.setUnitY("mA");
  secondGraph.setLabelX("time");
  secondGraph.setLabelY("Current");
  secondGraph.setColor("318E2E");

  secondGraph.setChannel(1, 3);

  PhyphoxBleExperiment::Graph thirdGraph;
  thirdGraph.setLabel("Charge");
  thirdGraph.setUnitX("s");
  thirdGraph.setUnitY("mC");
  thirdGraph.setLabelX("time");
  thirdGraph.setLabelY("Charge");
  thirdGraph.setColor("318E2E");

  thirdGraph.setChannel(1, 4);

  PhyphoxBleExperiment::Graph fourthGraph;
  fourthGraph.setLabel("I/V");
  fourthGraph.setUnitX("V");
  fourthGraph.setUnitY("mA");
  fourthGraph.setLabelX("Voltage");
  fourthGraph.setLabelY("Current");
  fourthGraph.setStyle("dots");
  fourthGraph.setColor("318E2E");

  fourthGraph.setChannel(2, 3);

  PhyphoxBleExperiment::Graph fifthGraph;
  fifthGraph.setLabel("Q/V");
  fifthGraph.setUnitX("V");
  fifthGraph.setUnitY("mC");
  fifthGraph.setLabelX("Voltage");
  fifthGraph.setLabelY("Charge");
  fifthGraph.setStyle("dots");
  fifthGraph.setColor("318E2E");

  fifthGraph.setChannel(2, 4);

  PhyphoxBleExperiment::Graph sixthGraph;
  sixthGraph.setLabel("log linear");
  sixthGraph.setUnitX("s");
  sixthGraph.setUnitY("lnV");
  sixthGraph.setLabelX("time");
  sixthGraph.setLabelY("lnV");

  sixthGraph.setChannel(1, 5);

  //Separator
  PhyphoxBleExperiment::Separator mySeparator;
  mySeparator.setHeight(0.3);
  mySeparator.setColor("404040");

  //Value
  PhyphoxBleExperiment::Value myValue;
  myValue.setLabel("Voltage");
  myValue.setPrecision(2);
  myValue.setUnit("V");
  myValue.setColor("FFFFFF");
  myValue.setChannel(2);
  myValue.setXMLAttribute("size=\"2\"");

  PhyphoxBleExperiment::Value myValue2;
  myValue2.setLabel("Current");
  myValue2.setPrecision(2);
  myValue2.setUnit("mA");
  myValue2.setColor("FFFFFF");
  myValue2.setChannel(3);
  myValue2.setXMLAttribute("size=\"2\"");

  //Populate Views
  firstview.addElement(graph);
  firstview.addElement(secondGraph);
  firstview.addElement(mySeparator);
  firstview.addElement(myValue);
  firstview.addElement(myValue2);
  secondview.addElement(fourthGraph);
  thirdview.addElement(myValue);
  thirdview.addElement(myValue2);
  fourthview.addElement(graph);
  fourthview.addElement(secondGraph);
  fourthview.addElement(thirdGraph);
  fourthview.addElement(fifthGraph);
  fourthview.addElement(sixthGraph);

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

void loop(void) {
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
    pixels.fill(0xfc9905); // yellow warning LED that button is held
    pixels.setBrightness(5);
    pixels.show();

    total_charge = 0; //also use button to reset charge stored to zero

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

  current_mA = ina219.getCurrent_mA();
  float charge = ((current_mA + last_current) / 2) * (t - last_t);
  total_charge = charge + total_charge;

  int16_t results;
  float   multiplier = 0.01052631579F;
  results = ads.readADC_Differential_0_1();
  float voltage = results * multiplier;
  float logV = log(voltage);

  PhyphoxBLE::write(t, voltage, current_mA, total_charge, logV);

  last_t = t;
  last_current = current_mA;

  delay(10);
}
