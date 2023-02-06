#include <phyphoxBle.h>
#include <Adafruit_NeoPixel.h>

#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include <Wire.h>

#define NUMPIXELS        1
#define VBATPIN A13

NAU7802 myScale;
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

float force = 0;
float mass = 0;
float zeroError = -257304; // this is reset using the button at any time
float CalibFactor = 392.53; // this MUST BE CALIBRATED - see build guide
float ADCvalue = 0;

const int buttonPin = 38;
int buttonState = 0;
int lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 2000;

void setup() {
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_38, 0); //1 = High, 0 = Low
  pinMode(buttonPin, INPUT);

  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(38400);

#if defined(NEOPIXEL_POWER)    // ensure that when reawoken power is restored to neopixel and QT Stemma port
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

  if (myScale.begin() == false) {
    Serial.println("Failed to find scale");
    pixels.fill(0xfc0505); //red warning LED
    pixels.show();
    while (1);
  }

  //####### phyphox experiment setup #####################################

  PhyphoxBLE::start("Force Sensor");
  PhyphoxBleExperiment experiment;

  experiment.setTitle("Load Cell Force Sensor");
  experiment.setCategory("Arduino Experiments");
  experiment.setDescription("Plot Force against time");

  //View
  PhyphoxBleExperiment::View firstview;
  firstview.setLabel("Force Time");
  PhyphoxBleExperiment::View secondview;
  secondview.setLabel("Raw Values");

  //Graphs
  PhyphoxBleExperiment::Graph graph;
  graph.setLabel("Force");
  graph.setUnitX("s");
  graph.setUnitY("N");
  graph.setLabelX("time");
  graph.setLabelY("Force");

  graph.setChannel(1, 2);

  //Value
  PhyphoxBleExperiment::Value myValue;
  myValue.setLabel("Force");
  myValue.setPrecision(2);
  myValue.setUnit("N");
  myValue.setColor("FFFFFF");
  myValue.setChannel(2);

  PhyphoxBleExperiment::Value myValue2;
  myValue2.setLabel("Mass");
  myValue2.setPrecision(1);
  myValue2.setUnit("g");
  myValue2.setColor("FFFFFF");
  myValue2.setChannel(3);

  PhyphoxBleExperiment::Value myValue3;
  myValue3.setLabel("ADC raw value");
  myValue3.setPrecision(2);
  myValue3.setUnit("");
  myValue3.setColor("FFFFFF");
  myValue3.setChannel(4);

  PhyphoxBleExperiment::Value myValue4;
  myValue4.setLabel("Battery Level");
  myValue4.setPrecision(2);
  myValue4.setUnit("V");
  myValue4.setColor("FFFFFF");
  myValue4.setChannel(5);

  firstview.addElement(graph);
  firstview.addElement(myValue);
  secondview.addElement(myValue);
  secondview.addElement(myValue2);
  secondview.addElement(myValue3);
  secondview.addElement(myValue4);

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
    pixels.fill(0xfc9905); // yellow warning LED that button is held
    pixels.setBrightness(5);
    pixels.show();

    zeroError = ADCvalue;  //use button to tare the balance

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

  long currentReading = myScale.getReading();
  ADCvalue = currentReading ;
  mass = (ADCvalue - zeroError) / CalibFactor ;
  force = mass * 0.00981 ;

  // Serial.print("Reading: ");
  // Serial.println(currentReading);

  PhyphoxBLE::write(t, force, mass, ADCvalue, measuredvbat); 

}
