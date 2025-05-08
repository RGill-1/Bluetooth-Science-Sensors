#include <phyphoxBle.h>
#include <Adafruit_NeoPixel.h>

#define NUMPIXELS        1
#define LED_BUILTIN 13
#define SENSORPIN 22 //IR breakbeam that must break first, screen side
#define SENSORPIN2 20
double len = 38000; //distance between IR sensors

Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

long leading_edge1;
long leading_edge2;
long falling_time1;
long falling_time2;
long elapsed_time;
long elapsed_time2;
long falling_edgeavg;
long leading_edgeavg;

float velocity; // Speed calculated of leading edge
float velocity2; // Speed calculated of falling edge
float accel; // acceleration between gates

// variables will change:
int sensorState = 0, lastState = 0;       // variable for reading the pushbutton status
int sensorState2 = 0, lastState2 = 0;       // variable for reading the pushbutton status
int hasbroken1 = 0;
int hasbroken2 = 0;

void setup() {
  // initialize the LED pin as an output:
  pinMode(LED_BUILTIN, OUTPUT);
  // initialize the sensor pins as input:
  pinMode(SENSORPIN, INPUT);
  digitalWrite(SENSORPIN, HIGH); // turn on the pullup
  pinMode(SENSORPIN2, INPUT);
  digitalWrite(SENSORPIN2, HIGH); // turn on the pullup

  Serial.begin(9600);

  //####### phyphox experiment setup #####################################

  PhyphoxBLE::start("Lightgate");
  PhyphoxBleExperiment experiment;

  experiment.setTitle("Lightgate");
  experiment.setCategory("Arduino Experiments");
  experiment.setDescription("Velocity and Acceleration");

  //View
  PhyphoxBleExperiment::View firstview;
  firstview.setLabel("Values");  //Create a "view"

  //Value
  PhyphoxBleExperiment::Value myValue;
  myValue.setLabel("Velocity - rising edge");
  myValue.setPrecision(2);
  myValue.setUnit("ms-1");
  myValue.setColor("FFFFFF");
  myValue.setChannel(1);

  PhyphoxBleExperiment::Value myValue2;
  myValue2.setLabel("Velocity - falling edge");
  myValue2.setPrecision(2);
  myValue2.setUnit("ms-1");
  myValue2.setColor("FFFFFF");
  myValue2.setChannel(2);

  PhyphoxBleExperiment::Value myValue3;
  myValue3.setLabel("Accel");
  myValue3.setPrecision(2);
  myValue3.setUnit("ms-2");
  myValue3.setColor("FFFFFF");
  myValue3.setChannel(3);

  //Populate Views
  firstview.addElement(myValue);
  firstview.addElement(myValue2);
  firstview.addElement(myValue3);
  experiment.addView(firstview);

  PhyphoxBLE::addExperiment(experiment);

}

// Function to determine speed
void speed()
{
  elapsed_time = ((leading_edge2 - leading_edge1));
  elapsed_time2 = ((falling_time2 - falling_time1));

  velocity = (len / elapsed_time);
  velocity2 = (len / elapsed_time2);

  falling_edgeavg = ((falling_time2 + falling_time1) / 2);
  leading_edgeavg = ((leading_edge2 + leading_edge1) / 2);

  accel = (((velocity2 - velocity) * 1000000) / (falling_edgeavg - leading_edgeavg));

  Serial.print(velocity);
  Serial.println(" m/s - leading");

  Serial.print(velocity2);
  Serial.println(" m/s - falling");

  Serial.print(accel);
  Serial.println(" m/s^2 - accel");

  PhyphoxBLE::write(velocity, velocity2, accel);

}


void loop() {
  // read the state of the pushbutton value:
  sensorState = digitalRead(SENSORPIN);

  // check if the sensor beam is broken, if it is, the sensorState is LOW:

  if (sensorState && !lastState) {
    if (hasbroken1 == 1) {
      falling_time1 = micros();
    }
  }
  if (!sensorState && lastState) {
    leading_edge1 = micros();
    digitalWrite(LED_BUILTIN, HIGH);
    hasbroken1 = 1;
  }

  sensorState2 = digitalRead(SENSORPIN2);
  if (sensorState2 && !lastState2) {

    if (hasbroken2 == 1) {
      falling_time2 = micros();
      digitalWrite(LED_BUILTIN, LOW);
      speed();
      hasbroken2 = 0;
      hasbroken1 = 0;
    }
  }
  if (!sensorState2 && lastState2) {

    if (hasbroken1 == 1) {
      leading_edge2 = micros();
      hasbroken2 = 1;
    }

  }

  lastState = sensorState;
  lastState2 = sensorState2;
}
