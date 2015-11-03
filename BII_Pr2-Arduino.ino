/***************************************************************************
  External libraries (copied for comodity):
    * https://github.com/adafruit/Adafruit_Sensor
    * https://github.com/adafruit/Adafruit_BME280_Library (slightly changed)
 **************************************************************************/

// White
#define BUTTON_PIN 2

// Green
#define LIGHTSENSOR_PIN 3

// Serial speed
#define SERIAL_BAUDRATE 115200

// BME280 API
// I2C Library (SCL = Orange, SDA = Yellow)
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BME280.h"


// Struct carrying BME280 data
struct Bme280Data {
  float humidity;
  float pressure;
  float temperature;
};


// Global variables
int buttonState = 0;
int light = 0;
struct Bme280Data bme280Data;


// Sensor driver for the BME280
Adafruit_BME280 bme;


void setup() {
  // Init serial
  Serial.begin(SERIAL_BAUDRATE);

  // Init button with internal pull-up
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH);

  // Light sensor pin needs no init

  // Init BME280 driver
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  };
}

void loop() {
  // Read data
  readButtonState();
  readLightsensor();
  readHumPressTemp();

  // Send PushButton state
  Serial.print("b\t");Serial.println(buttonState);

  // Send Lightsensor data
  Serial.print("l\t");Serial.println(light);

  // Send BME280 data
  Serial.print("h\t");Serial.println(bme280Data.humidity);
  Serial.print("p\t");Serial.println(bme280Data.pressure);
  Serial.print("t\t");Serial.println(bme280Data.temperature);

  // Since we are running at 24 FPS, Frametime is ~40msec
  delay(20);

}

void readButtonState() {
  buttonState = digitalRead(BUTTON_PIN);
}

void readLightsensor() {
  light = analogRead(LIGHTSENSOR_PIN);
}

void readHumPressTemp() {
  bme280Data.humidity = bme.readHumidity();
  // Read and convert to hPa
  bme280Data.pressure = bme.readPressure() / 100;
  bme280Data.temperature = bme.readTemperature();
}



