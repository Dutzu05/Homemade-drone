#include <Arduino.h>

// Potentiometer input pin
const int POT_PIN = 0;   // GPIO0 / ADC input

// PWM output pins connected to H-bridges
const int PWM_PINS[] = {3, 4, 6, 7};
const int NUM_PWM_PINS = 4;

// PWM channels for ESP32 LEDC
const int PWM_CHANNELS[] = {0, 1, 2, 3};

// PWM settings
const int PWM_FREQ = 20000;       // 20 kHz
const int PWM_RESOLUTION = 8;     // 8-bit PWM: 0 - 255

// ADC settings
const int ADC_RESOLUTION = 12;    // 12-bit ADC: 0 - 4095

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("ESP32-C3 Potentiometer PWM Control");

  // Configure ADC resolution
  analogReadResolution(ADC_RESOLUTION);

  // Configure PWM channels and attach pins
  for (int i = 0; i < NUM_PWM_PINS; i++) {
    ledcSetup(PWM_CHANNELS[i], PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWM_PINS[i], PWM_CHANNELS[i]);
    ledcWrite(PWM_CHANNELS[i], 0);
  }
}

void loop() {
  // Read potentiometer value: 0 - 4095
  int potValue = analogRead(POT_PIN);

  // Convert ADC value to PWM duty cycle: 0 - 255
  int dutyCycle = map(potValue, 0, 4095, 0, 255);
  dutyCycle = constrain(dutyCycle, 0, 255);

  // Send the same PWM signal to all 4 H-bridge inputs
  for (int i = 0; i < NUM_PWM_PINS; i++) {
    ledcWrite(PWM_CHANNELS[i], dutyCycle);
  }

  Serial.print("Potentiometer: ");
  Serial.print(potValue);
  Serial.print(" | PWM duty: ");
  Serial.println(dutyCycle);

  delay(20);
}
