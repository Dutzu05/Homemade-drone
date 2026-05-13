#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

const int POT_PIN = 0;
const int JOYSTICK_X_PIN = 1;
const int JOYSTICK_Y_PIN = 2;
const int ADC_RESOLUTION = 12;
const int ANALOG_FILTER_SAMPLES = 8;
const int ADC_MAX = 4095;
const int JOYSTICK_DEADBAND = 180;
const bool INVERT_JOYSTICK_X = false;
const bool INVERT_JOYSTICK_Y = true;
const uint8_t DRONE_MAC[] = {0xDC, 0xB4, 0xD9, 0x8B, 0x1A, 0xB8};

typedef struct {
  uint16_t potValue;
  int16_t rollInput;
  int16_t pitchInput;
  uint32_t sequence;
} CommandPacket;

uint32_t packetSequence = 0;
int filteredPotValue = 0;
int filteredJoystickX = 0;
int filteredJoystickY = 0;
int joystickCenterX = ADC_MAX / 2;
int joystickCenterY = ADC_MAX / 2;

int readAnalogAveraged(int pin) {
  long sum = 0;
  for (int i = 0; i < ANALOG_FILTER_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(150);
  }
  return (int)(sum / ANALOG_FILTER_SAMPLES);
}

int readJoystickAxis(int filteredValue, int centerValue, bool invertAxis) {
  int delta = filteredValue - centerValue;
  if (invertAxis) {
    delta = -delta;
  }

  if (abs(delta) <= JOYSTICK_DEADBAND) {
    return 0;
  }

  if (delta > 0) {
    return map(delta, JOYSTICK_DEADBAND, ADC_MAX - centerValue, 0, 1000);
  }

  return map(delta, -JOYSTICK_DEADBAND, -centerValue, 0, -1000);
}

void calibrateJoystickCenter() {
  const int samples = 200;
  long sumX = 0;
  long sumY = 0;

  for (int i = 0; i < samples; i++) {
    sumX += analogRead(JOYSTICK_X_PIN);
    sumY += analogRead(JOYSTICK_Y_PIN);
    delay(5);
  }

  joystickCenterX = (int)(sumX / samples);
  joystickCenterY = (int)(sumY / samples);
}

void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("SEND | status=");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

bool setupEspNowPeer() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    return false;
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, DRONE_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    return false;
  }

  return true;
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start) < 4000) {
    delay(10);
  }

  analogReadResolution(ADC_RESOLUTION);
  Serial.println("Keep joystick centered: calibrating...");
  calibrateJoystickCenter();

  filteredPotValue = readAnalogAveraged(POT_PIN);
  filteredJoystickX = readAnalogAveraged(JOYSTICK_X_PIN);
  filteredJoystickY = readAnalogAveraged(JOYSTICK_Y_PIN);

  if (!setupEspNowPeer()) {
    Serial.println("ESP-NOW sender init failed!");
  } else {
    Serial.println("ESP-NOW sender ready.");
  }
}

void loop() {
  int rawPotValue = readAnalogAveraged(POT_PIN);
  int rawJoystickX = readAnalogAveraged(JOYSTICK_X_PIN);
  int rawJoystickY = readAnalogAveraged(JOYSTICK_Y_PIN);
  filteredPotValue = (filteredPotValue * 3 + rawPotValue) / 4;
  filteredJoystickX = (filteredJoystickX * 3 + rawJoystickX) / 4;
  filteredJoystickY = (filteredJoystickY * 3 + rawJoystickY) / 4;

  int rollInput = constrain(
    readJoystickAxis(filteredJoystickX, joystickCenterX, INVERT_JOYSTICK_X),
    -1000,
    1000
  );
  int pitchInput = constrain(
    readJoystickAxis(filteredJoystickY, joystickCenterY, INVERT_JOYSTICK_Y),
    -1000,
    1000
  );

  CommandPacket packet;
  packet.potValue = (uint16_t)constrain(filteredPotValue, 0, 4095);
  packet.rollInput = (int16_t)rollInput;
  packet.pitchInput = (int16_t)pitchInput;
  packet.sequence = packetSequence++;

  esp_err_t result = esp_now_send(DRONE_MAC, (uint8_t*)&packet, sizeof(packet));

  Serial.print("POT | raw=");
  Serial.print(rawPotValue);
  Serial.print(" | filt=");
  Serial.print(filteredPotValue);
  Serial.print(" || JOY_X | raw=");
  Serial.print(rawJoystickX);
  Serial.print(" | filt=");
  Serial.print(filteredJoystickX);
  Serial.print(" | ctr=");
  Serial.print(joystickCenterX);
  Serial.print(" | cmd=");
  Serial.print(rollInput);
  Serial.print(" || JOY_Y | raw=");
  Serial.print(rawJoystickY);
  Serial.print(" | filt=");
  Serial.print(filteredJoystickY);
  Serial.print(" | ctr=");
  Serial.print(joystickCenterY);
  Serial.print(" | cmd=");
  Serial.print(pitchInput);
  Serial.print(" | seq=");
  Serial.print(packet.sequence);
  Serial.print(" | send=");
  Serial.println(result == ESP_OK ? "QUEUED" : "ERR");

  delay(50);
}
