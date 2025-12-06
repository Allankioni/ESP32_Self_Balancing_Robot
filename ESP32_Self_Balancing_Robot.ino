#include <WiFi.h>
#include <WiFiManager.h>  // tzapu WiFiManager
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <EEPROM.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// --- Motor Pins ---
#define A_IN1 27
#define A_IN2 14
#define B_IN3 26
#define B_IN4 12
#define INTERRUPT_PIN 2
#define LED_PIN 13

// --- EEPROM ---
#define EEPROM_SIZE 16
#define ADDR_KP 0
#define ADDR_KI 4
#define ADDR_KD 8
#define ADDR_SETPOINT 12

// --- PID ---
float kp, ki, kd;
float setpoint = 0.0;
float integral = 0.0, previousError = 0.0;

// --- Motor State ---
int wheelSpeed = 0, wheelDir = 1;

// --- MPU6050 ---
MPU6050 mpu;
bool dmpReady = false;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
unsigned long prevTimePID = 0;

// --- Web Server ---
AsyncWebServer server(80);

// --- Tuning State ---
float tuneParams[3];
float deltas[3] = {1.0, 0.1, 0.1};
float bestError = 999999.0;
int tuningIndex = 0;
bool tuningMode = false;
unsigned long tuningStartTime = 0;
float tuningErrorSum = 0;

// --- Interrupt Flag ---
volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() { mpuInterrupt = true; }

// --- HTML Page ---
const char html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot Monitor</title><style>body{font-family:sans-serif;text-align:center}</style></head><body>
<h2>Orientation</h2><p>Pitch: <span id="pitch">0</span>°</p>
<h2>Motor</h2><p>Speed: <span id="speed">0</span></p>
<h2>PID</h2>
<p>Kp: <span id="kp">0</span>, Ki: <span id="ki">0</span>, Kd: <span id="kd">0</span>, Setpoint: <span id="setpoint">0</span></p>
<input id="kpInput" type="number" step="0.1"><label>Kp</label><br>
<input id="kiInput" type="number" step="0.1"><label>Ki</label><br>
<input id="kdInput" type="number" step="0.1"><label>Kd</label><br>
<input id="setpointInput" type="number" step="0.1"><label>Setpoint</label><br>
<button onclick="updatePID()">Update PID</button>
<hr><button onclick="fetch('/startTune')">Start Tuning</button>
<button onclick="fetch('/tuneStep')">Tuning Step</button>
<script>
function updatePID(){
  fetch(`/setPID?kp=${kpInput.value}&ki=${kiInput.value}&kd=${kdInput.value}&setpoint=${setpointInput.value}`);
}
function updateStatus(){
  fetch("/status").then(r=>r.json()).then(j=>{
    pitch.textContent = j.pitch.toFixed(2);
    speed.textContent = j.speed;
    kp.textContent = j.kp.toFixed(2);
    ki.textContent = j.ki.toFixed(2);
    kd.textContent = j.kd.toFixed(2);
    setpoint.textContent = j.setpoint.toFixed(2);
  });
}
setInterval(updateStatus, 1000);
</script></body></html>)rawliteral";




void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pinMode(LED_PIN, OUTPUT);

  // Ensure motor pins are defined as outputs and forced LOW before any PWM attach
  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);
  pinMode(B_IN3, OUTPUT);
  pinMode(B_IN4, OUTPUT);
  digitalWrite(A_IN1, LOW);
  digitalWrite(A_IN2, LOW);
  digitalWrite(B_IN3, LOW);
  digitalWrite(B_IN4, LOW);
  // PWM
  ledcAttach(A_IN1, 5000, 8);
  ledcAttach(A_IN2, 5000, 8);
  ledcAttach(B_IN3, 5000, 8);
  ledcAttach(B_IN4, 5000, 8);
  // Keep motors OFF after PWM attach
  applyMotor("A", 0, 1);
  applyMotor("B", 0, 1);

  
  // EEPROM Init & Load
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(ADDR_KP, kp);
  EEPROM.get(ADDR_KI, ki);
  EEPROM.get(ADDR_KD, kd);
  EEPROM.get(ADDR_SETPOINT, setpoint);
  if (isnan(kp)) { kp = 20.0; ki = 0.5; kd = 0.1; setpoint = 0; }

  // --- WiFiManager by tzapu ---
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  bool res = wm.autoConnect("ESP32-Robot-Setup", "configure123");
  if (!res) {
    Serial.println("❌ WiFi failed. Restarting...");
    delay(3000);
    ESP.restart();
  }
  Serial.println("✅ Connected to WiFi");
  Serial.println(WiFi.localIP());



  // MPU6050 Init
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  // if (!mpu.testConnection()) {
  //   Serial.println("MPU6050 connection failed."); while (1);
  // }
  if (mpu.dmpInitialize() == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    dmpReady = true;
  }

  // Web Server
  server.on("/", HTTP_GET, [](auto *r){ r->send_P(200, "text/html", html); });
  server.on("/setPID", HTTP_GET, [](auto *r){
    if (r->hasParam("kp")) kp = r->getParam("kp")->value().toFloat();
    if (r->hasParam("ki")) ki = r->getParam("ki")->value().toFloat();
    if (r->hasParam("kd")) kd = r->getParam("kd")->value().toFloat();
    if (r->hasParam("setpoint")) setpoint = r->getParam("setpoint")->value().toFloat();
    EEPROM.put(ADDR_KP, kp); EEPROM.put(ADDR_KI, ki);
    EEPROM.put(ADDR_KD, kd); EEPROM.put(ADDR_SETPOINT, setpoint);
    EEPROM.commit();
    r->send(200, "text/plain", "PID updated");
  });
  server.on("/status", HTTP_GET, [](auto *r){
    float pitchDeg = ypr[1] * 180.0 / M_PI;
    String json = "{\"pitch\":" + String(pitchDeg, 2) +
                  ",\"speed\":" + String(wheelSpeed) +
                  ",\"kp\":" + String(kp) +
                  ",\"ki\":" + String(ki) +
                  ",\"kd\":" + String(kd) +
                  ",\"setpoint\":" + String(setpoint) + "}";
    r->send(200, "application/json", json);
  });
  server.on("/startTune", HTTP_GET, [](auto *r){
    tuningMode = true;
    tuneParams[0] = kp; tuneParams[1] = ki; tuneParams[2] = kd;
    bestError = 999999.0; tuningIndex = 0;
    tuningStartTime = millis(); tuningErrorSum = 0;
    r->send(200, "text/plain", "Tuning started");
  });
  server.on("/tuneStep", HTTP_GET, [](auto *r){
    if (!tuningMode) {
      r->send(400, "text/plain", "Start tuning first");
      return;
    }
    tuneParams[tuningIndex] += deltas[tuningIndex];
    kp = tuneParams[0]; ki = tuneParams[1]; kd = tuneParams[2];
    tuningStartTime = millis(); tuningErrorSum = 0;
    r->send(200, "text/plain", "Tuning step started");
  });

  server.begin();
}

void loop() {
  float gyroPitchRate = 0;
  if (!dmpReady || !mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // Keep motors off until IMU DMP is ready and a packet is available
    applyMotor("A", 0, 1);
    applyMotor("B", 0, 1);
    return;
  }

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  float pitch = ypr[1] * 180.0 / M_PI;

  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);
  gyroPitchRate = gy / 131.0;


  unsigned long now = millis();
  unsigned long delta = now - prevTimePID;
  if (delta == 0) return;
  prevTimePID = now;

  float error = setpoint - pitch;
  integral = constrain(integral + ki * error * delta / 1000.0, -255, 255);
  float derivative = kd * (error - previousError) / (delta / 1000.0);
  float output = kp * error + integral + derivative;
  previousError = error;

  

  int motorSpeed = constrain((int)output, -255, 255);
  wheelSpeed = abs(motorSpeed);
  wheelDir = motorSpeed >= 0 ? 1 : -1;
  applyMotor("A", wheelSpeed, wheelDir);
  applyMotor("B", wheelSpeed, wheelDir);

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  if (tuningMode) {
    // Read gyro rate
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    float gyroPitchRate = gy / 131.0;

    // Penalize both error and shakiness
    tuningErrorSum += abs(pitch - setpoint) + 0.05 * abs(gyroPitchRate);

    if (millis() - tuningStartTime > 2000) {
      if (tuningErrorSum < bestError) {
        bestError = tuningErrorSum;
        deltas[tuningIndex] *= 1.1;
        EEPROM.put(ADDR_KP, kp = tuneParams[0]);
        EEPROM.put(ADDR_KI, ki = tuneParams[1]);
        EEPROM.put(ADDR_KD, kd = tuneParams[2]);
        EEPROM.commit();
        Serial.printf("[✓] PID saved: Kp=%.2f Ki=%.2f Kd=%.2f\n", kp, ki, kd);
      } else {
        tuneParams[tuningIndex] -= 2 * deltas[tuningIndex];
        kp = tuneParams[0]; ki = tuneParams[1]; kd = tuneParams[2];
        deltas[tuningIndex] *= 0.9;
      }
      tuningIndex = (tuningIndex + 1) % 3;
      tuningErrorSum = 0;
      tuningStartTime = millis();
      
    }
  }


  // Plotting: pitch, setpoint, output, and gyro rate
  Serial.print("Pitch:");
  Serial.print(pitch);
  Serial.print(",Setpoint:");
  Serial.print(setpoint);
  Serial.print(",Output:");
  Serial.print(output);
  Serial.print(",Gyro:");
  Serial.println(gyroPitchRate);

}

void applyMotor(String m, int spd, int dir){
  int pin1 = (m == "A") ? A_IN1 : B_IN3;
  int pin2 = (m == "A") ? A_IN2 : B_IN4;
  if (dir > 0) {
    ledcWrite(pin1, spd);
    ledcWrite(pin2, 0);
  } else {
    ledcWrite(pin1, 0);
    ledcWrite(pin2, spd);
  }
}
