#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

// Pins
const int ecgPin = 34;      // AD8232 OUT
const int loPlusPin = 27;   // AD8232 LO+
const int loMinusPin = 26;  // AD8232 LO-
const int i2cSDA = 21;      // MAX30102 SDA
const int i2cSCL = 22;      // MAX30102 SCL

MAX30105 particleSensor;
WebServer server(80);

// ECG Processing (250Hz)
const int ECG_SAMPLE_RATE = 250;
const float ECG_SAMPLE_INTERVAL = 1000.0/ECG_SAMPLE_RATE;
unsigned long lastEcgTime = 0;

// Enhanced ECG Processing
#define ECG_FILTER_SIZE 7
int ecgRawBuffer[ECG_FILTER_SIZE];
int ecgFilterIndex = 0;
float ecgBaseline = 2048.0;
int filteredECG = 2048;

// MAX30102 Configuration
#define BUFFER_LENGTH 100
uint32_t irBuffer[BUFFER_LENGTH];
uint32_t redBuffer[BUFFER_LENGTH];
int32_t heartRate, spo2;
int8_t validHeartRate, validSPO2;
float temperature = 0.0;
unsigned long lastMax30102Time = 0;
const int MAX30102_INTERVAL = 20;  // 50Hz sampling

// System Status
bool sensorFound = false;
uint32_t lastIrValue = 0;
bool fingerDetected = false;

// Web Dashboard
const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Health Monitor</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <style>
    body { font-family: sans-serif; text-align: center; background: #f4f4f4; margin: 0; padding: 10px; }
    #graph { width: 100%; height: 300px; background: #fff; border: 1px solid #ccc; }
    h1 { font-size: 1.5em; }
    .stats { display: flex; justify-content: space-around; margin: 20px 0; }
    .stat-box { background: white; padding: 15px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); width: 30%; }
    .stat-value { font-size: 1.8em; font-weight: bold; }
    .heart-rate { color: #e74c3c; }
    .spo2 { color: #3498db; }
    .temp { color: #2ecc71; }
    .status { margin-top: 20px; font-size: 0.9em; color: #555; }
    .debug-info { margin-top: 10px; padding: 10px; background: #eee; border-radius: 5px; font-size: 0.8em; text-align: left; display: none; }
    #toggleDebug { margin-top: 10px; padding: 5px 10px; cursor: pointer; }
  </style>
</head>
<body>
  <h1>Health Monitor Dashboard</h1>
  <canvas id="graph"></canvas>
  <div class="stats">
    <div class="stat-box">
      <div>Heart Rate</div>
      <div class="stat-value heart-rate" id="bpm">--</div>
      <div>BPM</div>
    </div>
    <div class="stat-box">
      <div>SpO₂</div>
      <div class="stat-value spo2" id="spo2">--</div>
      <div>%</div>
    </div>
    <div class="stat-box">
      <div>Temperature</div>
      <div class="stat-value temp" id="temp">--</div>
      <div>°C</div>
    </div>
  </div>
  <div class="status" id="status">Initializing...</div>
  <button id="toggleDebug">Show Debug Info</button>
  <div class="debug-info" id="debugInfo">
    <div>Raw ECG Value: <span id="rawEcg">--</span></div>
    <div>IR Value: <span id="irValue">--</span></div>
    <div>Red Value: <span id="redValue">--</span></div>
    <div>Sensor Status: <span id="sensorStatus">--</span></div>
  </div>
  <script>
    const canvas = document.getElementById('graph');
    const ctx = canvas.getContext('2d');
    canvas.width = window.innerWidth - 40;
    canvas.height = 300;
    let ecgData = new Array(canvas.width).fill(150);
    let ecgBaseline = 512;
    let ecgScale = 0.5;

    document.getElementById('toggleDebug').addEventListener('click', function() {
      const debugInfo = document.getElementById('debugInfo');
      if (debugInfo.style.display === 'block') {
        debugInfo.style.display = 'none';
        this.textContent = 'Show Debug Info';
      } else {
        debugInfo.style.display = 'block';
        this.textContent = 'Hide Debug Info';
      }
    });

    function processECGValue(val) {
      const rawVal = parseInt(val);
      document.getElementById('rawEcg').textContent = rawVal;
      ecgBaseline = ecgBaseline * 0.995 + rawVal * 0.005;
      let processed = (rawVal - ecgBaseline) * ecgScale + 150;
      ecgData.push(processed);
      ecgData.shift();
      
      let min = Math.min(...ecgData);
      let max = Math.max(...ecgData);
      let range = max - min;
      if(range > 0) ecgScale = Math.min(0.8, 100/range);
    }

    function drawECG() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      
      // Draw grid
      ctx.strokeStyle = "#eeeeee";
      ctx.lineWidth = 1;
      for (let x = 0; x <= canvas.width; x += 20) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvas.height);
        ctx.stroke();
      }
      for (let y = 0; y <= canvas.height; y += 20) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
        ctx.stroke();
      }
      
      // Draw ECG signal
      ctx.beginPath();
      ctx.moveTo(0, ecgData[0]);
      for (let i = 1; i < ecgData.length; i++) {
        ctx.lineTo(i, ecgData[i]);
      }
      ctx.strokeStyle = "#e74c3c";
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    function fetchECG() {
      fetch('/ecg-data')
      .then(r => r.text())
      .then(val => {
        if (val !== "204") {
          processECGValue(val);
          drawECG();
          document.getElementById("status").textContent = "Monitoring ECG signal";
        } else {
          document.getElementById("status").textContent = "Lead-off detected - check electrodes";
        }
      })
      .catch(e => console.error('Error:', e));
    }

    function fetchVitals() {
      fetch('/vitals')
      .then(r => r.json())
      .then(data => {
        document.getElementById('irValue').textContent = data.ir;
        document.getElementById('redValue').textContent = data.red || "N/A";
        document.getElementById('sensorStatus').textContent = data.sensor_found ? "Connected" : "Not Found";

        if (!data.sensor_found) {
          document.getElementById("status").textContent = "MAX30102 sensor not found";
        } else if (data.ir < 50000) {
          document.getElementById("status").textContent = "Place finger on sensor";
        }

        if (data.hr && data.hr >= 40 && data.hr <= 220) {
          document.getElementById("bpm").textContent = data.hr;
        }
        if (data.spo2 && data.spo2 >= 70 && data.spo2 <= 100) {
          document.getElementById("spo2").textContent = data.spo2;
        }
        if (data.temp && data.temp >= 20 && data.temp <= 50) {
          document.getElementById("temp").textContent = data.temp.toFixed(1);
        }
      })
      .catch(e => console.error('Error:', e));
    }

    setInterval(fetchECG, 20);
    setInterval(fetchVitals, 1000);
  </script>
</body>
</html>
)rawliteral";

void setupWiFi() {
  WiFi.softAP("Health_Monitor", "12345678");
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
}

void setupMAX30102() {
  Wire.begin(i2cSDA, i2cSCL);
  Wire.setClock(400000);
  
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found!");
    sensorFound = false;
    return;
  }
  
  sensorFound = true;
  particleSensor.setup(60, 4, 2, 400, 411, 4096);
  particleSensor.enableDIETEMPRDY();
  
  // Pre-fill buffers
  for (int i = 0; i < BUFFER_LENGTH; i++) {
    while (!particleSensor.available()) particleSensor.check();
    irBuffer[i] = particleSensor.getIR();
    redBuffer[i] = particleSensor.getRed();
    particleSensor.nextSample();
  }
}

float readTemperature() {
  if (!sensorFound) return 0.0;
  float temp = particleSensor.readTemperature();
  return constrain(temp + 1.0, 32.0, 42.0);
}

// Enhanced ECG Processing
void processECG() {
  static float dcOffset = 2048.0;
  static float lastFiltered = 2048.0;
  
  // Oversampling (8x for better noise reduction)
  int raw = 0;
  for(int i=0; i<8; i++) {
    raw += analogRead(ecgPin);
    delayMicroseconds(50);
  }
  raw /= 8;

  // Dynamic baseline tracking
  dcOffset = 0.999 * dcOffset + 0.001 * raw;
  
  // Median filter (remove spikes)
  ecgRawBuffer[ecgFilterIndex] = raw;
  ecgFilterIndex = (ecgFilterIndex + 1) % ECG_FILTER_SIZE;
  
  int temp[ECG_FILTER_SIZE];
  memcpy(temp, ecgRawBuffer, sizeof(temp));
  for(int i=0; i<ECG_FILTER_SIZE-1; i++) {
    for(int j=0; j<ECG_FILTER_SIZE-i-1; j++) {
      if(temp[j] > temp[j+1]) {
        int swap = temp[j];
        temp[j] = temp[j+1];
        temp[j+1] = swap;
      }
    }
  }
  int median = temp[ECG_FILTER_SIZE/2];
  
  // Bandpass filter (0.5Hz - 40Hz)
  static float hpFiltered = 0, lastHp = 0, lastIn = 0;
  float highpassed = 0.996 * (lastHp + median - lastIn);
  lastHp = highpassed;
  lastIn = median;
  
  static float lpFiltered = 0;
  float lowpassed = 0.8 * lpFiltered + 0.2 * highpassed;
  lpFiltered = lowpassed;
  
  // Final output
  filteredECG = constrain((int)(lowpassed * 2.0) + 2048, 0, 4095);
}

void updateMAX30102() {
  static int bufferIndex = 0;
  static unsigned long lastTempRead = 0;
  
  // Read temperature every 2 seconds
  if (millis() - lastTempRead > 2000) {
    temperature = readTemperature();
    lastTempRead = millis();
  }

  // Finger detection
  lastIrValue = particleSensor.getIR();
  fingerDetected = (lastIrValue > 50000);
  
  if (!fingerDetected) {
    validHeartRate = 0;
    validSPO2 = 0;
    return;
  }

  // Circular buffer for calculations
  while (particleSensor.available()) {
    redBuffer[bufferIndex] = particleSensor.getRed();
    irBuffer[bufferIndex] = particleSensor.getIR();
    particleSensor.nextSample();
    
    bufferIndex = (bufferIndex + 1) % BUFFER_LENGTH;
    
    // Calculate every half buffer
    if (bufferIndex % (BUFFER_LENGTH/2) == 0) {
      maxim_heart_rate_and_oxygen_saturation(
        irBuffer, BUFFER_LENGTH, redBuffer,
        &spo2, &validSPO2, &heartRate, &validHeartRate
      );
      
      // Validate readings
      if (heartRate < 40 || heartRate > 220) validHeartRate = 0;
      if (spo2 < 70 || spo2 > 100) validSPO2 = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize ECG
  pinMode(loPlusPin, INPUT);
  pinMode(loMinusPin, INPUT);
  analogReadResolution(12);
  
  // Initialize MAX30102
  setupMAX30102();
  
  // Initialize WiFi
  setupWiFi();
  
  // Web server endpoints
  server.on("/", []() { server.send(200, "text/html", htmlPage); });
  server.on("/ecg-data", []() {
    server.send(200, "text/plain", String(map(filteredECG, 0, 4095, 0, 1023)));
  });
  server.on("/vitals", []() {
    String json = "{";
    json += "\"hr\":" + (validHeartRate ? String(heartRate) : "null");
    json += ",\"spo2\":" + (validSPO2 ? String(spo2) : "null");
    json += ",\"temp\":" + String(temperature, S1);
    json += ",\"ir\":" + String(lastIrValue);
    json += ",\"red\":" + String(particleSensor.getRed());
    json += ",\"sensor_found\":" + String(sensorFound);
    json += "}";
    server.send(200, "application/json", json);
  });
  server.begin();
}

void loop() {
  server.handleClient();
  
  // ECG Sampling (250Hz)
  if (millis() - lastEcgTime >= ECG_SAMPLE_INTERVAL) {
    lastEcgTime = millis();
    if (!digitalRead(loPlusPin) && !digitalRead(loMinusPin)) {
      processECG();
    }
  }
  
  // MAX30102 Sampling (50Hz)
  if (millis() - lastMax30102Time >= MAX30102_INTERVAL) {
    lastMax30102Time = millis();
    if (sensorFound) updateMAX30102();
  }
}