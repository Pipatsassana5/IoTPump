/*
  Project: IoT Water Pressure Control System
  Developer: PIPAT INTEGRATION
*/


#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>

// --- Configuration ---
const char* ssid = "Wokwi-GUEST";
const char* password = "";
String gasUrl = "YOUR_GOOGLE_WEB_APP_URL"; 

// --- Pins ---
const int SENSOR_PIN = 32;
const int PUMP_RELAY = 25;
const int VALVE_RELAY = 26;

// --- Variables & Settings ---
Preferences preferences;
float maxPressureRef = 0.0;     // ค่า 100% (อ้างอิง)
int upperThresholdPct = 60;     // ดึงจาก Google Sheets (เริ่มต้น 60%)
int lowerThresholdPct = 40;     // ดึงจาก Google Sheets (เริ่มต้น 40%)

// --- Timers (Non-blocking) ---
unsigned long lastGasUpdate = 0;
unsigned long lastSensorRead = 0;
unsigned long noUsageTimerStart = 0;
const long updateInterval = 3000; // ส่งข้อมูลทุก 3 วินาที

unsigned long lastWaterUsageTime = 0;       // เวลาที่มีการใช้น้ำครั้งล่าสุด
unsigned long calibrationStartTime = 0;     // เวลาเริ่ม Calibration
unsigned long lastPressureIncreaseTime = 0; // เวลาที่แรงดันเพิ่มขึ้นล่าสุดตอน Calibrate
const unsigned long NO_USAGE_DELAY = 5000; // 10 นาที (600,000 ms) สำหรับทดสอบอาจจะปรับลดลงก่อน
float lastLeakCheckPressure = 0.0;
unsigned long lastLeakCheckTime = 0;



// --- States ---
bool pumpStatus = false;
bool valveStatus = false;
bool isWaterBeingUsed = false;

bool isCalibrating = false;
float tempMaxPressure = 0.0;
bool isLeakDetected = false;
const float V_REF = 3.3;             // แรงดันอ้างอิงของ ESP32 (3.3V)
const int ADC_RESOLUTION = 4095;     // ความละเอียด 12-bit ของ ADC บน ESP32
const float SENSOR_MAX_BAR = 12.0;   // สเปกแรงดันน้ำสูงสุดของเซนเซอร์ (เช่น 1.2 MPa = 12 Bar)

// อัตราส่วนของ Voltage Divider (ตัวอย่าง: R1=10k, R2=20k จะได้ Vout = Vin * (20/30))
// ดังนั้นตัวคูณเพื่อแปลงกลับไปเป็น Voltage จริงของเซนเซอร์คือ 1.5
const float VOLTAGE_MULTIPLIER = 1.5; 

float readPressureSensor() {
  // 1. อ่านค่าดิบจากขา Analog (ค่าจะอยู่ระหว่าง 0 - 4095)
  int rawADC = analogRead(SENSOR_PIN);

  // 2. แปลงค่าดิบเป็นแรงดันไฟฟ้า (Voltage) ที่ตกคร่อมขา ESP32
  float pinVoltage = ((float)rawADC / ADC_RESOLUTION) * V_REF;

  // 3. คำนวณกลับเป็นแรงดันไฟฟ้าจริงที่เซนเซอร์ส่งออกมา (ชดเชย Voltage Divider)
  float sensorVoltage = pinVoltage * VOLTAGE_MULTIPLIER;

  // 4. เช็คความผิดปกติของเซนเซอร์ (Rule 1: เช็คสายขาด หรือ เซนเซอร์พัง)
  // ปกติที่ 0 Bar เซนเซอร์จะส่งไฟออกมา 0.5V
  // ถ้าน้อยกว่า 0.3V แปลว่าสายสัญญาณอาจจะขาด หรือไม่มีไฟเลี้ยงเซนเซอร์
  if (sensorVoltage < 0.3) {
    Serial.println("Error: Sensor Disconnected or No Power!");
    return -1.0; // ส่งค่า -1 เพื่อให้ระบบรู้ว่า Error
  } 
  // ถ้ามากกว่า 4.8V อาจเกิดการช็อตในวงจร
  else if (sensorVoltage > 4.8) {
    Serial.println("Error: Sensor Short Circuit!");
    return -1.0; // ส่งค่า -1 เพื่อให้ระบบรู้ว่า Error
  }

  // 5. คำนวณแปลงแรงดันไฟฟ้าเป็นแรงดันน้ำ (Bar)
  // สมการ: (Voltage ปัจจุบัน - Voltage เริ่มต้น) * (Max Pressure / ช่วง Voltage ที่แกว่ง 4.5V - 0.5V)
  float pressureBar = (sensorVoltage - 0.5) * (SENSOR_MAX_BAR / 4.0);

  // 6. ป้องกันค่าติดลบ (บางครั้งที่ 0 Bar แรงดันไฟอาจแกว่งนิดหน่อยเหลือ 0.48V ทำให้ค่าติดลบ)
  if (pressureBar < 0.0) {
    pressureBar = 0.0;
  }

  return pressureBar;
}

void setup() {
  Serial.begin(115200);
  pinMode(PUMP_RELAY, OUTPUT);
  pinMode(VALVE_RELAY, OUTPUT);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  
  // โหลดค่า Calibration จาก Memory
  preferences.begin("system_vars", false);
  maxPressureRef = preferences.getFloat("max_pressure", 0.0);
  
  // ดึงค่า Setting เริ่มต้นจาก Google Sheets ทันทีที่เปิดเครื่อง
  fetchSettingsFromGAS(); 
}

void loop() {
  unsigned long currentMillis = millis();

  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'c' || cmd == 'C') {
      startAutoCalibration(); // พิมพ์ 'c' เพื่อจำลองการกดปุ่ม Calibrate บนแอป
    }
    // เคลียร์ buffer ป้องกันการอ่านซ้ำ
    while(Serial.available() > 0) Serial.read(); 
  }

  if (currentMillis - lastSensorRead >= 100) { // อ่านเซนเซอร์ทุก 100ms
    float currentPressure = readPressureSensor();
    
    // เช็คเซนเซอร์เสีย/ค้าง (Rule 1)
    if (currentPressure < 0 || isnan(currentPressure)) {
      Serial.println("ERROR: Sensor Fault!");
      digitalWrite(PUMP_RELAY, LOW); 
      digitalWrite(VALVE_RELAY, LOW);
      pumpStatus = false;
      valveStatus = false;
      // ส่งแจ้งเตือน
    } else {
      // ประมวลผลลอจิก
      if (isCalibrating) {
        runAutoCalibration(currentPressure, currentMillis);
      } else {
        processControlLogic(currentPressure, currentMillis);
      }
    }
    lastSensorRead = currentMillis;
  }

  // ส่วนของการส่งข้อมูลขึ้น GAS ให้เพิ่มสถานะ isLeakDetected เข้าไปด้วย
  if (currentMillis - lastGasUpdate >= updateInterval) {
    sendDataToGAS(readPressureSensor(), pumpStatus, valveStatus, isLeakDetected);
    printDebugStatus(readPressureSensor());
    lastGasUpdate = currentMillis;
    
  }
}

// ==========================================
// ส่วนฟังก์ชันย่อย (Implementation)
// ==========================================

void processControlLogic(float currentPressure, unsigned long currentMillis) {
  // หากกำลัง Calibrate อยู่ หรือยังไม่เคย Calibrate ให้ข้ามลอจิกนี้ไปก่อน
  if (isCalibrating || maxPressureRef <= 0.0) return;

  // คำนวณเปอร์เซ็นต์แรงดันปัจจุบันเทียบกับค่าสูงสุดที่เรียนรู้มา
  float currentPct = (currentPressure / maxPressureRef) * 100.0;

  // -----------------------------------------------------------------
  // 1. ตรวจจับการใช้น้ำ (Rule 8)
  // หากแรงดันตกลงต่ำกว่าเกณฑ์การใช้งาน (เช่น ต่ำกว่า 40%) ถือว่ามีการเปิดใช้น้ำ
  // -----------------------------------------------------------------
  if (currentPct < lowerThresholdPct) {
    isWaterBeingUsed = true;
    lastWaterUsageTime = currentMillis; // รีเซ็ตตัวจับเวลา 10-15 นาที
    isLeakDetected = false;             // เคลียร์สถานะน้ำรั่วเพราะเป็นการจงใจใช้
    
    if (!pumpStatus) {
      digitalWrite(PUMP_RELAY, HIGH);   // สั่งเปิดปั๊มเพื่อรักษาแรงดัน
      pumpStatus = true;
      Serial.println("Water usage detected: PUMP ON");
    }
    if (valveStatus) {
      digitalWrite(VALVE_RELAY, LOW);   // ปิดวาล์วทิ้งน้ำ (ถ้าเปิดค้างอยู่)
      valveStatus = false;
    }
  } 
  // หากแรงดันกลับมาเต็ม (ใกล้ 100%) แปลว่าหยุดใช้น้ำแล้ว ให้ปิดปั๊ม
  else if (currentPct >= 98.0 && isWaterBeingUsed) {
    isWaterBeingUsed = false;
    digitalWrite(PUMP_RELAY, LOW);
    pumpStatus = false;
    Serial.println("Water usage stopped: PUMP OFF");
  }

  // -----------------------------------------------------------------
  // 2. ลอจิกควบคุมแรงดันเมื่อไม่มีการใช้น้ำ (Rule 4, 5, 6)
  // -----------------------------------------------------------------
  if (!isWaterBeingUsed && (currentMillis - lastWaterUsageTime >= NO_USAGE_DELAY)) {
    
    // ข้อ 5: แรงดัน >= 60% สั่งปิดปั๊ม เปิดโซลินอยด์วาล์วเพื่อระบายแรงดัน
    if (currentPct >= upperThresholdPct && !valveStatus) {
      digitalWrite(PUMP_RELAY, LOW);
      pumpStatus = false;
      digitalWrite(VALVE_RELAY, HIGH);
      valveStatus = true;
      Serial.println("High Pressure Standby: Valve OPEN to reduce pressure");
    }
    
    // ข้อ 6: แรงดันลดลงมาอยู่ในช่วง 40-60% สั่งปิดโซลินอยด์วาล์วเพื่อรักษาระดับ
    else if (currentPct < upperThresholdPct && currentPct >= lowerThresholdPct && valveStatus) {
      digitalWrite(VALVE_RELAY, LOW);
      valveStatus = false;
      Serial.println("Pressure stabilized in range: Valve CLOSED");
    }
  }

  // -----------------------------------------------------------------
  // 3. ระบบตรวจสอบการรั่วไหล (Rule 7)
  // -----------------------------------------------------------------
  // จะตรวจเช็คก็ต่อเมื่อระบบอยู่นิ่งๆ (ปั๊มปิด, วาล์วปิด, ไม่มีการใช้น้ำ)
  if (!pumpStatus && !valveStatus && !isWaterBeingUsed) {
    
    // เช็คการเปลี่ยนแปลงแรงดันทุกๆ 1 นาที (60000 ms)
    if (currentMillis - lastLeakCheckTime >= 60000) {
      float pressureDrop = lastLeakCheckPressure - currentPressure;
      
      // ถ้าระบบนิ่ง แต่แรงดันลดลงเรื่อยๆ (เช่น ตกมากกว่า 0.1 bar/นาที - ต้องปรับจูนค่านี้หน้างานจริง)
      if (pressureDrop > 0.1 && lastLeakCheckPressure > 0) {
        isLeakDetected = true;
        Serial.println("ALERT: Possible water leak detected!");
        // สถานะ isLeakDetected นี้ จะถูกดึงไปแพ็คส่งขึ้น Google Apps Script แจ้งเตือนแอป
      }
      
      lastLeakCheckPressure = currentPressure;
      lastLeakCheckTime = currentMillis;
    }
  } else {
    // ถ้าระบบมีการทำงาน ให้รีเซ็ตค่าเช็ครั่วไหล
    lastLeakCheckPressure = currentPressure;
    lastLeakCheckTime = currentMillis;
  }
}


void autoCalibration(float detectedMaxPressure) {
  // ลอจิกข้อ 2: เรียนรู้และเผื่อค่า +/- 2%
  maxPressureRef = detectedMaxPressure * 1.02; // เผื่อ 2%
  preferences.putFloat("max_pressure", maxPressureRef);
}

void sendDataToGAS(float pressure, bool pump, bool valve, bool leak) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    // ใช้ GET หรือ POST ตามที่ออกแบบไว้ใน Code.gs
    // ตัวอย่างการแนบ Parameter แบบ GET
    String url = gasUrl + "?action=update&pressure=" + String(pressure) + 
                 "&pump=" + String(pump) + "&valve=" + String(valve) + "&leak=" + String(leak);
    http.begin(url);
    int httpCode = http.GET();
    http.end();
  }
}

void fetchSettingsFromGAS() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = gasUrl + "?action=getSettings";
    http.begin(url);
    int httpCode = http.GET();
    if (httpCode > 0) {
      String payload = http.getString();
      // Parse JSON หรือ String ที่ได้มาเพื่ออัปเดตค่า 
      // upperThresholdPct และ lowerThresholdPct
    }
    http.end();
  }
}

// ฟังก์ชันนี้ถูกเรียกเมื่อต้องการเริ่มการ Calibrate (เช่น สั่งจาก Web App หรือกดปุ่ม)
void startAutoCalibration() {
  Serial.println("--- Starting Auto Calibration ---");
  isCalibrating = true;
  tempMaxPressure = 0.0;
  calibrationStartTime = millis();
  lastPressureIncreaseTime = millis();
  
  // ปิดวาล์วและเปิดปั๊มเพื่ออัดแรงดันให้สุด
  digitalWrite(VALVE_RELAY, LOW); 
  valveStatus = false;
  digitalWrite(PUMP_RELAY, HIGH); 
  pumpStatus = true;
}

// ฟังก์ชันนี้จะถูกเรียกวนใน loop() ตลอดเวลาที่ isCalibrating == true
void runAutoCalibration(float currentPressure, unsigned long currentMillis) {
  // หาค่าแรงดันสูงสุดที่ปั๊มทำได้
  if (currentPressure > tempMaxPressure) {
    tempMaxPressure = currentPressure;
    lastPressureIncreaseTime = currentMillis; // รีเซ็ตเวลาเมื่อแรงดันยังพุ่งขึ้น
  }

  // เงื่อนไขจบการทำงาน: ถ้าแรงดันไม่เพิ่มขึ้นเลยเป็นเวลา 5 วินาที หรือทำงานเกิน 30 วินาที (ป้องกันปั๊มไหม้)
  if ((currentMillis - lastPressureIncreaseTime > 5000) || 
      (currentMillis - calibrationStartTime > 30000)) {
    
    // ข้อ 2: กำหนดให้ค่าแรงดันสูงสุดที่วัดได้เป็น 100% และเผื่อค่า (Margin) +2%
    maxPressureRef = tempMaxPressure * 1.02; 
    
    // บันทึกลง Memory ถาวร (Preferences)
    preferences.putFloat("max_pressure", maxPressureRef);
    
    // จบกระบวนการ ปิดปั๊ม
    isCalibrating = false;
    digitalWrite(PUMP_RELAY, LOW);
    pumpStatus = false;
    
    Serial.println("Calibration Complete!");
    Serial.print("New Max Pressure Ref (100%): ");
    Serial.println(maxPressureRef);
  }
}

void printDebugStatus(float currentPressure) {
  Serial.print("Pressure: "); 
  Serial.print(currentPressure, 2); // แสดงทศนิยม 2 ตำแหน่ง
  Serial.print(" Bar | ");
  
  if (maxPressureRef > 0) {
    float pct = (currentPressure / maxPressureRef) * 100.0;
    Serial.print(pct, 0); Serial.print("% | ");
  } else {
    Serial.print("No Ref | ");
  }

  Serial.print("PUMP: "); Serial.print(pumpStatus ? "ON " : "OFF");
  Serial.print(" | VALVE: "); Serial.print(valveStatus ? "OPEN  " : "CLOSE");
  
  Serial.print(" | STATE: ");
  if (isCalibrating) {
    Serial.print("Calibrating...");
  } else if (isWaterBeingUsed) {
    Serial.print("Using Water");
  } else if (isLeakDetected) {
    Serial.print("!!! LEAK DETECTED !!!");
  } else {
    Serial.print("Standby");
  }
  Serial.println();
}