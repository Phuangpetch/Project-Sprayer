/*
 * NodeMCU (ESP8266) - Blynk + RC Receiver รวมกัน
 * หน้าที่: รับคำสั่งได้ทั้งจาก Blynk (ผ่าน WiFi) และจากตัวรับ RC (ต่อตรงกับ NodeMCU)
 *          แล้วส่งผลลัพธ์สุดท้ายไปหา UNO ผ่าน SoftwareSerial ทีเดียว
 *
 * กติกาการเลือกว่าใครคุม (priority):
 *   - ถ้าคันโยก RC ถูกขยับออกจากตำแหน่งกึ่งกลาง (ช่องใดช่องหนึ่ง) -> ใช้คำสั่งจาก RC ทันที
 *   - ถ้าคันโยก RC นิ่งอยู่กึ่งกลางหมดทุกช่อง -> กลับไปใช้คำสั่งจาก Blynk แทน
 *   เหตุผล: TX ตัวนี้มีแค่ 4 ช่อง ไม่มีสวิตช์แยกไว้เลือกโหมด การให้ RC "ชนะทันทีที่ขยับ"
 *   ทำให้คว้า RC มาคุมได้ทันทีโดยไม่ต้องกดอะไรก่อน ปลอดภัยกว่าเวลาอยู่หน้าเครื่อง
 *
 * การต่อสาย RC (เหมือนเดิม):
 *   CH1 (หมุนหัวพ่น)       -> NodeMCU D1 (GPIO5)
 *   CH2 (ยกหัวพ่น)         -> NodeMCU D2 (GPIO4)
 *   CH3 (เดินหน้า/ถอยหลัง) -> NodeMCU D7 (GPIO13)
 *   CH4 (เลี้ยวซ้าย/ขวา)   -> NodeMCU D8 (GPIO15)
 *   ** D8/GPIO15 เป็น boot-strap pin ของ ESP8266 ต้องเป็น LOW ตอนบูต ถ้าเจอบูตไม่ติด
 *      บางครั้งให้ลองย้าย CH4 ไปขาอื่น **
 *
 * Serial ไป UNO: NodeMCU D5(RX, ไม่ได้ใช้) / D6(TX) -> UNO pin 12(RX)
 * GND ทุกตัว (NodeMCU, UNO, ตัวรับ RC, แหล่งจ่ายมอเตอร์) ต้องต่อร่วมกันหมด
 */

#define BLYNK_TEMPLATE_ID   "TMPL6m7Yk8Px4"
#define BLYNK_TEMPLATE_NAME "Control"
#define BLYNK_AUTH_TOKEN    "ym_nTYgitjdMzbaswsQB-xUZ4kQ-sK7z"
#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>

char ssid[] = "internet";
char pass[] = "123456789";

// ---------- Serial ไป UNO ----------
SoftwareSerial toUno(D5, D6); // (RX ไม่ได้ใช้, TX)

// ---------- ขาสัญญาณ RC ----------
const int PIN_CH1 = D1;
const int PIN_CH2 = D2;
const int PIN_CH3 = D7;
const int PIN_CH4 = D8;

volatile uint32_t timerStart[4] = {0, 0, 0, 0};
volatile uint16_t rcValue[4]    = {1500, 1500, 1500, 1500};

const uint16_t TH_HIGH = 1650;
const uint16_t TH_LOW  = 1350;

void IRAM_ATTR isrCH1() {
  if (digitalRead(PIN_CH1) == HIGH) timerStart[0] = micros();
  else if (timerStart[0] > 0)       rcValue[0] = micros() - timerStart[0];
}
void IRAM_ATTR isrCH2() {
  if (digitalRead(PIN_CH2) == HIGH) timerStart[1] = micros();
  else if (timerStart[1] > 0)       rcValue[1] = micros() - timerStart[1];
}
void IRAM_ATTR isrCH3() {
  if (digitalRead(PIN_CH3) == HIGH) timerStart[2] = micros();
  else if (timerStart[2] > 0)       rcValue[2] = micros() - timerStart[2];
}
void IRAM_ATTR isrCH4() {
  if (digitalRead(PIN_CH4) == HIGH) timerStart[3] = micros();
  else if (timerStart[3] > 0)       rcValue[3] = micros() - timerStart[3];
}

// ---------- สถานะจาก Blynk (bit0..bit7 = V1..V8) ----------
byte blynkState = 0;

void setBlynkBit(int bitIndex, int value) {
  if (value) blynkState |= (1 << bitIndex);
  else       blynkState &= ~(1 << bitIndex);
}

BLYNK_WRITE(V1) { setBlynkBit(0, param.asInt()); }
BLYNK_WRITE(V2) { setBlynkBit(1, param.asInt()); }
BLYNK_WRITE(V3) { setBlynkBit(2, param.asInt()); }
BLYNK_WRITE(V4) { setBlynkBit(3, param.asInt()); }
BLYNK_WRITE(V5) { setBlynkBit(4, param.asInt()); }
BLYNK_WRITE(V6) { setBlynkBit(5, param.asInt()); }
BLYNK_WRITE(V7) { setBlynkBit(6, param.asInt()); }
BLYNK_WRITE(V8) { setBlynkBit(7, param.asInt()); }

BLYNK_CONNECTED() {
  Serial.println(">>> Blynk Connected! <<<");
}

// ---------- ส่งค่าสุดท้ายไป UNO ----------
byte lastSentState = 0xFF; // บังคับส่งครั้งแรก
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 200; // ms

void sendStateToUno(byte state) {
  toUno.write(0xFF);
  toUno.write(state);
  lastSentState = state;
}

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println("=== NodeMCU booting (Blynk + RC) ===");

  toUno.begin(9600);

  pinMode(PIN_CH1, INPUT);
  pinMode(PIN_CH2, INPUT);
  pinMode(PIN_CH3, INPUT);
  pinMode(PIN_CH4, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_CH1), isrCH1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CH2), isrCH2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CH3), isrCH3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CH4), isrCH4, CHANGE);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

void loop() {
  Blynk.run();

  // ---- อ่านค่า RC ----
  noInterrupts();
  uint16_t ch1 = rcValue[0];
  uint16_t ch2 = rcValue[1];
  uint16_t ch3 = rcValue[2];
  uint16_t ch4 = rcValue[3];
  interrupts();

  byte rcState = 0;
  if (ch3 > TH_HIGH) rcState |= (1 << 0); // Forward
  if (ch3 < TH_LOW)  rcState |= (1 << 1); // Backward
  if (ch4 < TH_LOW)  rcState |= (1 << 2); // Turn_Left
  if (ch4 > TH_HIGH) rcState |= (1 << 3); // Turn_Right
  if (ch2 > TH_HIGH) rcState |= (1 << 4); // Sprayer_Up
  if (ch2 < TH_LOW)  rcState |= (1 << 5); // Sprayer_Down
  if (ch1 > TH_HIGH) rcState |= (1 << 6); // Rotate_Left
  if (ch1 < TH_LOW)  rcState |= (1 << 7); // Rotate_Right

  // ---- เลือกว่าใครคุม: RC ขยับ (rcState != 0) ให้ RC ชนะ ไม่งั้นใช้ Blynk ----
  byte finalState = (rcState != 0) ? rcState : blynkState;

  // ---- ส่งไป UNO เมื่อค่าเปลี่ยน หรือครบรอบ heartbeat ----
  if (finalState != lastSentState || (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL)) {
    sendStateToUno(finalState);
    lastHeartbeat = millis();

    Serial.print(rcState != 0 ? "[RC]    " : "[Blynk] ");
    Serial.print("CH1-4: "); Serial.print(ch1); Serial.print(",");
    Serial.print(ch2); Serial.print(","); Serial.print(ch3); Serial.print(",");
    Serial.print(ch4);
    Serial.print("  buttonState = ");
    Serial.println(finalState, BIN);
  }
}
