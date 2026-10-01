/*
 * UNO - Motor Controller (รับคำสั่งจาก NodeMCU ผ่าน SoftwareSerial แล้วขับมอเตอร์ 4 ตัว)
 * edit4: เพิ่ม soft-start/soft-stop (ค่อยๆ ไต่ความเร็วขึ้น-ลง) ให้ล้อซ้าย-ขวา และแกนขึ้น-ลง
 *        จำกัดความเร็วสูงสุดไว้ที่ 70% ของสปีดเต็ม
 *        ** แกนหมุน (ROT) ยังเป็น on/off เต็มสปีดเหมือนเดิม เพราะขา A0/A1 ไม่ใช่ขา PWM จริง **
 *
 * ผังขา (ตามที่ตกลงกันไว้):
 * BTS7960 #1 (ล้อซ้าย)      RPWM->3   LPWM->5    R_EN/L_EN -> ต่อ 5V ค้างไว้
 * BTS7960 #2 (ล้อขวา)      RPWM->6   LPWM->9    R_EN/L_EN -> ต่อ 5V ค้างไว้
 * BTS7960 #3 (แกนขึ้น-ลง)   RPWM->10  LPWM->11   R_EN/L_EN -> ต่อ 5V ค้างไว้
 * BTS7960 #4 (แกนหมุน)     RPWM->A0  LPWM->15(A1) R_EN/L_EN -> ต่อ 5V ค้างไว้ (on/off เต็มสปีดเท่านั้น)
 *
 * Serial จาก NodeMCU: UNO RX=12, TX=13
 * อย่าลืม: GND ของ UNO, NodeMCU, และแหล่งจ่ายไฟทุกชุดของ BTS7960 ต้องต่อถึงกันหมด (common ground)
 */

#include <SoftwareSerial.h>

// ---------- มอเตอร์ล้อ (ปรับความเร็วได้ - PWM จริง) ----------
const int LEFT_RPWM  = 3;
const int LEFT_LPWM  = 5;
const int RIGHT_RPWM = 6;
const int RIGHT_LPWM = 9;

// ---------- มอเตอร์แกนขึ้น-ลง (ปรับความเร็วได้ - PWM จริง) ----------
const int LIFT_RPWM = 10;
const int LIFT_LPWM = 11;

// ---------- มอเตอร์แกนหมุน (on/off เต็มสปีดเท่านั้น) ----------
const int ROT_RPWM = A0;
const int ROT_LPWM = 15; // = A1 (เขียนเป็นเลขขาโดยตรง เลี่ยงปัญหา 'A1' was not declared)

// ---------- ความเร็วสูงสุด ----------
const int SPEED_PERCENT = 70;                              // <-- ปรับ % ความเร็วสูงสุดตรงนี้
const int MAX_SPEED = (int)(255 * SPEED_PERCENT / 100.0);  // แปลงเป็นค่า PWM 0-255 (70% = 178)

// ---------- ตั้งค่าการไต่/ผ่อนความเร็ว (soft-start / soft-stop) ----------
const unsigned long RAMP_INTERVAL_MS = 20; // อัปเดตความเร็วทุกกี่ ms
const int RAMP_STEP = 5;                   // เพิ่ม/ลดความเร็วทีละเท่านี้ต่อ 1 อัปเดต
// เวลาที่ใช้ไต่จาก 0 ถึงสปีดสูงสุด โดยประมาณ = (MAX_SPEED / RAMP_STEP) * RAMP_INTERVAL_MS
// ค่าเริ่มต้นนี้ ~712ms (ปรับ RAMP_STEP ให้มากขึ้น = ไต่เร็วขึ้น, น้อยลง = ไต่นุ่มขึ้น)
unsigned long lastRampUpdate = 0;

int leftTargetSpeed = 0,  leftCurrentSpeed  = 0;
int rightTargetSpeed = 0, rightCurrentSpeed = 0;
int liftTargetSpeed = 0,  liftCurrentSpeed  = 0;

// ---------- Serial จาก NodeMCU ----------
SoftwareSerial fromEsp(12, 13); // (RX, TX)

byte buttonState = 0;               // bit0..bit7 = V1..V8
unsigned long lastMsgTime = 0;
const unsigned long LINK_TIMEOUT = 1000; // ms - ถ้าไม่มีข้อความมาเกินนี้ ให้หยุดมอเตอร์ทั้งหมดทันที (failsafe)
bool wasLinkLost = false; // เก็บสถานะไว้ พิมพ์ข้อความแค่ตอน "เปลี่ยน" สถานะ ไม่ใช่ทุก loop

void setup() {
  pinMode(LEFT_RPWM, OUTPUT);  pinMode(LEFT_LPWM, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT); pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(LIFT_RPWM, OUTPUT);  pinMode(LIFT_LPWM, OUTPUT);
  pinMode(ROT_RPWM, OUTPUT);   pinMode(ROT_LPWM, OUTPUT);

  emergencyStopAll();
  fromEsp.begin(9600);
  Serial.begin(9600); // สำหรับดู debug ผ่าน USB Serial Monitor เท่านั้น ไม่ชนกับ SoftwareSerial (12,13)
  Serial.println("UNO พร้อมแล้ว รอรับข้อมูลจาก ESP...");
  lastMsgTime = millis();
}

void loop() {
  // ---- อ่านแพ็กเก็ต: byte นำ 0xFF ตามด้วย byte ข้อมูล ----
  while (fromEsp.available() >= 2) {
    if ((byte)fromEsp.read() == 0xFF) {
      byte newState = fromEsp.read();
      if (newState != buttonState) {           // print เฉพาะตอนค่าเปลี่ยน กันข้อความรัว
        printCommand(newState);
      }
      buttonState = newState;
      lastMsgTime = millis();
    }
  }

  // ---- Failsafe: ลิงก์ขาดนานเกินไป -> หยุดทุกมอเตอร์ทันที (ไม่ผ่อน ต้องหยุดเดี๋ยวนั้นเพื่อความปลอดภัย) ----
  bool linkLost = (millis() - lastMsgTime > LINK_TIMEOUT);
  if (linkLost && !wasLinkLost) {
    Serial.println("[!!!] ลิงก์ขาด -> หยุดมอเตอร์ทั้งหมดทันที");
  } else if (!linkLost && wasLinkLost) {
    Serial.println("[OK] ลิงก์กลับมาแล้ว");
  }
  wasLinkLost = linkLost;

  if (linkLost) {
    emergencyStopAll();
  } else {
    driveWheels();          // กำหนด "เป้าหมาย" ความเร็วล้อจากปุ่มที่กด
    driveLift();             // กำหนด "เป้าหมาย" ความเร็วแกนขึ้น-ลง
    driveRotate();           // แกนหมุน on/off ทันที ไม่มีการไต่
    updateRampAndApply();    // ไต่/ผ่อนความเร็วจริงเข้าหาเป้าหมายทีละนิด แล้วสั่งมอเตอร์
  }
}

bool bitOn(int i) { return buttonState & (1 << i); }

// ---------- พิมพ์คำสั่งที่กำลังทำงานให้อ่านง่าย แทนเลขฐาน 2 ล้วนๆ ----------
void printCommand(byte state) {
  Serial.print("[CMD] ");
  bool any = false;
  if (state & (1 << 0)) { Serial.print("Forward ");      any = true; }
  if (state & (1 << 1)) { Serial.print("Backward ");     any = true; }
  if (state & (1 << 2)) { Serial.print("Turn_Left ");    any = true; }
  if (state & (1 << 3)) { Serial.print("Turn_Right ");   any = true; }
  if (state & (1 << 4)) { Serial.print("Sprayer_Up ");   any = true; }
  if (state & (1 << 5)) { Serial.print("Sprayer_Down "); any = true; }
  if (state & (1 << 6)) { Serial.print("Rotate_Left ");  any = true; }
  if (state & (1 << 7)) { Serial.print("Rotate_Right "); any = true; }
  if (!any) Serial.print("STOP");

  Serial.print(" (0b");
  for (int i = 7; i >= 0; i--) Serial.print((state >> i) & 1);
  Serial.println(")");
}

// ---------- กำหนดเป้าหมายความเร็ว (ยังไม่ได้สั่งมอเตอร์ตรงนี้ แค่ตั้งเป้า) ----------
void driveWheels() {
  bool fwd   = bitOn(0); // V1 Forward
  bool bwd   = bitOn(1); // V2 Backward
  bool left  = bitOn(2); // V3 Turn_Left
  bool right = bitOn(3); // V4 Turn_Right

  if (left && !right) {                 // หมุนตัวซ้าย (pivot turn)
    leftTargetSpeed  = -MAX_SPEED;
    rightTargetSpeed =  MAX_SPEED;
  } else if (right && !left) {          // หมุนตัวขวา (pivot turn)
    leftTargetSpeed  =  MAX_SPEED;
    rightTargetSpeed = -MAX_SPEED;
  } else if (fwd && !bwd) {
    leftTargetSpeed  = MAX_SPEED;
    rightTargetSpeed = MAX_SPEED;
  } else if (bwd && !fwd) {
    leftTargetSpeed  = -MAX_SPEED;
    rightTargetSpeed = -MAX_SPEED;
  } else {                              // ไม่กด หรือกดขัดแย้งกัน -> เป้าหมายเป็น 0 (จะค่อยๆ ผ่อนลง)
    leftTargetSpeed  = 0;
    rightTargetSpeed = 0;
  }
}

void driveLift() {
  bool up   = bitOn(4); // V5 Sprayer_Up
  bool down = bitOn(5); // V6 Sprayer_Down

  if (up && !down)      liftTargetSpeed =  MAX_SPEED;
  else if (down && !up) liftTargetSpeed = -MAX_SPEED;
  else                   liftTargetSpeed = 0;
}

void driveRotate() {
  bool rLeft  = bitOn(6); // V7 Rotate_Left
  bool rRight = bitOn(7); // V8 Rotate_Right

  if (rLeft && !rRight)      setMotorDigital(ROT_RPWM, ROT_LPWM, -1);
  else if (rRight && !rLeft) setMotorDigital(ROT_RPWM, ROT_LPWM, 1);
  else                       setMotorDigital(ROT_RPWM, ROT_LPWM, 0);
}

// ---------- ไต่/ผ่อนความเร็วจริงเข้าหาเป้าหมาย แล้วค่อยสั่งมอเตอร์ ----------
void updateRampAndApply() {
  if (millis() - lastRampUpdate < RAMP_INTERVAL_MS) return; // ยังไม่ถึงรอบอัปเดต
  lastRampUpdate = millis();

  stepToward(leftCurrentSpeed,  leftTargetSpeed);
  stepToward(rightCurrentSpeed, rightTargetSpeed);
  stepToward(liftCurrentSpeed,  liftTargetSpeed);

  setMotorPWM(LEFT_RPWM,  LEFT_LPWM,  leftCurrentSpeed);
  setMotorPWM(RIGHT_RPWM, RIGHT_LPWM, rightCurrentSpeed);
  setMotorPWM(LIFT_RPWM,  LIFT_LPWM,  liftCurrentSpeed);
}

// ขยับ current เข้าหา target ทีละ RAMP_STEP (ถ้าทิศกลับกัน จะไล่ผ่าน 0 ไปเองอย่างนุ่มนวล)
void stepToward(int &current, int target) {
  if (current < target) {
    current += RAMP_STEP;
    if (current > target) current = target;
  } else if (current > target) {
    current -= RAMP_STEP;
    if (current < target) current = target;
  }
}

// speed: -255..255 | บวก = หมุนไปทาง RPWM, ลบ = หมุนไปทาง LPWM, 0 = หยุด (สำหรับขาที่รองรับ PWM จริง)
void setMotorPWM(int rpwmPin, int lpwmPin, int speed) {
  if (speed > 0) {
    analogWrite(rpwmPin, speed);
    analogWrite(lpwmPin, 0);
  } else if (speed < 0) {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, -speed);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

// dir: 1 = เดินหน้าเต็มสปีด, -1 = ถอยหลังเต็มสปีด, 0 = หยุด (สำหรับขาที่ไม่มี PWM เช่น A0/A1)
void setMotorDigital(int rpwmPin, int lpwmPin, int dir) {
  if (dir > 0) {
    digitalWrite(rpwmPin, HIGH);
    digitalWrite(lpwmPin, LOW);
  } else if (dir < 0) {
    digitalWrite(rpwmPin, LOW);
    digitalWrite(lpwmPin, HIGH);
  } else {
    digitalWrite(rpwmPin, LOW);
    digitalWrite(lpwmPin, LOW);
  }
}

// หยุดทุกมอเตอร์ทันที (ไม่มีการไต่/ผ่อน) ใช้ตอน boot และตอน failsafe เท่านั้น
void emergencyStopAll() {
  leftTargetSpeed  = leftCurrentSpeed  = 0;
  rightTargetSpeed = rightCurrentSpeed = 0;
  liftTargetSpeed  = liftCurrentSpeed  = 0;

  setMotorPWM(LEFT_RPWM,  LEFT_LPWM,  0);
  setMotorPWM(RIGHT_RPWM, RIGHT_LPWM, 0);
  setMotorPWM(LIFT_RPWM,  LIFT_LPWM,  0);
  setMotorDigital(ROT_RPWM, ROT_LPWM, 0);
}
