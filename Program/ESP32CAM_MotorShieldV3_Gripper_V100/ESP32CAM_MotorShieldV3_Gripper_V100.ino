// Author : Mason, masonchen1003@gmail.com
// FB : https://www.facebook.com/mason.chen.1420
// Licensed under the Creative Commons - Attribution - Non-Commercial license.

// 最後編輯 2022-4-01 by ShinWei Chiou
// 初版

/*
  Board: ESP32 Wrover Module
  Flash Frequency: 80MHz
  Flash Mode: QIO
  Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)

  ESP32-CAM Motor Shield V3.0
  MotorA = 15;
  MotorA = 14;
  MotorB = 13;
  MotorB = 12;
  ServoArmPin = 1;
  ServoGriPin = 2;

  PS3 Controller Bluetooth Connect
  Pairing the PS3 Controller : SixaxisPairToolSetup-0.3.1.exe
  <https://github.com/jvpernis/esp32-ps3>
*/

#include <Ps3Controller.h>  // 包含 ps3控制的 library 
String ESP32_MAC_Address;

// 宣告 Servo 用到的腳位
#define ServoArmPin   1 // TX , VCC , GND
#define ServoGriPin   2 // P2 , VCC , GND

// Control SG90 Servo, PWM Period is 20ms and Duty is 1->2ms
// 控制伺服馬達的 function
void servo_angle(int channel, int angle) {
  int SERVO_RESOLUTION = 16;
  float range = (pow(2, SERVO_RESOLUTION) - 1) / 10;
  float minDuty = (pow(2, SERVO_RESOLUTION) - 1) / 40;

  uint32_t duty = (range) * (float)angle / 180.0 + minDuty;
  ledcWrite(channel, duty);
  delay(5);
}

// 宣告 Motor 用到的腳位
#define MotorA1   15
#define MotorA2   14
#define MotorB1   13
#define MotorB2   12
int speed_FB = 250;    // 控制前進、後退的轉速
int speed_LR = 150;    // 控制左右轉的轉速

void Move_Forward() {
  digitalWrite(MotorA1, HIGH);
  ledcWrite(9, 255 - speed_FB);
  digitalWrite(MotorB1, HIGH);
  ledcWrite(10, 255 - speed_FB );
  delay(50);
}

void Move_Backward() {
  digitalWrite(MotorA1, LOW);
  ledcWrite(9, speed_FB );
  digitalWrite(MotorB1, LOW);
  ledcWrite(10, speed_FB );
  delay(50);
}

void Move_TurnRight() {
  digitalWrite(MotorA1, LOW);
  ledcWrite(9, speed_LR );
  digitalWrite(MotorB1, HIGH);
  ledcWrite(10, 255 - speed_LR );
  delay(50);
}

void Move_TurnLeft() {
  digitalWrite(MotorA1, HIGH);
  ledcWrite(9, 255 - speed_LR );
  digitalWrite(MotorB1, LOW);
  ledcWrite(10, speed_LR );
  delay(50);
}

void Motor_Stop() {
  digitalWrite(MotorA1, LOW);
  ledcWrite(9, 0 );
  digitalWrite(MotorB1, LOW);
  ledcWrite(10, 0 );
}

// servo offset, pleaee update the offset values
int cal[5]  = {  0,  10,  0,  0,  0};  // initial all 0


int rX; int rY; int lX; int lY;  // PS3 手把遙桿的 X 軸 / Y 軸

int posA = 90;
int posB = 90;
int posC = 90;
int posD = 90;
int posE = 90;

int delay_t = 10;

void setup() {
  // 設定 UART 通訊
  Serial.begin(9600);

  // 設定 ESP32-CAM MAC Address
//    Ps3.begin("01:02:03:04:05:06");  // 換成你自己手把的 address
  Ps3.begin("34:ab:95:72:b9:e2");  // 換成你自己手把的 address

  // 設定搖桿 LED 顯示
  Ps3.setPlayer(1);

  pinMode(MotorA1, OUTPUT); //MotorA
  pinMode(MotorB1, OUTPUT); //MotorB

  ledcSetup(9, 50, 8);   // 宣告腳位為 PWM，8 bits  0-255
  ledcAttachPin(MotorA2, 9);
  ledcSetup(10, 50, 8);
  ledcAttachPin(MotorB2, 10);

  ledcSetup(ServoArmPin, 50, 16); // channel , freq, resolution
  ledcAttachPin(ServoArmPin, ServoArmPin);
  ledcSetup(ServoGriPin, 50, 16);
  ledcAttachPin(ServoGriPin, ServoGriPin);

  servo_angle(ServoArmPin, (90 + cal[0]));
  servo_angle(ServoGriPin, (90 + cal[1]));

  Motor_Stop();
}

void loop() {
  // PS3 control
  if ( Ps3.data.button.up )  {
    Move_Forward();
  }
  else if ( Ps3.data.button.down )  {
    Move_Backward();
  }
  else if ( Ps3.data.button.left )  {
    Move_TurnLeft();
  }
  else if ( Ps3.data.button.right )  {
    Move_TurnRight();
  }

  lX = (Ps3.data.analog.stick.lx); // 讀取左遙桿的 X 值
  lY = (Ps3.data.analog.stick.ly); // 讀取左遙桿的 Y 值
  rX = (Ps3.data.analog.stick.rx); // 讀取右遙桿的 X 值
  rY = (Ps3.data.analog.stick.ry); // 讀取右遙桿的 Y 值

  // Serial.print(lX); Serial.print(" ");
  // Serial.print(lY); Serial.print(" ");
  // Serial.print(rX); Serial.print(" ");
  // Serial.println(rY);

  // 左遙桿控制車子
  if (abs(lX) < 30 && abs(lY) < 30) {
    Motor_Stop() ;
    Serial.println("s");
  } else if (abs(lX) > abs(lY)) {
    if (lX > 20) {   // r
      Move_TurnRight();
      Serial.println("r");
    } else {  // l
      Move_TurnLeft();
      Serial.println("l");
    }
  } else {
    if (lY < -20) {
      Move_Forward();
      Serial.println("f");
    } else {
      Move_Backward();
      Serial.println("b");
    }
  }

  //右遙桿控制手臂
  //右遙桿控制手臂底下的兩顆馬達
  if (rY < -50 && posB < 140) {
    posB = posB + 3;
    servo_angle(ServoArmPin, posB + cal[0]);
    delay(delay_t * 3);
    Serial.println(posB);
  }
  if (rY > 50 && posB > 80) {
    posB = posB - 3;
    servo_angle(ServoArmPin, posB + cal[0]);
    delay(delay_t * 3);
    Serial.println(posB);
  }
  if (rX > 50 && posC < 90) {
    posC = posC + 3;
    servo_angle(ServoGriPin, posC + cal[1]);
    delay(delay_t * 3);
    Serial.println("C+");
  }
  if (rX < -50 && posC > 40) {
    posC = posC - 3;
    servo_angle(ServoGriPin, posC + cal[1]);
    delay(delay_t * 3);
    Serial.println("C-");
  }
}
