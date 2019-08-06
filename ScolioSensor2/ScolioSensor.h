#include <SoftwareSerial.h>
#include <Time.h>
#include <Wire.h>
#include "Kalman.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#define TO_DEG 57.29577951308232087679815481410517033f
#define T_OUT 20 // каждый 20 миллисекунд будем проводить вычисления 
#define P_OUT 50 // каждый 50 миллисекунд будем выводить данные
#define FK 0.1 // коэффициент комплементарного фильтра

#define BTN 2
#define LED_FWD_X2 3
#define LED_FWD_X1 4
#define LED_BACK_X1 5
#define LED_BACK_X2 6
#define LED_RIGHT_Y2 7
#define LED_RIGHT_Y1 8
#define LED_LEFT_Y1 9
#define LED_LEFT_Y2 10

const int MPU_addr = 0x68; // I2C address of the MPU-6050

//int gRxPin = 10;
//int gTxPin = 11;

//SoftwareSerial BTSerial(gRxPin, gTxPin);

MPU6050 accelgyro;

Kalman kalmanX;
Kalman kalmanY;
uint8_t IMUAddress = 0x68;
/* IMU Data */
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double offsetX=0, offsetY=0;

// переменные для Калмана
float varVolt = 78.9;   // среднее отклонение (ищем в excel)
float varProcess = 0.5; // скорость реакции на изменение (подбирается вручную)
float Pc = 0.0, G = 0.0, P = 1.0, Xp = 0.0, Zp = 0.0, Xe = 0.0;
// переменные для Калмана

int gLedPin = 13;
int tx_count;

char arrData[2];

float filter(float val) {
  Pc = P + varProcess;
  G = Pc / (Pc + varVolt);
  P = (1 - G) * Pc;
  Xp = Xe;
  Zp = Xp;
  Xe = G * (val - Zp) + Xp; // "фильтрованное" значение
  return (Xe);
}

void setup() {
  // 38400 - для метода №1, 9600 - для метода №2
  //BTSerial.begin(9600);
  Serial.begin(9600);
  Wire.begin();
  //
  pinMode(gLedPin, OUTPUT);
  pinMode(BTN, INPUT);
  pinMode(LED_FWD_X2, OUTPUT);
  pinMode(LED_FWD_X1, OUTPUT);
  pinMode(LED_BACK_X1, OUTPUT);
  pinMode(LED_BACK_X2, OUTPUT);
  pinMode(LED_RIGHT_Y2, OUTPUT);
  pinMode(LED_RIGHT_Y1, OUTPUT);
  pinMode(LED_LEFT_Y1, OUTPUT);
  pinMode(LED_LEFT_Y2, OUTPUT);

  i2cWrite(0x6B,0x00); // Disable sleep mode      
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  /* Update all the values */

  digitalWrite(LED_FWD_X2, LOW);
  digitalWrite(LED_FWD_X1, HIGH);
  digitalWrite(LED_BACK_X1, HIGH);
  digitalWrite(LED_BACK_X2, LOW);
  digitalWrite(LED_RIGHT_Y2, LOW);
  digitalWrite(LED_RIGHT_Y1, HIGH);
  digitalWrite(LED_LEFT_Y1, HIGH);
  digitalWrite(LED_LEFT_Y2, LOW);
  delay(500);
  digitalWrite(LED_FWD_X2, HIGH);
  digitalWrite(LED_FWD_X1, HIGH);
  digitalWrite(LED_BACK_X1, HIGH);
  digitalWrite(LED_BACK_X2, HIGH);
  digitalWrite(LED_RIGHT_Y2, HIGH);
  digitalWrite(LED_RIGHT_Y1, HIGH);
  digitalWrite(LED_LEFT_Y1, HIGH);
  digitalWrite(LED_LEFT_Y2, HIGH);
  delay(500);
  digitalWrite(LED_FWD_X2, HIGH);
  digitalWrite(LED_FWD_X1, LOW);
  digitalWrite(LED_BACK_X1, LOW);
  digitalWrite(LED_BACK_X2, HIGH);
  digitalWrite(LED_RIGHT_Y2, HIGH);
  digitalWrite(LED_RIGHT_Y1, LOW);
  digitalWrite(LED_LEFT_Y1, LOW);
  digitalWrite(LED_LEFT_Y2, HIGH);
  delay(500);
  digitalWrite(LED_FWD_X2, LOW);
  digitalWrite(LED_FWD_X1, LOW);
  digitalWrite(LED_BACK_X1, LOW);
  digitalWrite(LED_BACK_X2, LOW);
  digitalWrite(LED_RIGHT_Y2, LOW);
  digitalWrite(LED_RIGHT_Y1, LOW);
  digitalWrite(LED_LEFT_Y1, LOW);
  digitalWrite(LED_LEFT_Y2, LOW);
  delay(500);

  for(int i=0; i<10;){
    unsigned long milsec;
    digitalWrite(LED_FWD_X2, LOW);
    digitalWrite(LED_FWD_X1, HIGH);
    digitalWrite(LED_BACK_X1, HIGH);
    digitalWrite(LED_BACK_X2, LOW);
    digitalWrite(LED_RIGHT_Y2, LOW);
    digitalWrite(LED_RIGHT_Y1, HIGH);
    digitalWrite(LED_LEFT_Y1, HIGH);
    digitalWrite(LED_LEFT_Y2, LOW);
    delay(50);
    digitalWrite(LED_FWD_X2, HIGH);
    digitalWrite(LED_FWD_X1, HIGH);
    digitalWrite(LED_BACK_X1, HIGH);
    digitalWrite(LED_BACK_X2, HIGH);
    digitalWrite(LED_RIGHT_Y2, HIGH);
    digitalWrite(LED_RIGHT_Y1, HIGH);
    digitalWrite(LED_LEFT_Y1, HIGH);
    digitalWrite(LED_LEFT_Y2, HIGH);
    delay(50);
    digitalWrite(LED_FWD_X2, HIGH);
    digitalWrite(LED_FWD_X1, LOW);
    digitalWrite(LED_BACK_X1, LOW);
    digitalWrite(LED_BACK_X2, HIGH);
    digitalWrite(LED_RIGHT_Y2, HIGH);
    digitalWrite(LED_RIGHT_Y1, LOW);
    digitalWrite(LED_LEFT_Y1, LOW);
    digitalWrite(LED_LEFT_Y2, HIGH);
    if(millis() - milsec >= 500){
      milsec = millis();
      digitalWrite(13, HIGH);
      gyro();
      digitalWrite(13, LOW);
      Serial.println(".");
      digitalWrite(LED_FWD_X2, LOW);
      digitalWrite(LED_FWD_X1, LOW);
      digitalWrite(LED_BACK_X1, LOW);
      digitalWrite(LED_BACK_X2, LOW);
      digitalWrite(LED_RIGHT_Y2, LOW);
      digitalWrite(LED_RIGHT_Y1, LOW);
      digitalWrite(LED_LEFT_Y1, LOW);
      digitalWrite(LED_LEFT_Y2, LOW);
      i++;
    }
  }
  //zero();
  offsetX = kalAngleX;
  offsetY = kalAngleY;
  delay(500);
  Serial.println("Ready");
  //Serial.print("X offset:"); Serial.print(offsetX); Serial.print("; Y offset:"); Serial.println(offsetY);
}

void loop() {
  digitalWrite(13, LOW);
  gyro();
  Serial.print(AcZ); Serial.print(" "); Serial.println((int)filter(AcZ));
  delay(100);
  //   Сохраняем полученные значения в массив и передаём его:
  /*if(varX != oldvarX || varY != oldvarY){
    //digitalWrite(13, HIGH);
    //BTSerial.print("/");
    oldvarX = varX;
    oldvarY = varY;
    arrData[0]=varX;
    arrData[1]=varY;
    digitalWrite(13, LOW);
   //for (int i=0; i<sizeof(arrData); i++){
      digitalWrite(13, HIGH);
      Serial.write(arrData, sizeof(arrData));
      //Serial.print(kalAngleY); Serial.print("  "); Serial.println(kalAngleX);
      //Serial.print(varX); Serial.print("  "); Serial.println(varY);
    //}
    
    //Serial.print(kalAngleX); Serial.print("  "); Serial.print(kalAngleY); Serial.print("; "); Serial.print(varX); Serial.print("  "); Serial.println(varY); // отладка
    digitalWrite(13, HIGH);
    //BTSerial.print(".");
  }*/
  delay(50);
}



void gyro(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers

  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H)  & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  kalAngleX=filter(GyZ)-offsetX;
  kalAngleY=filter(GyY)-offsetY;

  //   Преобразуем считанные показания и добавляем люфт:
    if(kalAngleX <= -32){
      varX = -5;
      digitalWrite(LED_FWD_X2, HIGH);
      digitalWrite(LED_FWD_X1, LOW);
      digitalWrite(LED_BACK_X1, LOW);
      digitalWrite(LED_BACK_X2, LOW);
    }
    if(kalAngleX <= -22 && kalAngleX >-30){
      varX = -3;
      digitalWrite(LED_FWD_X2, HIGH);
      digitalWrite(LED_FWD_X1, HIGH);
      digitalWrite(LED_BACK_X1, LOW);
      digitalWrite(LED_BACK_X2, LOW);
    }
     if(kalAngleX <= -12 && kalAngleX > -20){
      varX = -1;
      digitalWrite(LED_FWD_X2, LOW);
      digitalWrite(LED_FWD_X1, HIGH);
      digitalWrite(LED_BACK_X1, LOW);
      digitalWrite(LED_BACK_X2, LOW);
    }
    if(kalAngleX < 10 && kalAngleX > -10){
      varX = 0;
      digitalWrite(LED_FWD_X2, LOW);
      digitalWrite(LED_FWD_X1, LOW);
      digitalWrite(LED_BACK_X1, LOW);
      digitalWrite(LED_BACK_X2, LOW);
    }
    if(kalAngleX < 20 && kalAngleX >= 12){
      varX = 1;
      digitalWrite(LED_FWD_X2, LOW);
      digitalWrite(LED_FWD_X1, LOW);
      digitalWrite(LED_BACK_X1, HIGH);
      digitalWrite(LED_BACK_X2, LOW);
    }
    if(kalAngleX < 30 && kalAngleX >= 22){
      varX = 3;
      digitalWrite(LED_FWD_X2, LOW);
      digitalWrite(LED_FWD_X1, LOW);
      digitalWrite(LED_BACK_X1, HIGH);
      digitalWrite(LED_BACK_X2, HIGH);
    }
    if(kalAngleX >= 32){
      varX = 5;
      digitalWrite(LED_FWD_X2, LOW);
      digitalWrite(LED_FWD_X1, LOW);
      digitalWrite(LED_BACK_X1, LOW);
      digitalWrite(LED_BACK_X2, HIGH);
    }

    if(kalAngleY <= -16){
      varY = -3;
      digitalWrite(LED_RIGHT_Y2, HIGH);
      digitalWrite(LED_RIGHT_Y1, LOW);
      digitalWrite(LED_LEFT_Y1, LOW);
      digitalWrite(LED_LEFT_Y2, LOW);
    }
    if(kalAngleY <= -11 && kalAngleY >-15){
      varY = -2;
      digitalWrite(LED_RIGHT_Y2, HIGH);
      digitalWrite(LED_RIGHT_Y1, HIGH);
      digitalWrite(LED_LEFT_Y1, LOW);
      digitalWrite(LED_LEFT_Y2, LOW);
    }
    if(kalAngleY <= -6 && kalAngleY > -10){
      varY = -1;
      digitalWrite(LED_RIGHT_Y2, LOW);
      digitalWrite(LED_RIGHT_Y1, HIGH);
      digitalWrite(LED_LEFT_Y1, LOW);
      digitalWrite(LED_LEFT_Y2, LOW);
    }
    if(kalAngleY < 5 && kalAngleY > -5){
      varY = 0;
      digitalWrite(LED_RIGHT_Y2, LOW);
      digitalWrite(LED_RIGHT_Y1, LOW);
      digitalWrite(LED_LEFT_Y1, LOW);
      digitalWrite(LED_LEFT_Y2, LOW);
    }
    if(kalAngleY < 10 && kalAngleY >= 6){
      varY = 1;
      digitalWrite(LED_RIGHT_Y2, LOW);
      digitalWrite(LED_RIGHT_Y1, LOW);
      digitalWrite(LED_LEFT_Y1, HIGH);
      digitalWrite(LED_LEFT_Y2, LOW);
    }
    if(kalAngleY < 15 && kalAngleY >= 11){
      varY = 2;
      digitalWrite(LED_RIGHT_Y2, LOW);
      digitalWrite(LED_RIGHT_Y1, LOW);
      digitalWrite(LED_LEFT_Y1, HIGH);
      digitalWrite(LED_LEFT_Y2, HIGH);
    }
    if(kalAngleY >= 16){
      varY = 3;
      digitalWrite(LED_RIGHT_Y2, LOW);
      digitalWrite(LED_RIGHT_Y1, LOW);
      digitalWrite(LED_LEFT_Y1, LOW);
      digitalWrite(LED_LEFT_Y2, HIGH);
    }
}

void zero(){
  Serial.println("Updating internal sensor offsets...");
  for(int i=0; i<10;){
    unsigned long milsec;
    digitalWrite(LED_FWD_X2, LOW);
    digitalWrite(LED_FWD_X1, HIGH);
    digitalWrite(LED_BACK_X1, HIGH);
    digitalWrite(LED_BACK_X2, LOW);
    digitalWrite(LED_RIGHT_Y2, LOW);
    digitalWrite(LED_RIGHT_Y1, HIGH);
    digitalWrite(LED_LEFT_Y1, HIGH);
    digitalWrite(LED_LEFT_Y2, LOW);
    delay(50);
    digitalWrite(LED_FWD_X2, HIGH);
    digitalWrite(LED_FWD_X1, HIGH);
    digitalWrite(LED_BACK_X1, HIGH);
    digitalWrite(LED_BACK_X2, HIGH);
    digitalWrite(LED_RIGHT_Y2, HIGH);
    digitalWrite(LED_RIGHT_Y1, HIGH);
    digitalWrite(LED_LEFT_Y1, HIGH);
    digitalWrite(LED_LEFT_Y2, HIGH);
    delay(50);
    digitalWrite(LED_FWD_X2, HIGH);
    digitalWrite(LED_FWD_X1, LOW);
    digitalWrite(LED_BACK_X1, LOW);
    digitalWrite(LED_BACK_X2, HIGH);
    digitalWrite(LED_RIGHT_Y2, HIGH);
    digitalWrite(LED_RIGHT_Y1, LOW);
    digitalWrite(LED_LEFT_Y1, LOW);
    digitalWrite(LED_LEFT_Y2, HIGH);
    if(millis() - milsec >= 500){
      milsec = millis();
      digitalWrite(13, HIGH);
      gyro();
      digitalWrite(13, LOW);
      Serial.println(".");
      digitalWrite(LED_FWD_X2, LOW);
      digitalWrite(LED_FWD_X1, LOW);
      digitalWrite(LED_BACK_X1, LOW);
      digitalWrite(LED_BACK_X2, LOW);
      digitalWrite(LED_RIGHT_Y2, LOW);
      digitalWrite(LED_RIGHT_Y1, LOW);
      digitalWrite(LED_LEFT_Y1, LOW);
      digitalWrite(LED_LEFT_Y2, LOW);
      i++;
    }
  }

  offsetX = kalAngleX;
  offsetY = kalAngleY;
  delay(500);
}