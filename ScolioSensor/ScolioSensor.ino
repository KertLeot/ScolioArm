#include <Wire.h>
#include <Kalman.h>
#include <SoftPWM.h>

#define BTN 2
#define LED_FWD_X2 3
#define LED_FWD_X1 4
#define LED_BACK_X1 5
#define LED_BACK_X2 6
#define LED_RIGHT_Y2 7
#define LED_RIGHT_Y1 8
#define LED_LEFT_Y1 9
#define LED_LEFT_Y2 10
#define BRIGHTNESS 60

#define BT_IN 11
#define BT_OUT 12

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;



/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

double offsetX = 0, offsetY = 0, AngleX, AngleY;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

int gLedPin = 13;

String message = "";

int varX, varY, oldvarX, oldvarY, mode = 0;

void bluetooth(int X, int Y, int m) {
  SoftPWMSet(13, 255);
  oldvarX = X;
  oldvarY = Y;
  message += "$";
  message += X;
  message += " ";
  message += Y;
  message += " ";
  message += m;
  message += ";";
  SoftPWMSet(13, 0);
  SoftPWMSet(13, 255);
  //Serial.write(message);
  SoftPWMSet(13, 255);
  Serial.println(message);
  //Serial.print(X); Serial.print("  "); Serial.println(Y);
  message = "";
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  SoftPWMBegin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    //Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees

  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;


  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
  //
  //pinMode(gLedPin, OUTPUT);
  pinMode(BTN, INPUT);
  pinMode(LED_FWD_X2, OUTPUT);
  pinMode(LED_FWD_X1, OUTPUT);
  pinMode(LED_BACK_X1, OUTPUT);
  pinMode(LED_BACK_X2, OUTPUT);
  pinMode(LED_RIGHT_Y2, OUTPUT);
  pinMode(LED_RIGHT_Y1, OUTPUT);
  pinMode(LED_LEFT_Y1, OUTPUT);
  pinMode(LED_LEFT_Y2, OUTPUT);
  pinMode(BT_IN, INPUT);
  pinMode(BT_OUT, OUTPUT);



  attachInterrupt(digitalPinToInterrupt(BTN), zero, RISING);

  /* Запуск модуля */
  led_X1();
  led_Y3();
  led_X_1();
  led_Y_3();
  delay(250);
  led_X2();
  led_Y2();
  led_X_2();
  led_Y_2();
  delay(250);
  led_X3();
  led_Y1();
  led_X_3();
  led_Y_1();
  delay(250);
  led_0();
  delay(250);

  for (int i = 0; i < 10;) {
    unsigned long milsec;
    led_X3();
    led_Y1();
    led_X_3();
    led_Y_1();
    if (millis() - milsec >= 100) {
      milsec = millis();
      SoftPWMSet(13, 255);
      gyro();
      SoftPWMSet(13, 0);
      //Serial.print(".");
      led_X2();
      led_Y2();
      led_X_2();
      led_Y_2();
      delay(50);
      i++;
    }
  }
  led_0();
  zero();
  delay(500);
  //Serial.println("Ready");
  //Serial.print("X offset:"); Serial.print(offsetX); Serial.print("; Y offset:"); Serial.println(offsetY);
}

void loop() {
  if (digitalRead(BT_IN) == HIGH){
    SoftPWMSet(BT_OUT, 60);
  } else{
    SoftPWMSet(BT_OUT, 0);
  }
  SoftPWMSet(13, 0);
  gyro();
  level();
  motion();
  //   Сохраняем полученные значения в массив и передаём его:

  if (varX != oldvarX || varY != oldvarY) {
    bluetooth(varX, varY, mode);
    //Serial.println(gyroY / 131.00);
  }

  delay(50);
}


void gyro() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees

  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;


  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);


  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;


  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  AngleX = -(kalAngleY - offsetY);
  AngleY = -(kalAngleX - offsetX);
}

void motion() {
  if (gyroY / 131.00 < -100.00 && accZ < 8000.00) {
    ++mode;
    varX = 0;
    varY = 0;
    if ( mode > 1) { //ограничение режимов
      mode = 0;
    }
    if (mode == 0) {
      SoftPWMSet(LED_FWD_X2, 0);
      SoftPWMSet(LED_FWD_X1, 0);
      SoftPWMSet(LED_BACK_X1, 0);
      SoftPWMSet(LED_BACK_X2, 0);
      SoftPWMSet(LED_RIGHT_Y2, 0);
      SoftPWMSet(LED_RIGHT_Y1, 0);
      SoftPWMSet(LED_LEFT_Y1, 0);
      SoftPWMSet(LED_LEFT_Y2, 0);
    } else {
      SoftPWMSet(LED_FWD_X2, BRIGHTNESS);
      SoftPWMSet(LED_FWD_X1, BRIGHTNESS);
      SoftPWMSet(LED_BACK_X1, BRIGHTNESS);
      SoftPWMSet(LED_BACK_X2, BRIGHTNESS);
      SoftPWMSet(LED_RIGHT_Y2, BRIGHTNESS);
      SoftPWMSet(LED_RIGHT_Y1, BRIGHTNESS);
      SoftPWMSet(LED_LEFT_Y1, BRIGHTNESS);
      SoftPWMSet(LED_LEFT_Y2, BRIGHTNESS);
    }
    bluetooth(0, 0, mode);
    //Serial.println(mode);
    //Serial.println(gyroY / 131.00);
    //Serial.println(accZ);
  }
}

void level() {
  if (abs(gyroY / 131.00) < 30.00 && abs(gyroX / 131.00) < 30.00) {
    //   Преобразуем считанные показания и добавляем люфт:
    if (AngleX <= -32) {
      varX = -5;
      led_X3();
    }
    if (AngleX <= -22 && AngleX > -30) {
      varX = -3;
      led_X2();
    }
    if (AngleX <= -12 && AngleX > -20) {
      varX = -1;
      led_X1();
    }
    if (AngleX < 10 && AngleX > -10) {
      varX = 00;
      if (mode == 0) {
        SoftPWMSet(LED_FWD_X2, 0);
        SoftPWMSet(LED_FWD_X1, 0);
        SoftPWMSet(LED_BACK_X1, 0);
        SoftPWMSet(LED_BACK_X2, 0);
      } else {
        SoftPWMSet(LED_FWD_X2, BRIGHTNESS);
        SoftPWMSet(LED_FWD_X1, BRIGHTNESS);
        SoftPWMSet(LED_BACK_X1, BRIGHTNESS);
        SoftPWMSet(LED_BACK_X2, BRIGHTNESS);
      }
    }
    if (AngleX < 20 && AngleX >= 12) {
      varX = 1;
      led_X_1();
    }
    if (AngleX < 30 && AngleX >= 22) {
      varX = 3;
      led_X_2();
    }
    if (AngleX >= 32) {
      varX = 5;
      led_X_3();
    }

    if (AngleY <= -16) {
      varY = -3;
      led_Y3();
    }
    if (AngleY <= -11 && AngleY > -15) {
      varY = -2;
      led_Y2();
    }
    if (AngleY <= -6 && AngleY > -10) {
      varY = -1;
      led_Y1();
    }
    if (AngleY < 5 && AngleY > -5) {
      varY = 00;
      if (mode == 0) {
        SoftPWMSet(LED_RIGHT_Y2, 0);
        SoftPWMSet(LED_RIGHT_Y1, 0);
        SoftPWMSet(LED_LEFT_Y1, 0);
        SoftPWMSet(LED_LEFT_Y2, 0);
      } else {
        SoftPWMSet(LED_RIGHT_Y2, BRIGHTNESS);
        SoftPWMSet(LED_RIGHT_Y1, BRIGHTNESS);
        SoftPWMSet(LED_LEFT_Y1, BRIGHTNESS);
        SoftPWMSet(LED_LEFT_Y2, BRIGHTNESS);
      }
    }
    if (AngleY < 10 && AngleY >= 6) {
      varY = 1;
      led_Y_1();
    }
    if (AngleY < 15 && AngleY >= 11) {
      varY = 2;
      led_Y_2();
    }
    if (AngleY >= 16) {
      varY = 3;
      led_Y_3();
    }
  } else {
    varX = 0;
    varY = 0;
  }
}

void zero() {
  offsetX = kalAngleX;
  offsetY = kalAngleY;
  bluetooth(0, 0, mode);
}

void led_0() {
  SoftPWMSet(LED_FWD_X2, 0);
  SoftPWMSet(LED_FWD_X1, 0);
  SoftPWMSet(LED_BACK_X1, 0);
  SoftPWMSet(LED_BACK_X2, 0);
  SoftPWMSet(LED_RIGHT_Y2, 0);
  SoftPWMSet(LED_RIGHT_Y1, 0);
  SoftPWMSet(LED_LEFT_Y1, 0);
  SoftPWMSet(LED_LEFT_Y2, 0);
}

void led_X1() {
  if (mode == 0) {
    SoftPWMSet(LED_FWD_X2, 0);
    SoftPWMSet(LED_FWD_X1, BRIGHTNESS);
  } else {
    SoftPWMSet(LED_FWD_X2, BRIGHTNESS);
    SoftPWMSet(LED_FWD_X1, 0);
  }
}

void led_X2() {
  if (mode == 0) {
    SoftPWMSet(LED_FWD_X2, BRIGHTNESS);
    SoftPWMSet(LED_FWD_X1, BRIGHTNESS);
  } else {
    SoftPWMSet(LED_FWD_X2, 0);
    SoftPWMSet(LED_FWD_X1, 0);
  }
}

void led_X3() {
  if (mode == 0) {
    SoftPWMSet(LED_FWD_X2, BRIGHTNESS);
    SoftPWMSet(LED_FWD_X1, 0);
  } else {
    SoftPWMSet(LED_FWD_X2, 0);
    SoftPWMSet(LED_FWD_X1, BRIGHTNESS);
  }
}

void led_X_1() {
  if (mode == 0) {
    SoftPWMSet(LED_BACK_X1, BRIGHTNESS);
    SoftPWMSet(LED_BACK_X2, 0);
  } else {
    SoftPWMSet(LED_BACK_X1, 0);
    SoftPWMSet(LED_BACK_X2, BRIGHTNESS);
  }
}

void led_X_2() {
  if (mode == 0) {
    SoftPWMSet(LED_BACK_X1, BRIGHTNESS);
    SoftPWMSet(LED_BACK_X2, BRIGHTNESS);
  } else {
    SoftPWMSet(LED_BACK_X1, 0);
    SoftPWMSet(LED_BACK_X2, 0);
  }
}

void led_X_3() {
  if (mode == 0) {
    SoftPWMSet(LED_BACK_X1, 0);
    SoftPWMSet(LED_BACK_X2, BRIGHTNESS);
  } else {
    SoftPWMSet(LED_BACK_X1, BRIGHTNESS);
    SoftPWMSet(LED_BACK_X2, 0);
  }
}

void led_Y1() {
  if (mode == 0) {
    SoftPWMSet(LED_RIGHT_Y1, BRIGHTNESS);
    SoftPWMSet(LED_RIGHT_Y2, 0);
  } else {
    SoftPWMSet(LED_RIGHT_Y1, 0);
    SoftPWMSet(LED_RIGHT_Y2, BRIGHTNESS);
  }
}

void led_Y2() {
  if (mode == 0) {
    SoftPWMSet(LED_RIGHT_Y1, BRIGHTNESS);
    SoftPWMSet(LED_RIGHT_Y2, BRIGHTNESS);
  } else {
    SoftPWMSet(LED_RIGHT_Y1, 0);
    SoftPWMSet(LED_RIGHT_Y2, 0);
  }
}

void led_Y3() {
  if (mode == 0) {
    SoftPWMSet(LED_RIGHT_Y1, 0);
    SoftPWMSet(LED_RIGHT_Y2, BRIGHTNESS);
  } else {
    SoftPWMSet(LED_RIGHT_Y1, BRIGHTNESS);
    SoftPWMSet(LED_RIGHT_Y2, 0);
  }
}

void led_Y_1() {
  if (mode == 0) {
    SoftPWMSet(LED_LEFT_Y1, BRIGHTNESS);
    SoftPWMSet(LED_LEFT_Y2, 0);
  } else {
    SoftPWMSet(LED_LEFT_Y1, 0);
    SoftPWMSet(LED_LEFT_Y2, BRIGHTNESS);
  }
}

void led_Y_2() {
  if (mode == 0) {
    SoftPWMSet(LED_LEFT_Y1, BRIGHTNESS);
    SoftPWMSet(LED_LEFT_Y2, BRIGHTNESS);
  } else {
    SoftPWMSet(LED_LEFT_Y1, 0);
    SoftPWMSet(LED_LEFT_Y2, 0);
  }
}

void led_Y_3() {
  if (mode == 0) {
    SoftPWMSet(LED_LEFT_Y1, 0);
    SoftPWMSet(LED_LEFT_Y2, BRIGHTNESS);
  } else {
    SoftPWMSet(LED_LEFT_Y1, BRIGHTNESS);
    SoftPWMSet(LED_LEFT_Y2, 0);
  }
}
