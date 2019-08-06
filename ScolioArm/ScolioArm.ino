#include <Servo.h>


/*
* 
************************************************************ 
*      https://goo.gl/photos/xvHwKM44gh7wN5UK6             *
*      https://www.youtube.com/watch?v=PL4174oBPTs         *
*************************************************************
*
*/



float TODEG = 57.3; // Conversion from rad to deg

#define PARSE_AMOUNT 3
#define BT_STATE_1 43
#define BT_STATE_2 40
#define BT_ENABLE_1 42
#define BT_ENABLE_2 41

// ======================== Servos & distances ===================================

// Размеры руки
float elbow =  0.0;  // cm
float arm = 17.0;  // cm плечо
float forearm = 18.0;  // cm предплечье

// Сервоприводы суставов
Servo Sigma1;
Servo Sigma2;
Servo Sigma3;
Servo Sigma5;

// Пины сервоприводов
#define SIGMA_1 4
#define SIGMA_2 3
#define SIGMA_3 2
#define SIGMA_5 6

// Координаты кисти
double targetX, targetY, targetZ;

// Диапазоны углов сгибания суставов
#define MAX_SIGMA_1 180 
#define MIN_SIGMA_1 0
#define MAX_SIGMA_2 180
#define MIN_SIGMA_2 0
#define MAX_SIGMA_3 160
#define MIN_SIGMA_3 55
#define MAX_SIGMA_4 95
#define MIN_SIGMA_4 0
#define MAX_SIGMA_5 180
#define MIN_SIGMA_5 0

// Offset angles of each servo
double offsetSigma1 = 135;
double offsetSigma2 = 210;
double offsetSigma3 = 27;
double offsetSigma5 = 0;

// Максимальный/минимальный радиус-вектор
float Rmax = arm + forearm;
float Rmin = sqrt(sq(arm) + sq(forearm) - 2 * arm*forearm*cos(50));

// Переменные для показаний с датчиков
int X_R = 0, Y_R = 0, X_L = 0, Y_L = 0;

// Переменные для значений углов
int sigma_1 = 0, sigma_2 = (MAX_SIGMA_2 - MIN_SIGMA_2) / 2, sigma_3 = (MAX_SIGMA_3 - MIN_SIGMA_3) / 2, sigma_5 = (MAX_SIGMA_5 - MIN_SIGMA_5) / 2;

int intData1[PARSE_AMOUNT], intData2[PARSE_AMOUNT];
boolean recievedFlag1, recievedFlag2;
boolean getStarted1, getStarted2;
boolean readyFlag;
byte index1, index2;
String string_convert1 = "", string_convert2 = "";
int modeR = 0, modeL = 0;

int i = 0;

boolean demonstration;

unsigned long time_oled_update;
unsigned long timeout = 0;

// ===================================================================================

void serialEvent1() {
  if (Serial1.available() > 0) {
    char incomingByte1 = Serial1.read();        // обязательно ЧИТАЕМ входящий символ
    if (getStarted1) {                         // если приняли начальный символ (парсинг разрешён)
      if (incomingByte1 != ' ' && incomingByte1 != ';') {   // если это не пробел И не конец
        string_convert1 += incomingByte1;       // складываем в строку
      } else {                                // если это пробел или ; конец пакета
        intData1[index1] = string_convert1.toInt();  // преобразуем строку в int и кладём в массив
        string_convert1 = "";                  // очищаем строку
        index1++;                              // переходим к парсингу следующего элемента массива
      }
    }
    if (incomingByte1 == '$') {                // если это $
      getStarted1 = true;                      // поднимаем флаг, что можно парсить
      index1 = 0;                              // сбрасываем индекс
      string_convert1 = "";                    // очищаем строку
    }
    if (incomingByte1 == ';') {                // если таки приняли ; - конец парсинга
      getStarted1 = false;                     // сброс
      recievedFlag1 = true;                    // флаг на принятие
    }
  }
}

void serialEvent2() {
  if (Serial2.available() > 0) {
    char incomingByte2 = Serial2.read();        // обязательно ЧИТАЕМ входящий символ
    if (getStarted2) {                         // если приняли начальный символ (парсинг разрешён)
      if (incomingByte2 != ' ' && incomingByte2 != ';') {   // если это не пробел И не конец
        string_convert2 += incomingByte2;       // складываем в строку
      } else {                                // если это пробел или ; конец пакета
        intData2[index2] = string_convert2.toInt();  // преобразуем строку в int и кладём в массив
        string_convert2 = "";                  // очищаем строку
        index2++;                              // переходим к парсингу следующего элемента массива
      }
    }
    if (incomingByte2 == '$') {                // если это $
      getStarted2 = true;                      // поднимаем флаг, что можно парсить
      index2 = 0;                              // сбрасываем индекс
      string_convert2 = "";                    // очищаем строку
    }
    if (incomingByte2 == ';') {                // если таки приняли ; - конец парсинга
      getStarted2 = false;                     // сброс
      recievedFlag2 = true;                    // флаг на принятие
    }
  }
}

// ======================== Setup ====================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Start!");
  Serial1.begin(9600);
  Serial2.begin(9600);
  Sigma1.attach(SIGMA_1);
  Sigma2.attach(SIGMA_2);
  Sigma3.attach(SIGMA_3);
  Sigma5.attach(SIGMA_5);
  

  // Init coords
  targetX = forearm;
  targetY = arm;
  targetZ = 0;
  
  moveArm(targetX,targetY,targetZ,10,10);
}
// ===================================================================================



// ======================== Movements methods =======================================
/*void walk2(int div, int time){

  movLegs(9,2,5.5,div,time);
  movLegs(9,7,10,div,time);
  movLegs(9,7,7,div,time);
  movLegs(9,7,4,div,time);
  
}*/
// ===================================================================================



// ======================== Methods to move legs =============================================

// Linearly move arm to the given coordinates
void moveArm(double t_targetX, double t_targetY, double t_targetZ, int div, int time){
  
  // div -> divisions (number of movements to reach the new coordinates)
  // time-> time between each sub-movement

  double subtargetX = (t_targetX - targetX)/div;  // Increment to be added to the current position to reach the destination in "div" movements
  double subtargetY = (t_targetY - targetY)/div;
  double subtargetZ = (t_targetZ - targetZ)/div;
  

  // Do the "div" movements
  for(int i = 0; i < div; i++){
    targetX += subtargetX;
    targetY += subtargetY;
    targetZ += subtargetZ;
    

    updateLServos();
    
    delay(time);
  } 
}

// ===================================================================================

// ======================== IK functions =============================================
void updateLServos(){
    // Set the servos to their coords with IK functions
    sigma_1 = IKang1(targetX,targetY,targetZ);
    sigma_2 = IKang2(targetX,targetY,targetZ);
    sigma_3 = IKang3(targetX,targetY,targetZ);
    Sigma1.write(offsetSigma1 + constrain(sigma_1, MIN_SIGMA_1, MAX_SIGMA_1));
    Sigma2.write(offsetSigma2 + constrain(sigma_2, MIN_SIGMA_2, MAX_SIGMA_2));
    Sigma3.write(offsetSigma3 + constrain(sigma_3, MIN_SIGMA_3, MAX_SIGMA_3));
    Sigma5.write(offsetSigma5 + constrain(sigma_5, MIN_SIGMA_5, MAX_SIGMA_5));
/*Serial.print("Servo1="); Serial.print(sigma_1); 
Serial.print("; Servo2="); Serial.print(sigma_2);
Serial.print("; Servo3="); Serial.println(sigma_3);*/
    
}

// Angle for servo 1
int IKang1(double x, double y, double z){
  double gamma = atan2(z, x);
  return (gamma * TODEG);
}

// Angle for servo 2
int IKang2(double x, double y, double z){
  double hip = sqrt( pow(y,2) + pow((x - elbow),2));
  double alpha1 = acos(y/hip);
  double alpha2 = acos( (pow(forearm,2) - pow(arm, 2) - pow(hip,2))/(-2*arm*hip));
  double alpha = alpha1 + alpha2;
  return (alpha * TODEG);
}

// Angle for servo 3
int IKang3(double x, double y, double z){
  double hip = sqrt( pow(y,2) + pow((x - elbow),2));
  double beta = acos(( pow(hip,2) - pow(forearm,2) - pow(arm,2))/(-2*forearm*arm));
  return (beta * TODEG);
}
// ===================================================================================




void loop() {

  /*if (millis() - time_oled_update > 200) {
    time_oled_update = millis();
    //u8g2.sendBuffer();
  }
  if (digitalRead(BT_STATE_1) == LOW && digitalRead(BT_STATE_2) == LOW) {
    if (timeout == 0) {
      timeout = millis();
    }
    if (millis() - timeout > 10000) {
      demonstration = true;
    }
  } else {
    timeout = 0;
    demonstration = false;
  }*/

  //demo();
  
    X_R = intData2[0];
    Y_R = intData2[1];
    X_L = -intData1[0];
    Y_L = intData1[1];
    modeR = intData2[2];
    modeL = intData1[2];
/*Serial.print("X_R="); Serial.print(X_R); Serial.print(" Y_R="); Serial.print(Y_R);
    Serial.print("; X_L="); Serial.print(X_L); Serial.print(" Y_L="); Serial.println(Y_L);*/
    if (X_R != 0 || Y_R != 0 || X_L != 0 || Y_L != 0) {

      double oldTargetX, oldTargetY, oldTargetZ;
      if ( sq(targetX)  <= (sq(forearm + arm) - sq(targetY) - sq(targetZ)) ) {
        oldTargetX = targetX;
        targetX = targetX + X_R;
      } else {
        if (targetX > 0) {
          targetX = oldTargetX - 1;
        } else {
          targetX = oldTargetX + 1;
        }
      }
      if ( sq(targetY) <= (sq(forearm + arm) - sq(targetX) - sq(targetZ)) ) {
        oldTargetY = targetY;
        targetY = targetY + X_L;
      } else {
        if (targetY > 0) {
          targetY = oldTargetY - 1;
        } else {
          targetY = oldTargetY + 1;
        }
      }
      if ( sq(targetZ) <= (sq(forearm + arm) - sq(targetX) - sq(targetY)) ) {
        oldTargetZ = targetZ;
        targetZ = targetZ + Y_R;
      } else {
        if (targetZ > 0) {
          targetZ = oldTargetZ - 1;
        } else {
          targetZ = oldTargetZ + 1;
        }
      }
      sigma_5 = (sigma_5 + Y_L);
Serial.print("X="); Serial.print(targetX); 
Serial.print("; Y="); Serial.print(targetY);
Serial.print("; Z="); Serial.println(targetZ);
      moveArm(targetX, targetY, targetZ, 20, 30);
    }
    //delay(10);    
  //oled();

}
