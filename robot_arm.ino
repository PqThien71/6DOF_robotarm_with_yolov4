#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <cvzone.h>

#include <SimpleKalmanFilter.h>
SimpleKalmanFilter locnhieu(2, 2, 0.1);
int analogSenSor = 0;
int analog_locnhieuSenSor = 0;
float volSenSor = 0;
const int stepPin = 4; //xung
const int dirPin = 3; // hướng

// $001
SerialData serialData;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define numOfValsRec                7
#define digitsPerValRec             3
#define MIN_PULSE_WIDTH             105
#define MAX_PULSE_WIDTH             535
#define MIN_PULSE_WIDTH_SMALL       100
#define MAX_PULSE_WIDTH_SMALL       517
#define FREQUENCY                   50

#define BtnBlack    A9
#define BtnYellow   A8
//board servo
#define motor0      0
#define motor1      1
#define motor2      2
#define motor3      3
#define motor4      4
#define motor5      5



int valsRec[numOfValsRec];
int angleServo[6];      // destination
int now_angleServo[6];  // now
int prv_angleServo[6];  // before
int packSite[6];
int stringLength = numOfValsRec*digitsPerValRec+1;
int counter =0 ;
bool counterStart = false;
String receiveString;
int pos;
int           runMode;
bool          recvData = true;
const int speed = 1;



int v_ref(int des, int prev) {
  int v;
  if (des - prev > 0) {
    v = speed;
  } else if (des - prev < 0) {
    v = -speed;
  } else {
    v = 0;
  }
  return v;
}

void moveOneStep(int dir){
//// high quay thuận kim đồng hồ, xylanh lùi ra
//// low quay ngược kim đồng hồ, xy lanh lao vào
  if (dir == 1){
    digitalWrite(dirPin,LOW);
    }
  else{
    digitalWrite(dirPin,HIGH);
    }
  digitalWrite(stepPin,HIGH);
  delay(3); 
  digitalWrite(stepPin,LOW); 
  delay(3); 
  }

void DEBUG(int x0, int x1, int x2, int x3, int x4, int x5) {
  Serial.print(x0); Serial.print(", ");
  Serial.print(x1); Serial.print(", ");
  Serial.print(x2); Serial.print(", ");
  Serial.print(x3); Serial.print(", ");
  Serial.print(x4); Serial.print(", ");
  Serial.print(x5); Serial.println();
}

float readPresSenSor(){
    analogSenSor = analogRead(A0);
    analog_locnhieuSenSor = locnhieu.updateEstimate(analogSenSor);
    volSenSor = (analog_locnhieuSenSor * 5.) / 1023.;
  return  volSenSor ;
  }
void moveAllMotor(int k){
//  prv_angleServo, now_angleServo, angleServo
//    now_angleServo[0] ++
// 
while (angleServo[0] != prv_angleServo[0] || angleServo[1] != prv_angleServo[1] || angleServo[2] != prv_angleServo[2] || angleServo[3] != prv_angleServo[3] || angleServo[4] != prv_angleServo[4] || angleServo[5] != prv_angleServo[5]){
  if (k ==0 ){
  int v0 = v_ref(angleServo[0],prv_angleServo[0]);
  while(now_angleServo[0] != angleServo[0]) {
    now_angleServo[0] += v0;
    moveMotor(motor0,now_angleServo[0]);
        Serial.print(now_angleServo[0]);
    delay(50);
  }
  }
 if( k ==1){
   int v1 = v_ref(angleServo[1],prv_angleServo[1]);
  while(now_angleServo[1] != angleServo[1]) {
    now_angleServo[1] += v1;
    moveMotor(motor1,now_angleServo[1]);
    delay(50);
    Serial.print(now_angleServo[1]);
    }
  }

  int v0 = v_ref(angleServo[0],prv_angleServo[0]);
  if (now_angleServo[0] != angleServo[0]) {
    now_angleServo[0] += v0;
  } else {
    prv_angleServo[0] = now_angleServo[0];
  }

  int v1 = v_ref(angleServo[1],prv_angleServo[1]);
  if (now_angleServo[1] != angleServo[1]) {
    now_angleServo[1] += v1;

  } else {
    prv_angleServo[1] = now_angleServo[1];

  }

  int v2 = v_ref(angleServo[2],prv_angleServo[2]);
  if (now_angleServo[2] != angleServo[2]) {
    now_angleServo[2] += v2;

  } else {
    prv_angleServo[2] = now_angleServo[2];

  }

  int v3 = v_ref(angleServo[3],prv_angleServo[3]);
  if (now_angleServo[3] != angleServo[3]) {
    now_angleServo[3] += v3;
  
  } else {
    prv_angleServo[3] = now_angleServo[3];

  }

  int v4 = v_ref(angleServo[4],prv_angleServo[4]);
  if (now_angleServo[4] != angleServo[4]) {
    now_angleServo[4] += v4;

  } else {
    prv_angleServo[4] = now_angleServo[4];

  }

  int v5 = v_ref(angleServo[5],prv_angleServo[5]);
  if (now_angleServo[5] != angleServo[5]) {
    now_angleServo[5] += v5;

  } else {
    prv_angleServo[5] = now_angleServo[5];

  }

   moveMotor(motor5,now_angleServo[5]);
   delay(10);
   moveMotor(motor4,now_angleServo[4]);
   delay(10);
   moveMotor(motor3,now_angleServo[3]);
   delay(10);
   moveMotor(motor2,now_angleServo[2]);
   delay(10);
   moveMotor(motor1,now_angleServo[1]);
   delay(10);
   moveMotor(motor0,now_angleServo[0]);
   delay(10);
   DEBUG(now_angleServo[0], now_angleServo[1], now_angleServo[2], now_angleServo[3], now_angleServo[4], now_angleServo[5]);
}
}
 
void moveToHome(int k){
  
    angleServo[0]= 0;
    angleServo[1]= 95;
    angleServo[2]= 94;
    angleServo[3]= 85;
    angleServo[4]= 90;
    angleServo[5]= 0;
    if (k==0){
      moveAllMotor(0);
    }
    if (k==1){
      moveAllMotor(1);
    }
} 

//moveServo
void moveMotor(int motorOut, int angle)
{  
  // Convert angle to pulse width
  int pulse_width;
  if (motorOut == 4 || motorOut == 5)
  {
    pulse_width = map(angle, 0, 180, MIN_PULSE_WIDTH_SMALL, MAX_PULSE_WIDTH);
  }
  else{
     pulse_width = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  }
//Control Motor
  pwm.setPWM(motorOut, 0, pulse_width);
}

void resetAll(){
  moveMotor(motor0, 0);
  delay(400);
  moveMotor(motor1, 95);
  delay(400);
  moveMotor(motor2, 94);
  delay(400);
  moveMotor(motor3, 85);
  delay(400);
  moveMotor(motor4, 90);
  delay(400);
  moveMotor(motor5, 0);
  delay(400);
}
  

void receiveData(){
  while(Serial.available())
  {
    char c = Serial.read();
    if (c=='$'){
      counterStart = true;
    }
    if (counterStart){
      if (counter< stringLength){
        receiveString = String(receiveString+c);
        counter++;
        }
      if (counter >= stringLength){
        for(int i =0; i <numOfValsRec; i++){
          int num = (i*digitsPerValRec)+1;
          valsRec[i] = receiveString.substring(num,num + digitsPerValRec).toInt();
        }
          receiveString = "";
          counter =0 ;
          counterStart = false;
      }
     }
   }
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  runMode = 0;
  Serial.setTimeout(100);
  pinMode(A0, INPUT);   

  //  ----Pin Mode Step motor-----//
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(2,OUTPUT);
  digitalWrite(2,LOW); 
  resetAll();
  angleServo[0]= now_angleServo[0]=  prv_angleServo[0]= 00;
  angleServo[1]= now_angleServo[1]=  prv_angleServo[1]= 95;
  angleServo[2]= now_angleServo[2]=  prv_angleServo[2]= 94;
  angleServo[3]= now_angleServo[3]=  prv_angleServo[3]= 85;
  angleServo[4]= now_angleServo[4]=  prv_angleServo[4]= 90;
  angleServo[5]= now_angleServo[5]=  prv_angleServo[5]= 0;
}

void loop() {
  receiveData();
  runMode = valsRec[0];
  Serial.print(runMode);
  if (runMode == 1) {
    packSite[0]=0;
    packSite[1]=180;
    packSite[2]=120;
    packSite[3]=85;
    packSite[4]=90;
    packSite[5]=0;
    }
  else if (runMode == 2){
    packSite[0]=180;
    packSite[1]=180;
    packSite[2]=120;
    packSite[3]=85;
    packSite[4]=90;
    packSite[5]=0;
    }
  else if (runMode == 3){
    packSite[0]=40;
    packSite[1]=130;
    packSite[2]=90;
    packSite[3]=85;
    packSite[4]=90;
    packSite[5]=0;
    }
  else if (runMode == 0){
    moveToHome(1);
  }
  if ( runMode == 1 || runMode == 2 || runMode == 3 ){
    angleServo[0]= valsRec[1];
    angleServo[1]= valsRec[2];
    angleServo[2]= valsRec[3]+30;
    angleServo[3]= valsRec[4];
    angleServo[4]= valsRec[5];
    angleServo[5]= valsRec[6];
    moveAllMotor(0);
    delay(1000);
    angleServo[2]=angleServo[2]-29 ;
    moveAllMotor(0);
    delay(1000);
    volSenSor = readPresSenSor();
      while (volSenSor < 3.3){
        ///3 .2 là số cần để đạt đủ 15kpa
    moveOneStep(0);
    analogSenSor = readPresSenSor();
    }
    // moveToHome(1);
    delay(5000);
    angleServo[2] = angleServo[2] +30;
    moveAllMotor(0);
    delay(500);
    angleServo[0]= packSite[0];
    angleServo[1]= packSite[1];
    angleServo[2]= packSite[2];
    angleServo[3]= packSite[3];
    angleServo[4]= packSite[4];
    angleServo[5]= packSite[5];
    moveAllMotor(0);
    
    readPresSenSor();
    while (volSenSor >2.6){
    moveOneStep(1);  
    volSenSor = readPresSenSor();
      }
     delay(1000);
    valsRec[0] = 0;
  }
}