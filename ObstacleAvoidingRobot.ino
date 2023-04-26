/* Run Wheels and Servo
 */

#define DECODE_NEC
#include <IRremote.hpp>
#define IR_Pin 12
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

const int pingPin = 11; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 13; // Echo Pin of Ultrasonic Sensor

 const int RSPD1 = 150;        //Right Wheel PWM.  Change this value so your car will go roughly straight
 const int LSPD1 = 150;        //Left Wheel PWM
 const int RSPD2 = 150;        //Right Wheel PWM
 const int LSPD2 = 150;        //Left Wheel PWM
 
 const int in1 = 6;   // Connet to L298H
 const int in2 = 7;   // Left
 const int eA = 9;

 const int in3 = 4;  //Right
 const int in4 = 5;
 const int eB = 10; 
 
 int gain=12;
 int delCntr=0; 
 volatile long cntrL, cntrR;
 volatile long LIntTime, RIntTime;
 long stopTime, inch1, inch2, inches, duration;

void setup() 
{

  pinMode(IR_Pin, INPUT);
  Serial.begin(9600);

  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  IrReceiver.begin(IR_Pin, ENABLE_LED_FEEDBACK);
  
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(eA,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(eB,OUTPUT);

  digitalWrite(in2,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(eA,LOW);
  
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
  digitalWrite(eB,LOW);

  attachInterrupt(digitalPinToInterrupt(2), leftWhlCnt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), rightWhlCnt, CHANGE);
   
  cntrR = 0;
  cntrL = 0;
  LIntTime = 0;
  RIntTime = 0;

  stopTime = micros() + 10*1000000;
  
}

void loop(){

  
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
   
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
   
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  inches = microsecondsToInches(duration); // inches = microsecondsToInches(duration);

  Serial.print(inches);
  Serial.print("in, ");
  Serial.println();


  if (inches <= 10){
    pause();
    right();
    
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pingPin, LOW);
    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);
    inch1 = microsecondsToInches(duration);
    
    int a = inch1;
    Serial.println(a);
    delay(500);
    backleft();
    delay(500);
    left();
    
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pingPin, LOW);
    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);
    inch2 = microsecondsToInches(duration);
    
    int b = inch2;
    Serial.println(b);
    delay(500);
    backright();
    delay(500);
    
    if(a > b) {
      right();
    } else {
      left();
      
    }
  
    forward();
  }
  
if (IrReceiver.decode()) {
    IrReceiver.printIRResultShort(&Serial);
    Serial.println(IrReceiver.decodedIRData.command);
    
    if(IrReceiver.decodedIRData.command == 64){
      forward();
    }
    
    if(IrReceiver.decodedIRData.command == 67){
      pause();
    }
    
    if(IrReceiver.decodedIRData.command == 68){
      backward();
    }    
    IrReceiver.resume(); 
  } 
  
   delay(100);
  long tmpLcntr, tmpRcntr;
  tmpLcntr = cntrL;
  tmpRcntr = cntrR;
  delCntr = abs(tmpLcntr - tmpRcntr);
  
  if(tmpLcntr > tmpRcntr)
  {
    analogWrite(eB,RSPD1);
    analogWrite(eA,max(LSPD1-int(gain*delCntr+.5),0));
  }
  else if(tmpLcntr < tmpRcntr)
  {
    analogWrite(eA,LSPD1);
    analogWrite(eB,max(RSPD1-int(gain*delCntr+.5),0));
   }
  else
  {
    analogWrite(eB,RSPD1);
    analogWrite(eA,LSPD1);
  }
}

void leftWhlCnt()
{
  long intTime = micros();
  if(intTime > LIntTime + 1000L)
  {
    LIntTime = intTime;
    cntrL++;
  }
}

void rightWhlCnt()  // Complete this ISR
{
  long intTime = micros();
  if(intTime > RIntTime + 1000L)
  {
    RIntTime = intTime;
    cntrR++;
  }

}

void pause(){
  
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  analogWrite(eA, 0);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
  analogWrite(eB, 0);
  delay(500);
}

void forward(){
  pause();
  //left motor
  digitalWrite(in2, LOW);
  digitalWrite(in1, HIGH);
  //right motor
  digitalWrite(in4, LOW);
  digitalWrite(in3, HIGH);
  
  //speed control
  analogWrite(eB, RSPD1); //0-255
  analogWrite(eA, LSPD1); //0-255
}

void backward(){
  pause();
  //left motor
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  //right motor
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
  //speed control
  analogWrite(eB, RSPD1); //0-255
  analogWrite(eA, LSPD1); //0-255
}

void right(){
  
  digitalWrite(in2, LOW);
  digitalWrite(in1, HIGH);
  
  volatile long oldcntrL = cntrL;
  volatile long oldcntrR = cntrR;
  cntrL = 0;
  cntrR = 0;
  while (cntrL < (cntrR + 34)) { 
    analogWrite(eB, 0);
    analogWrite(eA, LSPD1); 
  }
  cntrL = oldcntrL;
  cntrR = oldcntrR;

  pause();
}

void backleft(){
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
  volatile long oldcntrL = cntrL;
  volatile long oldcntrR = cntrR;
  cntrL = 0;
  cntrR = 0;
  while (cntrL < (cntrR + 34)) { 
    analogWrite(eB, 0);
    analogWrite(eA, LSPD1); 
  }
  cntrL = oldcntrL;
  cntrR = oldcntrR;

  pause();
}

void backright(){
  
 digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  
  volatile long oldcntrL = cntrL;
  volatile long oldcntrR = cntrR;
  cntrL = 0;
  cntrR = 0;
  while (cntrR < (cntrL + 34)) { 
    analogWrite(eA, 0);
    analogWrite(eB, RSPD1); 
  }
  cntrL = oldcntrL;
  cntrR = oldcntrR;

  pause();
}

void left(){
  
  digitalWrite(in4, LOW);
  digitalWrite(in3, HIGH);
  
  volatile long oldcntrL = cntrL;
  volatile long oldcntrR = cntrR;
  cntrL = 0;
  cntrR = 0;
  while (cntrR < (cntrL + 34)) { 
    analogWrite(eA, 0);
    analogWrite(eB, RSPD1); 
  }
  cntrL = oldcntrL;
  cntrR = oldcntrR;

  pause();
}


long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
 return microseconds / 29 / 2;
}
