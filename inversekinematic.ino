int pulses1;                              
int encoderA1 = 3;
int encoderB1 = 2;
const int pwm1 = 11;                  
const int M11 = 5;                       
const int M12 = 4;
int angle1 =0;
int angle1out=0;

int pulses2;                              
int encoderA2 = 18;
int encoderB2 = 19;
const int pwm2 = 12;                      
const int M21 = 6;                       
const int M22 = 7;
const int pin5V=22;
int angle2 =0;
int angle2out=0;

int pulses3;                              
int encoderA3 = 20;
int encoderB3 = 21;
const int pwm3 = 10;                      
const int M31 = 8;                       
const int M32 = 9;
int angle3 =0;
int angle3out=0;

#define total 2400                        

#include "math.h"
using namespace std;

//time difference
unsigned long time;
 float dt=0;
 float prev=0;

//PID_A
float e_A=0;
float I_A=0;
float D_A=0;
float prev_eA=0;
int oA=0;

//PID_B
float e_B=0;
float I_B=0;
float D_B=0;
float prev_eB=0;
int oB=0;

//PID_C
float e_C=0;
float I_C=0;
float D_C=0;
float prev_eC=0;
int oC=0;

//user input position
String px;
String py;
String pz;
int xx=0;
int yy=0;
int zz=0;
float x=0;
float y=0;
float z=0;
float ang1=0;
float ang2=0;
float ang3=0;
float ppx=0;
float ppy=0;
float ppz=0;

void setup() {
  
  Serial.begin(9600);
  pinMode(pin5V,OUTPUT);
  digitalWrite(pin5V, HIGH);
  pinMode(pwm1, OUTPUT);
  pinMode(M11, OUTPUT);
  pinMode(M12, OUTPUT);
  analogWrite(pwm1, 0);                     
  digitalWrite(M11, HIGH);
  digitalWrite(M12, HIGH);
  pinMode(encoderA1, INPUT);
  digitalWrite(encoderA1,HIGH);
  pinMode(encoderB1, INPUT);
  digitalWrite(encoderB1,HIGH);
  attachInterrupt(0, A1_CHANGE, CHANGE);
  attachInterrupt(1, B1_CHANGE, CHANGE);

  pinMode(pwm2, OUTPUT);
  pinMode(M21, OUTPUT);
  pinMode(M22, OUTPUT);
  analogWrite(pwm2, 0);                     
  digitalWrite(M21, HIGH);
  digitalWrite(M22, HIGH);
  pinMode(encoderA2, INPUT);
  digitalWrite(encoderA2,HIGH);
  pinMode(encoderB2, INPUT);
  digitalWrite(encoderB1,HIGH);
  attachInterrupt(4, A2_CHANGE, CHANGE);
  attachInterrupt(5, B2_CHANGE, CHANGE);

  pinMode(pwm3, OUTPUT);
  pinMode(M31, OUTPUT);
  pinMode(M32, OUTPUT);
  analogWrite(pwm3, 0);                     
  digitalWrite(M31, HIGH);
  digitalWrite(M32, HIGH);
  pinMode(encoderA3, INPUT);
  digitalWrite(encoderA3,HIGH);
  pinMode(encoderB3, INPUT);
  digitalWrite(encoderB3,HIGH);
  attachInterrupt(2, A3_CHANGE, CHANGE);
  attachInterrupt(3, B3_CHANGE, CHANGE);
  Serial.print("key in position in cm: Px,Py,Pz");

}//setup

void loop(){
  while(Serial.available() == 0){
     
  }
  
  if (Serial.available() >0){
    px  = Serial.readStringUntil(',');
    Serial.read();
    py = Serial.readStringUntil(',');
    Serial.read();
    pz  = Serial.readStringUntil('\0'); 
    xx=px.toInt();
    yy=py.toInt();
    zz=pz.toInt();
    Serial.println();
    Serial.print(xx);
    Serial.print(" ");
    Serial.print(yy);
    Serial.print(" ");
    Serial.println(zz);  
  }
    while(1){
      x=xx*0.01;
      y=yy*0.01;
      z=zz*0.01;
    ang1=(atan(0.075/sqrt(pow(x,2)+pow(y,2)-0.005625))+atan(-x/y))*6.66666667;
    ang2=(atan((10*z)/sqrt(1-100*pow(z,2))))*6.66666667; //in pulse
    ang2=90*6.66666667;
    timer();
    PID_A(ang1); 
    PID_B(ang2); 
    PID_C(ang3); 
    
    angle1out = pulses1*0.15;  
    Serial.print("1.) " );  
    Serial.print(angle1out);
    Serial.print(",  ");
    Serial.print(e_A);
    Serial.print(",  ");
    Serial.println(oA);
    
    angle2out = pulses2*0.15;
    Serial.print("2.) " );     
    Serial.print(angle2out);
    Serial.print(",  ");
    Serial.print(e_B);
    Serial.print(",  ");
    Serial.println(oB);
    
    angle3out = pulses3*0.15; 
    Serial.print("3.) " );    
    Serial.print(angle3out);
    Serial.print(",  ");
    Serial.print(e_C);
    Serial.print(",  ");
    Serial.println(oC);
    
    //print the px py pz   
    pos();
    Serial.print("Current (Px,Py,Pz)=(" );    
    Serial.print(ppx);
    Serial.print(",  ");
    Serial.print(ppy);
    Serial.print(",  ");
    Serial.print(ppz);
    Serial.println(")  ");

    if (Serial.available() >0){
      break;
    }
    
}}

void timer()
{
  time = millis();
  dt=(time-prev)*0.001;
  prev=time;
  delay(10);
}

void PID_A(int setpoint)
{
  e_A=setpoint-pulses1;
  I_A=I_A+e_A*dt;
  D_A=(e_A-prev_eA)/dt;
  oA=e_A*0.7+0.03*D_A;
  prev_eA=e_A;
  delay(dt);
  
   if (oA<-255 ){
    oA=-255 ;     
   }
   else if(oA>255){
     oA=255;   
    }
    else{
      oA=oA;       
    }
      if (oA<0){
      oA=-1*oA;
      oA=oA*0.67058+81;
      analogWrite(pwm1,oA);
      digitalWrite(M11,LOW);
      digitalWrite(M12,HIGH);
      if (oA <82)
     {
        analogWrite(pwm1,0);   
     }      
    }
    else{
      oA=oA*0.67058+81;
      analogWrite(pwm1,oA);
      digitalWrite(M11,HIGH);
      digitalWrite(M12,LOW);
      if (oA <82)
     {
        analogWrite(pwm1,0);   
     }
    }  
}

void PID_B(int setpoint)
{
  e_B=setpoint-pulses2;
  I_B=I_B+e_B*dt;
  D_B=(e_B-prev_eB)/dt;
  oB=e_B*0.75+0.035*D_B;
  prev_eB=e_B;
  delay(dt);
  
   if (oB<-255 ){
    oB=-255 ;     
   }
   else if(oB>255){
     oB=255;   
    }
    else{
      oB=oB;       
    }
      if (oB<0){
      oB=-1*oB;
      oB=oB*0.67058+81;
      analogWrite(pwm2,oB);
      digitalWrite(M21,HIGH);
      digitalWrite(M22,LOW); 
      if (oB <82)
     {
        analogWrite(pwm2,0);   
     }
    }
    else{
      oB=oB*0.67058+81;
      analogWrite(pwm2,oB);
      digitalWrite(M21,LOW);
      digitalWrite(M22,HIGH);
      if (oB <82)
     {
        analogWrite(pwm2,0);   
     }
    }  
}

void PID_C(int setpoint)
{
  e_C=setpoint-pulses3;
  I_C=I_C+e_C*dt;
  D_C=(e_C-prev_eC)/dt;
 oC=e_C*0.75+0.035*D_C;
  prev_eC=e_C;
  delay(dt);
  
   if (oC<-255 ){
    oC=-255 ;     
   }
   else if(oC>255){
     oC=255;   
    }
    else{
      oC=oC;       
    }
      if (oC<0){
      oC=-1*oC;
      oC=oC*0.67058+81;
      analogWrite(pwm3,oC);
      digitalWrite(M31,HIGH);
      digitalWrite(M32,LOW);  
      if (oC <82)
     {
        analogWrite(pwm3,0);   
     }    
    }
    else{
      oC=oC*0.67058+81;
      analogWrite(pwm3,oC);
      digitalWrite(M31,LOW);
      digitalWrite(M32,HIGH);
      if (oC<82)
     {
        analogWrite(pwm3,0);   
     }
    }
  
}
void pos()
{
  float d1 =angle1out*0.01745;
  float d2 =angle2out*0.01745;
  float d3 =angle3out*0.01745;
  ppx=7.5*sin(d1)+10*cos(d1)*cos(d2);
  ppy=-7.5*cos(d1)+10*sin(d1)*sin(d2);
  ppz=10*sin(d2);

  
  
}
void A1_CHANGE(){
  if( digitalRead(encoderB1) == 0 ) {
    if ( digitalRead(encoderA1) == 0 ) { // A fell, B is low
      pulses1--;
    } else {// A rose, B is low
      pulses1++;
    }
  }else {
    if ( digitalRead(encoderA1) == 0 ) { // B fell, A is high
      pulses1++;
    } else {// B rose, A is high
      pulses1--; 
    }
  }
}
void B1_CHANGE(){
 if ( digitalRead(encoderA1) == 0 ) {
    if ( digitalRead(encoderB1) == 0 ) { // B fell, A is low
      pulses1++; 
    } else {// B rose, A is low
      pulses1--; 
    }
 } else {
    if ( digitalRead(encoderB1) == 0 ) {// B fell, A is high
      pulses1--; 
    } else {// B rose, A is high
      pulses1++; 
    }
  }
}

void A2_CHANGE(){
 if( digitalRead(encoderB2) == 0 ) {
    if ( digitalRead(encoderA2) == 0 ) { // A fell, B is low
      pulses2--;
    } else {// A rose, B is low
      pulses2++;
    }
  }else {
    if ( digitalRead(encoderA2) == 0 ) { // B fell, A is high
      pulses2++;
    } else {// B rose, A is high
      pulses2--; 
    }
  }
}
void B2_CHANGE(){
  if ( digitalRead(encoderA2) == 0 ) {
    if ( digitalRead(encoderB2) == 0 ) { // B fell, A is low
      pulses2++; 
    } else {// B rose, A is low
      pulses2--; 
    }
 } else {
    if ( digitalRead(encoderB2) == 0 ) {// B fell, A is high
      pulses2--; 
    } else {// B rose, A is high
      pulses2++; 
    }
  }
}

void A3_CHANGE(){
 if( digitalRead(encoderB3) == 0 ) {
    if ( digitalRead(encoderA3) == 0 ) { // A fell, B is low
      pulses3--;
    } else {// A rose, B is low
      pulses3++;
    }
  }else {
    if ( digitalRead(encoderA3) == 0 ) { // B fell, A is high
      pulses3++;
    } else {// B rose, A is high
      pulses3--; 
    }
  }
}
void B3_CHANGE(){
  if ( digitalRead(encoderA3) == 0 ) {
    if ( digitalRead(encoderB3) == 0 ) { // B fell, A is low
      pulses3++; 
    } else {// B rose, A is low
      pulses3--; 
    }
 } else {
    if ( digitalRead(encoderB3) == 0 ) {// B fell, A is high
      pulses3--; 
    } else {// B rose, A is high
      pulses3++; 
    }
  }
}
