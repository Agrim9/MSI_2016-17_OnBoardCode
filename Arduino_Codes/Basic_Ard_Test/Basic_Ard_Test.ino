
//Arduino Mega and Leonardo chips only support some pins for receiving data back from the RoboClaw
//This is because only some pins of these boards support PCINT interrupts or are UART receivers.
//Mega: 0,10,11,12,13,14,15,17,19,50,51,52,53,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15
//Leonardo: 0,8,9,10,11

//Arduino Due currently does not support SoftwareSerial. Only hardware uarts can be used, pins 0/1, 14/15, 16/17 or 18/19.

//Note: Most Arduinos do not support higher baudrates rates than 115200.  Also the arduino hardware uarts generate 57600 and 115200 with a
//relatively large error which can cause communications problems.

//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include "BMSerial.h"
#include "RoboClaw.h"

//Roboclaw Address
#define address1 0x80
#define address2 0x81

//Velocity PID coefficients
#define Kp 1.0
#define Ki 0.5
#define Kd 0.25
#define qpps 44000

//Definte terminal for display. Use hardware serial pins 0 and 1
BMSerial terminal(0,1);

//Setup communcaitions with roboclaw. Use pins 10 and 11 with 10ms timeout
RoboClaw roboclaw1(19,18,10000);
RoboClaw roboclaw2(19,18,10000);

int Rm = 2;
int Lm = 5;


void setup() {  
  pinMode(2, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  
  pinMode(5, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);
  terminal.begin(9600);
  roboclaw1.begin(9600);
  roboclaw2.begin(9600);
}
int targetAngle=0;
char c=' ';
int incomingValue=0;
int motorspeed=100;
int motorleftdir = 0;
int motorrightdir = 0;
int select = 2; // 1, 2, 3 left right , both
void loop(){ 
if (Serial.available()){
    c= Serial.read();
  
    /*char buffer[] = {' ',' ',' ',' ',' ',' ',' ',' ',' '}; // Receive up to 7 bytes
    //while (!Serial.available());                   // Wait for characters
    Serial.readBytesUntil('o', buffer, 9);
    incomingValue = atoi(buffer);              // incomingValue= first input from serial input
    if (c=='-') incomingValue=-1*incomingValue;
    else incomingValue=(c-48)*10+incomingValue;*/
    if (c == 'w'){
      bothMotorForward();
    }
    else if (c == 's'){
      bothMotorBackward();
    }
    else if (c == 'x'){
      stopBothMotor();
    }
    else if (c == 'a'){
      targetAngle = -30;
    }
    else if (c == 'd'){
      targetAngle = 30;
    }
    else if (c == 'e'){
      targetAngle = 0;
    }
    terminal.print("Target angle is : ");
    terminal.println(targetAngle);
    terminal.print("Millisecond : ");
    terminal.println(millis());
    int savedangle = targetAngle;
  //steerLeft(64);
    
  
}
actuatorLoop();
}
void actuatorLoop(){
  if(motorleftdir == 1){
    analogWrite(Rm, motorspeed);
    digitalWrite(22, LOW); 
    digitalWrite(23, HIGH);
  }
  if(motorleftdir == -1){
    analogWrite(Rm, motorspeed);
    digitalWrite(22, HIGH); 
    digitalWrite(23, LOW);
  }
  if(motorleftdir == 0){
    analogWrite(Rm, 0);
  }
  
  
  if(motorrightdir == 1){
    analogWrite(Lm, motorspeed);
    digitalWrite(28, LOW); 
    digitalWrite(29, HIGH);
  }
  if(motorrightdir == -1){
    analogWrite(Lm, motorspeed);
    digitalWrite(28, HIGH); 
    digitalWrite(29, LOW);
  }
  if(motorrightdir == 0){
    analogWrite(Lm, 0);
  }
  gotoAngle(targetAngle);

}

void gotoAngle(int target){
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;

  struct AngleMap {
   int minA = -90;
   int maxA = 90;

   int minEnc = -450;
   int maxEnc = 450;
   
  }angleMap; 
  
  if(select == 2){
    int enc1pos = roboclaw1.ReadEncM1(address1, &status1, &valid1);
        int theta=targetAngle;//type in the value for the theta...it could be between -90 and +90
  int ne=5.34*theta;
  if(enc1pos<ne-80)
  { float f_ang = ne-80;
    float vel = turn(enc1pos,enc1pos+25,1);
    roboclaw1.BackwardM1(address1,vel);
    delay(100);
   //terminal.println(enc1pos); 
  }
 else if(enc1pos>ne+80)                   // +5 encoder reading is the tolerance
{ float f_ang = ne+100;
  float vel =  turn(enc1pos,enc1pos-25,1);
  roboclaw1.ForwardM1(address1,-vel);
  delay(100);
  //terminal.println(enc1pos);
} 
else
{
  roboclaw1.BackwardM1(address1,0);
  delay(100);
  //terminal.println(enc1pos);
}
    
  }
  
  if(select == 2){
    int enc2pos = roboclaw1.ReadEncM2(address1, &status2, &valid2);
        int theta=-targetAngle;//type in the value for the theta...it could be between -90 and +90
  int ne=5.34*theta;
  if(enc2pos<ne-80)
  { float f_ang = ne-100;
    float vel = turn(enc2pos,enc2pos+25,1.2);
    roboclaw1.ForwardM2(address1,vel);
    delay(100);
   //terminal.println(enc2pos); 
  }
 else if(enc2pos>ne+80)                   // +5 encoder reading is the tolerance
{ float f_ang = ne+100;
  float vel = turn(enc2pos,enc2pos-25,1.2);
  roboclaw1.BackwardM2(address1,-vel);
  delay(100);
  //terminal.println(enc2pos);
} 
else
{ 
  roboclaw1.ForwardM2(address1,0);
  delay(100);
  //terminal.println(enc2pos);
}
  }
  
if(select == 2){
    int enc1pos = roboclaw2.ReadEncM1(address2, &status1, &valid1);
        int theta=+targetAngle;//type in the value for the theta...it could be between -90 and +90
  int ne=5.34*theta;
  if(enc1pos<ne-80)
  { float f_ang = ne-100;
    float vel =  turn(enc1pos,enc1pos+25,1);
    roboclaw2.BackwardM1(address2,vel);
    delay(100);
  //terminal.println(enc1pos); 
  }
 else if(enc1pos>ne+80)                   // +5 encoder reading is the tolerance
{ float f_ang = ne+100;
  float vel =  turn(enc1pos,enc1pos-25,1);
  roboclaw2.ForwardM1(address2,-vel);
  delay(100);
  //terminal.println(enc1pos);
} 
else
{
  roboclaw2.BackwardM1(address2,0);
  delay(100);
  //terminal.println(enc1pos);
}
    
  }
if(select == 2){
    int enc2pos = roboclaw2.ReadEncM2(address2, &status2, &valid2);
        int theta=-targetAngle;//type in the value for the theta...it could be between -90 and +90
  int ne=5.34*theta;
  if(enc2pos<ne-80)
  { float f_ang = ne-100;
    float vel =  turn(enc2pos,enc2pos+25,1.2);
    roboclaw2.ForwardM2(address2,vel);
    delay(100);
   //terminal.println(enc2pos); 
  }
 else if(enc2pos>ne+80)                   // +5 encoder reading is the tolerance
{ float f_ang = ne+100;
  float vel =  turn(enc2pos,enc2pos-25,1.2);
  roboclaw2.BackwardM2(address2,-vel);
  delay(100);
  //terminal.println(enc2pos);
} 
else
{
  roboclaw2.ForwardM2(address2,0);
  delay(100);
  //terminal.println(enc2pos);
}
    
  }

}
float turn(float initial,float targeted,float k_value){
  float c = targeted - initial;
  float vel = k_value*c;
  return vel;
}
void bothMotorForward(){
  motorleftdir = 1;
  motorrightdir = -1;
  Serial.println("Moving Forward");
}
void bothMotorBackward(){
  motorleftdir = -1;
  motorrightdir = 1;
  Serial.println("Moving Backward");
}
void stopBothMotor(){
  motorrightdir = 0;
  motorleftdir = 0;
  Serial.println("Motors Stopped");
}
/*void PrintStatus(){
  uint8_t status1,status2,status3,status4,status5,status6,status7,status8;
  bool valid1,valid2,valid3,valid4,valid5,valid6,valid7,valid8;
  
  //Read all the data from Roboclaw before displaying on terminal window
  //This prevents the hardware serial interrupt from interfering with
  //reading data using software serial.
  int32_t enc1= roboclaw1.ReadEncM1(address1, &status1, &valid1);
  int32_t enc2 = roboclaw1.ReadEncM2(address1, &status2, &valid2);
  int32_t speed1 = roboclaw1.ReadSpeedM1(address1, &status3, &valid3);
  int32_t speed2 = roboclaw1.ReadSpeedM2(address1, &status4, &valid4);
  int32_t enc3= roboclaw2.ReadEncM1(address2, &status5, &valid5);
  int32_t enc4 = roboclaw2.ReadEncM2(address2, &status6, &valid6);
  int32_t speed3 = roboclaw2.ReadSpeedM1(address2, &status7, &valid7);
  int32_t speed4 = roboclaw2.ReadSpeedM2(address2, &status8, &valid8);

  terminal.print("Encoder1:");
  if(valid1){
    terminal.print(enc1);
    terminal.print(" ");
    terminal.print(status1,HEX);
    terminal.print(" ");
  }
  else{
    terminal.print("invalid ");
  }
  terminal.print("Encoder2:");
  if(valid2){
    terminal.print(enc2);
    terminal.print(" ");
    terminal.print(status2,HEX);
    terminal.print(" ");
  }
  else{
    terminal.print("invalid ");
  }
  terminal.print("Speed1:");
  if(valid3){
    terminal.print(speed1,HEX);
    terminal.print(" ");
  }
  else{
    terminal.print("invalid ");
  }
  terminal.print("Speed2:");
  if(valid4){
    terminal.print(speed2,HEX);
    terminal.print(" ");
  }
  else{
    terminal.print("invalid ");
  }
  terminal.print("Encoder3:");
  if(valid5){
    terminal.print(enc3);
    terminal.print(" ");
    terminal.print(status5,HEX);
    terminal.print(" ");
  }
  else{
    terminal.print("invalid ");
  }
  terminal.print("Encoder4:");
  if(valid6){
    terminal.print(enc4);
    terminal.print(" ");
    terminal.print(status6,HEX);
    terminal.print(" ");
  }
  else{
    terminal.print("invalid ");
  }
  terminal.print("Speed3:");
  if(valid7){
    terminal.print(speed3,HEX);
    terminal.print(" ");
  }
  else{
    terminal.print("invalid ");
  }
  terminal.print("Speed4:");
  if(valid8){
    terminal.print(speed4,HEX);
    terminal.print(" ");
  }
  else{
    terminal.print("invalid ");
  }
  terminal.println();

}*/
