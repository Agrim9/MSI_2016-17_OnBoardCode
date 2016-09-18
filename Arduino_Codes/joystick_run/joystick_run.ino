#define USE_USBCON
#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>


ros::NodeHandle nh;
std_msgs::Float64 feedback_msg;
std_msgs::String str_msg;
ros::Publisher fb_pub("/rover/ard_feedback", &feedback_msg);
ros::Publisher str_pub("/rover/ard_talk", &str_msg);

float targetAngle1;
float targetAngle2;
float targetAngle3;
float targetAngle4;
float motorspeed1;
float motorspeed2;
float newmotorspeed2;
float motorspeed3;
float motorspeed4;
float motorspeed5;
float newmotorspeed5;
float motorspeed6;
int motorleftdir;
int motorrightdir;

void setDrive(float* data) {

  motorspeed1 = data[0];
  motorspeed2 = data[1];
  motorspeed3 = data[2];
  motorspeed4 = data[3];
  motorspeed5 = data[4];
  motorspeed6 = data[5];
  if (motorspeed1 < -10){
    motorspeed1 = -motorspeed1;
    bothMotorBackward();
  }
  else if (motorspeed1 > 10){
    bothMotorForward();
  }
  else {
    stopBothMotor();
  }
}

void mobCallback(const std_msgs::Float64MultiArray& msg){

  float* inp = msg.data;
  setDrive(inp);

  targetAngle1 = msg.data[6];
  targetAngle4 = -msg.data[7];
  targetAngle2 = -msg.data[8];
  targetAngle3 = msg.data[9];
  
  publishString("In callback");

} 

ros::Subscriber<std_msgs::Float64MultiArray> mob_sub("/rover/ard_directives", &mobCallback);

void setup() {  
  pinMode(2, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);

  pinMode(3, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);

  pinMode(4, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);

  pinMode(5, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);

  pinMode(6, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);

  pinMode(7, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);


  targetAngle1 = 0;
  targetAngle2 = 0;
  targetAngle3 = 0;
  targetAngle4 = 0;
  motorspeed1 = 0;
  motorspeed2 = 0;
  newmotorspeed2 = 0;
  motorspeed3 = 0;
  motorspeed4 = 0;
  motorspeed5 = 0;
  newmotorspeed5 = 0;
  motorspeed6 = 0;
  motorleftdir = 0;
  motorrightdir = 0;

  nh.initNode();
  nh.subscribe(mob_sub);
  nh.advertise(fb_pub);
  nh.advertise(str_pub);
}

void loop(){ 

  actuatorLoop();
  publishValue(motorspeed1);
  publishValue(motorspeed2);
  publishValue(motorspeed3);
  publishValue(motorspeed4);
  publishValue(motorspeed5);
  publishValue(motorspeed6);
  nh.spinOnce();
  delay(10);
}

void newVelocities() {
  
  if (targetAngle1 > 0){
    int c = targetAngle1 / 50;
    int d = 1 - c ;
    newmotorspeed2 = motorspeed1*d;
    newmotorspeed5 = motorspeed4*d; 
  }
  else if (targetAngle1 < 0){
    int c = targetAngle1 / 50;
    int d = 1 + c ;
    newmotorspeed2 = motorspeed1*d;
    newmotorspeed5 = motorspeed4*d; 
  }
  else {
    newmotorspeed2 = motorspeed1;
    newmotorspeed5 = motorspeed4;
  }
}
void actuatorLoop() {
  
  newVelocities();
  
  if(motorleftdir == 1){
    analogWrite(2, motorspeed1);
    digitalWrite(22, LOW); 
    digitalWrite(23, HIGH);
    analogWrite(3, newmotorspeed2);
    digitalWrite(24, LOW); 
    digitalWrite(25, HIGH);
    analogWrite(4, motorspeed3);
    digitalWrite(26, LOW); 
    digitalWrite(27, HIGH);
  }
  if(motorleftdir == -1){
    analogWrite(2, motorspeed1);
    digitalWrite(22, HIGH); 
    digitalWrite(23, LOW);
    analogWrite(3, newmotorspeed2);
    digitalWrite(24, HIGH); 
    digitalWrite(25, LOW);
    analogWrite(4, motorspeed3);
    digitalWrite(26, HIGH); 
    digitalWrite(27, LOW);
  }
  if(motorleftdir == 0){
    analogWrite(2, 0);
    analogWrite(3, 0);
    analogWrite(4, 0);
  }

  if(motorrightdir == 1){
    analogWrite(5, motorspeed4);
    digitalWrite(28, LOW); 
    digitalWrite(29, HIGH);
    analogWrite(6, newmotorspeed5);
    digitalWrite(30, LOW); 
    digitalWrite(31, HIGH);
    analogWrite(7, motorspeed6);
    digitalWrite(32, LOW); 
    digitalWrite(33, HIGH);
  }
  if(motorrightdir == -1){
    analogWrite(5, motorspeed4);
    digitalWrite(28, HIGH); 
    digitalWrite(29, LOW);
    analogWrite(6, newmotorspeed5);
    digitalWrite(30, HIGH); 
    digitalWrite(31, LOW);
    analogWrite(7, motorspeed6);
    digitalWrite(32, HIGH); 
    digitalWrite(33, LOW);
  }
  if(motorrightdir == 0){
    analogWrite(5, 0);
    analogWrite(6, 0);
    analogWrite(7, 0);
  } 
}


void bothMotorForward(){
  motorleftdir = 1;
  motorrightdir = -1;
}

void bothMotorBackward(){
  motorleftdir = -1;
  motorrightdir = 1;
}

void stopBothMotor(){
  motorrightdir = 0;
  motorleftdir = 0;
}

void publishValue(float val) {
  feedback_msg.data = val;
  fb_pub.publish(&feedback_msg);
}

void publishString(char* str) {
  str_msg.data = str;
  str_pub.publish(&str_msg);
}
