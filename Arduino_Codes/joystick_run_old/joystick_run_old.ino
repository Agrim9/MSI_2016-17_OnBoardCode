#define USE_USBCON
#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include "BMSerial.h"
#include "RoboClaw.h"

//Roboclaw Address
#define address1 0x80
#define address2 0x81

//Setup communications with roboclaw. Use pins 10 and 11 with 10ms timeout
RoboClaw roboclaw1(19,18,10000);
RoboClaw roboclaw2(19,18,10000);

int Rm = 2;
int Lm = 5;

ros::NodeHandle  nh;
std_msgs::Float64 feedback_msg;
ros::Publisher fb_pub("/rover/ard_feedback", &feedback_msg);

int targetAngle1=0;
int targetAngle2=0;
int targetAngle3=0;
int targetAngle4=0;
int motorspeed=0;
int motorleftdir = 0;
int motorrightdir = 0;

void mobCallback(const std_msgs::Float64MultiArray& msg){

  motorspeed = msg.data[0];
  if (motorspeed < -10){
    motorspeed = -motorspeed;
    bothMotorBackward();
  }
  else if (motorspeed > 10){
    bothMotorForward();
  }
  else {
    stopBothMotor();
  }

  targetAngle1 = msg.data[6];
  targetAngle4 = -msg.data[7];
  targetAngle2 = -msg.data[8];
  targetAngle3 = msg.data[9];

  actuatorLoop();
} 

ros::Subscriber<std_msgs::Float64MultiArray> mob_sub("/rover/ard_directives", &mobCallback);

void setup() {  
  pinMode(2, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);

  pinMode(5, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);

  roboclaw1.begin(9600);
  roboclaw2.begin(9600);

  nh.initNode();
  nh.subscribe(mob_sub);
  nh.advertise(fb_pub);
}

void loop(){ 
  nh.spinOnce();
  delay(1);
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
  gotoAngle();

}

void gotoAngle(){

  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;

  int enc1pos = roboclaw1.ReadEncM1(address1, &status1, &valid1);
  int theta1 = targetAngle1; //type in the value for the theta...it could be between -90 and +90

  feedback_msg.data = theta1;
  fb_pub.publish(&feedback_msg);

  int ne1 = 5.34 * theta1;
  if (enc1pos < ne1 - 80)
  {
    float vel1 = turn(enc1pos, enc1pos + 25, 1);
    roboclaw1.BackwardM1(address1, vel1);
    delay(10);
  }
  else if (enc1pos > ne1 + 80)              // +5 encoder reading is the tolerance
  {
    float vel1 =  turn(enc1pos, enc1pos - 25, 1);
    roboclaw1.ForwardM1(address1, -vel1);
    delay(10);
  }
  else
  {
    roboclaw1.BackwardM1(address1, 0);
    delay(10);
  }



  int enc2pos = roboclaw1.ReadEncM2(address1, &status2, &valid2);
  int theta2 = -targetAngle2; //type in the value for the theta...it could be between -90 and +90
  int ne2 = 5.34 * theta2;
  if (enc2pos < ne2 - 80)
  {
    float vel2 = turn(enc2pos, enc2pos + 25, 1);
    roboclaw1.ForwardM2(address1, vel2);
    delay(10);
  }
  else if (enc2pos > ne2 + 80)              // +5 encoder reading is the tolerance
  {
    float vel2 = turn(enc2pos, enc2pos - 25, 1);
    roboclaw1.BackwardM2(address1, -vel2);
    delay(10);
  }
  else
  {
    roboclaw1.ForwardM2(address1, 0);
    delay(10);
  }



  int enc3pos = roboclaw2.ReadEncM1(address2, &status1, &valid1);
  int theta3 = +targetAngle3; //type in the value for the theta...it could be between -90 and +90
  int ne3 = 5.34 * theta3;
  if (enc3pos < ne3 - 100)
  {
    float vel3 =  turn(enc3pos, enc3pos + 25, 1);
    roboclaw2.BackwardM1(address2, vel3);
    delay(10);
  }
  else if (enc3pos > ne3 + 100)              // +5 encoder reading is the tolerance
  {
    float vel3 =  turn(enc3pos, enc3pos - 25, 1);
    roboclaw2.ForwardM1(address2, -vel3);
    delay(10);
  }
  else
  {
    roboclaw2.BackwardM1(address2, 0);
    delay(10);
  }


  int enc4pos = roboclaw2.ReadEncM2(address2, &status2, &valid2);
  int theta4 = -targetAngle4; //type in the value for the theta...it could be between -90 and +90
  int ne4 = 5.34 * theta4;
  if (enc4pos < ne4 - 80)
  {
    float vel4 =  turn(enc4pos, enc4pos + 25, 1.2);
    roboclaw2.ForwardM2(address2, vel4);
    delay(10);
  }
  else if (enc4pos > ne4 + 80)              // +5 encoder reading is the tolerance
  {
    float vel4 =  turn(enc4pos, enc4pos - 25, 1.2);
    roboclaw2.BackwardM2(address2, -vel4);
    delay(10);
  }
  else
  {
    roboclaw2.ForwardM2(address2, 0);
    delay(10);
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
}
void bothMotorBackward(){
  motorleftdir = -1;
  motorrightdir = 1;
}
void stopBothMotor(){
  motorrightdir = 0;
  motorleftdir = 0;
}




