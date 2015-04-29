/**
Purpose: This arduino file handles all the mechanisms for the robot pong launcher.
          It utilizes rosserial to communicate back with the master pc that has a ros core.
          The arudino acts as a node looking for a message that contains three different motor speed values.
          
          Once a motor speeds topic is received, the motors are then set to those speeds. 
          
          Hall effect sensors are used to verify the motors are at the right speed, and a PID controller is used
          to tune them. Once every motor is set to the correct speed, the launhcer node then sends a topic saying
          that it is ready for a ball to be loaded.
         
 Date: 4/28/15
 
 Authors: Hayden Conner and Kory Kraft

*/

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

//#include <PID_v1.h>
#define int2 2
#define int3 3
#define int4 4

int MotorA = 6;
int MotorB = 9;
int MotorC = 10;
int val = 0;

//Hall effect sensor variables
//int halla_old = 0, hallb_old = 0, hallc_old = 0;
//double rota = 0, rotb = 0, rotc =0;
//int timeold = 0;
//volatile int halla = 0, hallb = 0, hallc = 0;
//volatile int ina, inb, inc;

//PID variables
//char motorSelect = 0;
//double Setpointa, Inputa, Outputa;
//double Setpointb, Inputb, Outputb;
//double Setpointc, Inputc, Outputc;
//double Kp = 1, Kd = 0.25, Ki = .05;

//PID aPID(&Inputa, &Outputa, &Setpointa, Kp, Kd, Ki, DIRECT);
//PID bPID(&Inputb, &Outputb, &Setpointb, Kp, Kd, Ki, DIRECT);
//PID cPID(&Inputc, &Outputc, &Setpointc, Kp, Kd, Ki, DIRECT);

// ROS Stuff
ros::NodeHandle  nh;

std_msgs::String state_msg;
ros::Publisher state_pub("launcher/state", &state_msg);

void setMotorSpeed(int motor, double speed){
  // Set the motor specified to the speed given.
  bool is_set = false;
  while (!is_set){
    
    // use hall effect and pid to tune
    
    is_set = true;
    
  }
  
}

void motor_controls_cb(const geometry_msgs::Vector3& cmd_msg){

  // publish command saying that we are adjusting
  std_msgs::String str;
  str.data = "adjusting";
  state_pub.publish(&str);
  
  // Get motor speed A, B, and C from the message
  double motorSpeedA = cmd_msg.x;
  double motorSpeedB = cmd_msg.y;
  double motorSpeedC = cmd_msg.z;
    
  // Set each motor speed, verification done in setting motor speed
  setMotorSpeed(MotorA, motorSpeedA);
  setMotorSpeed(MotorB, motorSpeedB);
  setMotorSpeed(MotorC, motorSpeedC);
  
  // publish command alerting the master system that we are ready
//  std_msgs::String str;
  str.data = "ready";
  state_pub.publish(&str);
  
}

ros::Subscriber<geometry_msgs::Vector3> motor_sub("launcher/motor_vel/", motor_controls_cb);

void setup(){
  //Pinmodes for the motor's PWM
  pinMode(MotorA,OUTPUT);
  pinMode(MotorB,OUTPUT);
  pinMode(MotorC,OUTPUT);
	
  //Defining the interupt pins as interrupt, required by the Teensy 3.1
  pinMode(int2, INPUT);
  pinMode(int3, INPUT);
  pinMode(int4, INPUT);
  
//  aPID.SetMode(AUTOMATIC);
//  bPID.SetMode(AUTOMATIC);
//  cPID.SetMode(AUTOMATIC);

//  attachInterrupt(int2, hall_isra, RISING);
//  attachInterrupt(int3, hall_isrb, RISING);
//  attachInterrupt(int4, hall_isrc, RISING);

  nh.initNode();
  nh.subscribe(motor_sub);
}

void loop(){

  // ros necesisity
  nh.spinOnce();
  
//  if( millis() - timeold > 20){
//    rota = (halla - halla_old)/(millis()-timeold);
//    rotb = (hallb - hallb_old)/(millis()-timeold);
//    rotc = (hallc - hallc_old)/(millis()-timeold);
//    halla_old = halla;
//    hallb_old = hallb;
//    hallc_old = hallc;
//  }
  
  delay(1);

}

//void hall_isra(){
//  if (ina > (millis()+1)){
//    halla++;
//  }
//}
//
//void hall_isrb(){
//  if (inb > (millis()+1)){
//    hallb++;
//  }
//}  
//
//void hall_isrc(){
//  if (inc > (millis()+1)){
//    hallc++;
//  }
//}


