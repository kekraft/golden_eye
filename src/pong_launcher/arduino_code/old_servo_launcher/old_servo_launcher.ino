/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo yaw_servo;
Servo pitch_servo;

void yaw_servo_cb( const std_msgs::UInt16& cmd_msg){
  yaw_servo.write(cmd_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

void pitch_servo_cb(const std_msgs::UInt16& cmd_msg){
  pitch_servo.write(cmd_msg.data);
  digitalWrite(13, HIGH-digitalRead(13));
}


ros::Subscriber<std_msgs::UInt16> yaw_sub("launcher/yaw", yaw_servo_cb);
ros::Subscriber<std_msgs::UInt16> pitch_sub("launcher/pitch",pitch_servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(yaw_sub);
  nh.subscribe(pitch_sub);
  
  yaw_servo.attach(9); //attach it to pin 9
  pitch_servo.attach(11);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
