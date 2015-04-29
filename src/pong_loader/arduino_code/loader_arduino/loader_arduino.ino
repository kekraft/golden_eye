/* 
 * Purpose: Loads a ball into a hopper.
            Designed to run on a teensie 3.1.
            The pwm controlled motor does nothing until 
              it receives a ros message telling motor to spin.
            Motor spins until a sensor switch is hit.
            
   Date: 4/29/15
   Author: Kory Kraft
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>


ros::NodeHandle nh;

std_msgs::String state_msg;
ros::Publisher pub_state("loader/state", &state_msg);

const int switch_pin = 5;
const int led_pin = 13;
const int motor_pin = 9;

const int motor_speed = 50;

//bool last_reading;
//long last_debounce_time=0;
//long debounce_delay=50;
//bool published = true;

void run_motor(int motor_speed){
  analogWrite(motor_pin, motor_speed);
}

void stop_motor(){
  analogWrite(motor_pin, 0);
}

void load_ball(){
  // While we haven't seen the switch be high, 
  //   run the motor
  
  while(digitalRead(switch_pin) != HIGH){
    run_motor(motor_speed); 


    delay(10);
    break; // put this in here to test, don't want to kill the motor
  }
  
  stop_motor();
  
}

void load_cmd_cb(const std_msgs::Bool& cmd_msg){
  
  if (cmd_msg.data == true){
    // load ball
    state_msg.data = "loading";
    pub_state.publish(&state_msg);
    digitalWrite(led_pin, HIGH);
    
    
    load_ball();
    
    state_msg.data = "loaded";
    pub_state.publish(&state_msg);
    digitalWrite(led_pin,LOW);
    
  }
  
}

ros::Subscriber<std_msgs::Bool> load_sub("load_cmd", load_cmd_cb);

void setup()
{
  nh.initNode();
  nh.advertise(pub_state);
  
  //initialize an LED output pin for status purposes
  //init input pin for switch input and motor control pin
  pinMode(led_pin, OUTPUT);
  pinMode(switch_pin, INPUT);
  pinMode(motor_pin, OUTPUT);
 
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
