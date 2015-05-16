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
ros::Publisher pub_state("/loader/state", &state_msg);

const int switch_pin = 2;
const int led_pin = 13;
const int motor_pin = 3;

const int motor_speed = 127;

//bool last_reading;
//long last_debounce_time=0;
//long debounce_delay=50;
//bool published = true;

/**
 * Divides a given PWM pin frequency by a divisor.
 * 
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 * 
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0
 *   - Pins 9 and 10 are paired on timer1
 *   - Pins 3 and 11 are paired on timer2
 * 
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 * 
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235060559/0#4
 */
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}


void run_motor(){
  analogWrite(motor_pin, motor_speed);
}

void stop_motor(){
  analogWrite(motor_pin, 0);
}

//void stop_motor_interrupt(){
//  if switc
//}



//void load_ball(){
//  // While we haven't seen the switch be high, 
////  //   run the motor
////  
////  while(digitalRead(switch_pin) != HIGH){
////    run_motor(motor_speed); 
////
////
////    delay(10); // run for .1 seconds.
//////    break; // put this in here to test, don't want to kill the motor
////  }
////  
////  stop_motor();
//  
//}

void load_cmd_cb(const std_msgs::Bool& cmd_msg){
  
  if (cmd_msg.data == true){
    // load ball
    state_msg.data = "loading";
    pub_state.publish(&state_msg);
    digitalWrite(led_pin, HIGH);
    
    run_motor();
//    load_ball();
    
//    delay(100);
    state_msg.data = "loaded";
    pub_state.publish(&state_msg);
    digitalWrite(led_pin,LOW);
    
  } else {
    stop_motor();
  } 
  
}

ros::Subscriber<std_msgs::Bool> load_sub("/loader/load_cmd", load_cmd_cb);

void setup()
{
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(pub_state);
  nh.subscribe(load_sub);
  
  //initialize an LED output pin for status purposes
  //init input pin for switch input and motor control pin
  pinMode(led_pin, OUTPUT);
  pinMode(switch_pin, INPUT);
  pinMode(motor_pin, OUTPUT);
  
  // set the motor to be initially off
  setPwmFrequency(motor_pin, 1); // highest pwm frequency
  stop_motor();
  
  // interrupt on the switch pin that 
  //  turns the motor off when the switch pin
  //   goes from low to high
  attachInterrupt(0, stop_motor, RISING);
  
  // load a ball on start
  run_motor();
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
