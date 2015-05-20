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

//#if (ARDUINO >= 100)
// #include <Arduino.h>
//#else
// #include <WProgram.h>
//#endif

#include <PID_v1.h>

#include <Servo.h> 

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

// PWM Pin definitions *** Need to check***
//Motor stuff, enum class
const int motorA_pwm_pin = 3;
const int motorB_pwm_pin = 4;
const int motorC_pwm_pin = 5;

// initialize servo (arduino handles low level)
Servo motorA;
Servo motorB;
Servo motorC;

// Speed pwm motor vals
int motor_a_pwm_val = 25;
int motor_b_pwm_val = 0;
int motor_c_pwm_val = 0;

//Hall effect sensors
int ha1 = A8;
int ha2 = A7;
int ha3 = A6;

int hb1 = A5;
int hb2 = A4;
int hb3 = A3;

int hc1 = A2;
int hc2 = A1;
int hc3 = A0;

//Hall effect sensor variables
volatile double curSpeedA = 0.0, curSpeedB = 0.0, curSpeedC = 0.0;
volatile double oldSpeedA = 0.0, oldSpeedB = 0.0, oldSpeedC = 0.0;
volatile unsigned long timeoldA = 0, timeoldB = 0, timeoldC = 0, time = 0;
volatile int hallA1 = 0, hallA2 = 0, hallA3 = 0;
volatile int hallB1 = 0, hallB2 = 0, hallB3 = 0;
volatile int hallC1 = 0, hallC2 = 0, hallC3 = 0;

// hall ctr to send speed msg
volatile int hall_a_ctr = 0, hall_b_ctr = 0, hall_c_ctr = 0;
int loop_ctr = 0;


// PID variables
double Setpointa = 0, Inputa;
double Setpointb = 0, Inputb;
double Setpointc = 0, Inputc;
// The PID initializer was NOT liking the volatile qualifier

double Outputa = 0, Outputb = 0, Outputc = 0;
//volatile double Outputa_temp = 0, Outputb_temp = 0, Outputc_temp = 0;

double a_min = 25, a_max = 60;
double b_min = 0, b_max = 60;
double c_min = 0, c_max = 60;

double Kp = 1, Kd = 0.25, Ki = .05;

PID aPID(&Inputa, &Outputa, &Setpointa, Kp, Kd, Ki, DIRECT);
PID bPID(&Inputb, &Outputb, &Setpointb, Kp, Kd, Ki, DIRECT);
PID cPID(&Inputc, &Outputc, &Setpointc, Kp, Kd, Ki, DIRECT);

// ROS Stuff
ros::NodeHandle  nh;

std_msgs::String state_msg;
ros::Publisher pub_state("/launcher/state", &state_msg);
geometry_msgs::Vector3 speed_msg;
ros::Publisher pub_speed("/launcher/speed", &speed_msg);

void stop_motor(int motor){
 ;
}

void motor_controls_cb(const geometry_msgs::Vector3& cmd_msg){

	// publish command saying that we are adjusting
	std_msgs::String str;
	str.data = "Adjusting Motor Vals";
	pub_state.publish(&str);

	// Get motor speed A, B, and C from the message and set
//        motor_a_pwm_val = (int) cmd_msg.x;
//        motor_b_pwm_val = (int) cmd_msg.y;
//        motor_c_pwm_val = (int) cmd_msg.z;
        Setpointa = cmd_msg.x;
        Setpointb = cmd_msg.y;
        Setpointc = cmd_msg.z;


	// verification done in loop, running constantly through PID library

	// publish command alerting the master system that we are ready
	//  std_msgs::String str;
	str.data = "Motor Vals Set";
	pub_state.publish(&str);
}

ros::Subscriber<geometry_msgs::Vector3> motor_sub("/launcher/motor_vel", motor_controls_cb);

void pid_controls_cb(const geometry_msgs::Vector3& pid_msg){

	// publish command saying that we are adjusting
	std_msgs::String str;
	str.data = "Reading PID";
	pub_state.publish(&str);

	// Get motor speed A, B, and C from the message
	Kp = pid_msg.x;
	Kd = pid_msg.y;
	Ki = pid_msg.z;

	// publish command alerting the master system that we are ready
	//  std_msgs::String str;
	str.data = "Updated PID";
	pub_state.publish(&str);
}

ros::Subscriber<geometry_msgs::Vector3> pid_sub("/launcher/pid_val", pid_controls_cb);

// Arm all three servos at the same time
// Initially, not spinning
void arm_servos(){
 // arm the speed controller, modify as necessary for your ESC  
 motorA.write(25);
 motorB.write(0);
 motorC.write(0);

 //delay 1 second,  some speed controllers may need longer
 delay(5000); 
}

void setup(){  
      // Init ros stuff
        nh.getHardware()->setBaud(9600);
	nh.initNode();
        nh.advertise(pub_state);
        nh.advertise(pub_speed);
	nh.subscribe(motor_sub);
        nh.subscribe(pid_sub);
  
	// Attach servos
	motorA.attach(motorA_pwm_pin);
	motorB.attach(motorB_pwm_pin);
	motorC.attach(motorC_pwm_pin); 
	       
        //Defining the interupt pins as interrupt, pinMode is required by the Teensy 3.1
	pinMode(ha1, INPUT);
	pinMode(ha2, INPUT);
	pinMode(ha3, INPUT);
	attachInterrupt(ha1, motor_a_hall_change, CHANGE);
	attachInterrupt(ha2, motor_a_hall_change, CHANGE);
	attachInterrupt(ha3, motor_a_hall_change, CHANGE);
	pinMode(hb1, INPUT);
	pinMode(hb2, INPUT);
	pinMode(hb3, INPUT);
	attachInterrupt(hb1, motor_b_hall_change, CHANGE);
	attachInterrupt(hb2, motor_b_hall_change, CHANGE);
	attachInterrupt(hb3, motor_b_hall_change, CHANGE);
	pinMode(hc1, INPUT);
	pinMode(hc2, INPUT);
	pinMode(hc3, INPUT);
	attachInterrupt(hc1, motor_c_hall_change, CHANGE);
	attachInterrupt(hc2, motor_c_hall_change, CHANGE);
	attachInterrupt(hc3, motor_c_hall_change, CHANGE);
       
       //PID mode set
	aPID.SetMode(AUTOMATIC);
	bPID.SetMode(AUTOMATIC);
	cPID.SetMode(AUTOMATIC);

        aPID.SetOutputLimits(a_min, a_max);
        bPID.SetOutputLimits(b_min, b_max);
        cPID.SetOutputLimits(c_min, c_max);
        
        // Arm servos (set speeds to 0);
        arm_servos();
}

void loop(){
    // don't think this is neccessary
//  delay(1);

  if (hall_a_ctr > 500 || hall_b_ctr > 500 || hall_c_ctr > 500) { 
          speed_msg.x = curSpeedA;
          speed_msg.y = curSpeedB;
          speed_msg.z = curSpeedC;
          pub_speed.publish(&speed_msg);
          
          hall_a_ctr = 0;
          hall_b_ctr = 0;
          hall_c_ctr = 0;
          loop_ctr = 0;
  }  
  loop_ctr++;
  
   // from the hall effect sensors (speed)  
  //  the current speed is updated anytime the halls are read
  Inputa = curSpeedA;
  Inputb = curSpeedB;
  Inputc = curSpeedC;
    
  // use hall effect and pid to tune
  //  which updates OutputA, OutputB, OutputC
  aPID.Compute();
  bPID.Compute();
  cPID.Compute();
  
  // set pwm rates/speeds for motors
//  motorA.write(motor_a_pwm_val);
//  motorB.write(motor_b_pwm_val);
//  motorC.write(motor_c_pwm_val);
  motorA.write(Outputa);
  motorB.write(Outputb);
  motorC.write(Outputc);
  
  // ros necesisity
  nh.spinOnce();
  
}

void motor_a_hall_change(){  
/*
These ISR will increment whenever a hall sensor attached to their motor increments. Every 3 times it
will calculate a new speed value for the PID library to use.
*/
      if (hall_a_ctr % 3 == 0){
        time = micros();
        /// 3 in the numerator is there because there are three hall sensors
        // 1,000,000 in numerator is to convert microseconds to seconds
        // 14 is there because there are 14 hall readings per rotation
	curSpeedA = (3.0 * 1000000.0)/(14.0 * ((double)time - (double)timeoldA)); 
	timeoldA = time;
	oldSpeedA = curSpeedA;
      }
        hall_a_ctr ++;
        
        // debugging
//        Serial.print(hall_ctr, DEC);
        
}

void motor_b_hall_change() {  
        // Updating speed based off current hall readings
      if (hall_b_ctr % 3 == 0){
        time = micros();
	curSpeedB = (3.0 * 1000000.0)/(14.0 * ((double)time - (double)timeoldB)); // just remove the 3
	timeoldB = time;
	oldSpeedB = curSpeedB;
      }
       hall_b_ctr ++;
}

void motor_c_hall_change() {
//        // Updating speed based off current hall reading
      if (hall_c_ctr % 3 == 0){
        time = micros();
	curSpeedC = (3.0 * 1000000.0 )/(14.0 * ((double)time - (double)timeoldC));
	timeoldC = time;
	oldSpeedC = curSpeedC;
      }
        hall_c_ctr++;
}

