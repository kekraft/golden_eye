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

#include <PID_v1.h>

// PWM Pin definitions *** Need to check***
//Motor stuff, enum class
const int motorA_pwm_pin = 3;
const int motorB_pwm_pin = 4;
const int motorC_pwm_pin = 5;

// initialize servo (arduino handles low level)
Servo motorA;
Servo motorB;
Servo motorC;

//struct Motor_Struct {
//  int top1;
//  int top2;
//  int top3;
//  int bot1;
//  int bot2;
//  int bot3;
//  
//  int h1;
//  int h2;
//  int h3;
//  
//  volatile double curSpeed;
//  
//  volatile int timeold;
//  
//  volatile int hall1, hall2, hall3;
//  
//  double pid_setpoint, pid_input, pid_output;
//};

//Hall effect sensors
int ha1 = 2;
int ha2 = A5;
int ha3 = A9;

int hb1 = A10;
int hb2 = A11;
int hb3 = A13;

int hc1 = 29;
int hc2 = 30;
int hc3 = 31;

//Hall effect sensor variables
volatile double curSpeedA = 0.0, curSpeedB = 0.0, curSpeedC = 0.0;
volatile double oldSpeedA = 0.0, oldSpeedB = 0.0, oldSpeedC = 0.0;
volatile int timeoldA = 0, timeoldB = 0, timeoldC = 0, time = 0;
volatile int hallA1 = 0, hallA2 = 0, hallA3 = 0;
volatile int hallB1 = 0, hallB2 = 0, hallB3 = 0;
volatile int hallC1 = 0, hallC2 = 0, hallC3 = 0;

//Commute variables
int top_1, top_2, top_3;
int bot_1, bot_2, bot_3;
int spd;
int dir;
char motr;

//PID variables
//char motorSelect = 0;
double Setpointa = 0, Inputa;
double Setpointb = 0, Inputb;
double Setpointc = 0, Inputc;
// The PID initializer was NOT liking the volatile qualifier

// PWM values for each motor
// output for pid controller to measure against
int Outputa = 0, Outputb = 0, Outputc = 0;
volatile double Outputa_temp = 0, Outputb_temp = 0, Outputc_temp = 0;

double Kp = 1, Kd = 0.25, Ki = .05;

PID aPID(&Inputa, (double*) &Outputa, &Setpointa, Kp, Kd, Ki, DIRECT);
PID bPID(&Inputb, (double*) &Outputb, &Setpointb, Kp, Kd, Ki, DIRECT);
PID cPID(&Inputc, (double*) &Outputc, &Setpointc, Kp, Kd, Ki, DIRECT);

// ROS Stuff
ros::NodeHandle  nh;

std_msgs::String state_msg;
ros::Publisher pub_state("/launcher/state", &state_msg);

void stop_motor(int motor){
 
}

void motor_controls_cb(const geometry_msgs::Vector3& cmd_msg){

	// publish command saying that we are adjusting
	std_msgs::String str;
	str.data = "Adjusting Motor Vals";
	pub_state.publish(&str);

	// Get motor speed A, B, and C from the message and set
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
 motorA.write(0);
 motorB.write(0);
 motorC.write(0);

 //delay 1 second,  some speed controllers may need longer
 delay(1000); 
}

void setup(){
	// Attach servos
	motorA.attach(motorA_pwm_pin);
	motorB.attach(motorB_pwm_pin);
	motorC.attach(motorC_pwm_pin);
 
	// Arm servos (set speeds to 0);
        arm_servos();
	
	//Defining the interupt pins as interrupt, pinMode is required by the Teensy 3.1
	pinMode(ha1, INPUT);
	pinMode(ha2, INPUT);
	pinMode(ha3, INPUT);
	attachInterrupt(ha1, A, CHANGE);
	attachInterrupt(ha2, A, CHANGE);
	attachInterrupt(ha3, A, CHANGE);
	pinMode(hb1, INPUT);
	pinMode(hb2, INPUT);
	pinMode(hb3, INPUT);
	attachInterrupt(ha1, B, CHANGE);
	attachInterrupt(ha2, B, CHANGE);
	attachInterrupt(ha3, B, CHANGE);
	pinMode(hc1, INPUT);
	pinMode(hc2, INPUT);
	pinMode(hc3, INPUT);
	attachInterrupt(ha1, C, CHANGE);
	attachInterrupt(ha2, C, CHANGE);
	attachInterrupt(ha3, C, CHANGE);

	//PID mode set
	aPID.SetMode(AUTOMATIC);
	bPID.SetMode(AUTOMATIC);
	cPID.SetMode(AUTOMATIC);

        // Init ros stuff
        nh.getHardware()->setBaud(9600);
	nh.initNode();
        nh.advertise(pub_state);
	nh.subscribe(motor_sub);
        nh.subscribe(pid_sub);
}

void loop(){
  // ros necesisity
  nh.spinOnce();
  // don't think this is neccessary
//  delay(1);
  
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
  motorA.write(Outputa);
  motorB.write(Outputb);
  motorC.write(Outputc);
  
}

void A(){
        // Turn off motor output
//        stop_motor(MotorA);
  
        // Updating speed based off current hall reading
	hallA1 = digitalRead(ha1);
	hallA2 = digitalRead(ha2);
	hallA3 = digitalRead(ha3);
	time = millis();
	curSpeedA = 1/(3 * (time - timeoldA)); // just remove the 3
        // 3 digital rotations per single mechanical rotation
        curSpeedA *= 3;
	timeoldA = time;
	oldSpeedA = curSpeedA;
}

void B() {
        // Turn off motor output
//        stop_motor(MotorB);
  
        // Updating speed based off current hall readings
	hallB1 = digitalRead(hb1);
	hallB2 = digitalRead(hb2);
	hallB3 = digitalRead(hb3);

	time = millis();
	curSpeedB = 1/(3 * (time - timeoldB)); // just remove the 3
        // 3 digital rotations per single mechanical rotation
        curSpeedB *= 3;
	timeoldB = time;
	oldSpeedB = curSpeedB;
}

void C() {
        // Turn off motor output
//        stop_motor(MotorC);
        
        // Updating speed based off current hall reading
	hallC1 = digitalRead(hc1);
	hallC2 = digitalRead(hc2);
	hallC3 = digitalRead(hc3);

	time = millis();
	curSpeedC = 1/(3 * (time - timeoldC)); // just remove the 3
        // 3 digital rotations per single mechanical rotation
        curSpeedC *= 3;
	timeoldC = time;
	oldSpeedC = curSpeedC;
}

