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

//Pin definitions
//Motor stuff
int A1top = 3;
int A2top = 4;
int A3top = 5;
int A1bot = 7;
int A2bot = 8;
int A3bot = 11;

int B1top = 6;
int B2top = 9;
int B3top = 10;
int B1bot = 12;
int B2bot = 13;
int B3bot = A0;

int C1top = A6;
int C2top = A7;
int C3top = A8;
int C1bot = A1;
int C2bot = A2;
int C3bot = A3;
//Hall effect sensors
int ha1 = 2;
int ha2 = A5;
int ha3 = A9;

int hb1 = A10;
int hb2 = A11;
int hb3 = A12;

int hc1 = 29;
int hc2 = 30;
int hc3 = 31;

//Hall effect sensor variables
volatile int timeoldA = 0, timeoldB = 0, timeoldC = 0;
volatile int hallA1 = 0, hallA2 = 0, hallA3 = 0;
volatile int hallB1 = 0, hallB2 = 0, hallB3 = 0;
volatile int hallC1 = 0, hallC2 = 0, hallC3 = 0;

//Commute variables
int 1top, 2top, 3top;
int 1bot, 2bot, 3bot;
int dir
char motor

//PID variables
//char motorSelect = 0;
double Setpointa = 0, Inputa;
double Setpointb = 0, Inputb;
double Setpointc = 0, Inputc;
volatile double Outputa = 0, Outputb = 0, Outputc = 0;

double Kp = 1, Kd = 0.25, Ki = .05;

PID aPID(&Inputa, &Outputa, &Setpointa, Kp, Kd, Ki, DIRECT);
PID bPID(&Inputb, &Outputb, &Setpointb, Kp, Kd, Ki, DIRECT);
PID cPID(&Inputc, &Outputc, &Setpointc, Kp, Kd, Ki, DIRECT);

// ROS Stuff
ros::NodeHandle  nh;

std_msgs::String state_msg;
ros::Publisher state_pub("launcher/state", &state_msg);

void setMotorSpeed(int motor, double speed){
  // Set the motor specified to the speed given.
  bool is_set = false;
  while (!is_set){
    Inputa = curspeedA;
    Inputb = curspeedB;
    Inputc = curspeedC;

    Setpointa = motorSpeedA;
    Setpointb = motorSpeedB;
    Setpointc = motorSpeedC;
    // use hall effect and pid to tune
    aPID.compute();
    bPID.compute();
    cPID.compute();

	if ((curspeedA - oldspeedA) < 0.05) && ((curspeedB - oldspeedB) < 0.05) && ((curspeedC - oldspeedC) < 0.05){
		is_set = true;
	}
  
}

void commute(motor, hall1, hall2, hall3, dir){

	switch (motor) {
	case 1:
		1top = A1top;
		2top = A2top;
		3top = A3top;
		1bot = A1bot;
		2bot = A2bot;
		3bot = A3bot;
		spd = Outputa;
		break;
	case 2:
		1top = B1top;
		2top = B2top;
		3top = B3top;
		1bot = B1bot;
		2bot = B2bot;
		3bot = B3bot;
		spd = Outputb;
		break;
	case 3:
		1top = C1top;
		2top = C2top;
		3top = C3top;
		1bot = C1bot;
		2bot = C2bot;
		3bot = C3bot;
		spd = Outputc;
		break;
	}

	if ( dir == 1 ){ //Forward
		//Case 1
		if ((hall1 == HIGH) && (hall2 == LOW) && (hall3 == HIGH)){
			analogWrite(1top, spd);
			analogWrite(2top, 0);
			analogWrite(3top, 0);
			digitalWrite(1bot, HIGH);
			digitalWrite(2bot, HIGH);
			digitalWrite(3bot, LOW);
		}
		//Case 2
		else if ((hall1 == HIGH) && (hall2 == LOW) && (hall3 == LOW)){
			analogWrite(1top, spd);
			analogWrite(2top, 0);
			analogWrite(3top, 0);
			digitalWrite(1bot, HIGH);
			digitalWrite(2bot, LOW);
			digitalWrite(3bot, HIGH);
		}
		//Case 3
		else if ((hall1 == HIGH) && (hall2 == HIGH) && (hall3 == LOW)){
			analogWrite(1top, 0);
			analogWrite(2top, spd);
			analogWrite(3top, 0);
			digitalWrite(1bot, LOW);
			digitalWrite(2bot, HIGH);
			digitalWrite(3bot, HIGH);
		}
		//Case 4
		else if ((hall1 == LOW) && (hall2 == HIGH) && (hall3 == LOW)){
			analogWrite(1top, 0);
			analogWrite(2top, spd);
			analogWrite(3top, 0);
			digitalWrite(1bot, HIGH);
			digitalWrite(2bot, HIGH);
			digitalWrite(3bot, LOW);
		}
		//Case 5
		else if ((hall1 == LOW) && (hall2 == HIGH) && (hall3 == HIGH)){
			analogWrite(1top, 0);
			analogWrite(2top, 0);
			analogWrite(3top, spd);
			digitalWrite(1bot, HIGH);
			digitalWrite(2bot, LOW);
			digitalWrite(3bot, HIGH);
		}
		//Case 6
		else if ((hall1 == LOW) && (hall2 == LOW) && (hall3 == HIGH)){
			analogWrite(1top, 0);
			analogWrite(2top, 0);
			analogWrite(3top, spd);
			digitalWrite(1bot, LOW);
			digitalWrite(2bot, HIGH);
			digitalWrite(3bot, HIGH);
		}
	}
	else{ //Reverse
		//Case 1
		if ((hall1 == HIGH) && (hall2 == LOW) && (hall3 == HIGH)){
			analogWrite(1top, 0);
			analogWrite(2top, spd);
			analogWrite(3top, 0);
			digitalWrite(1bot, HIGH);
			digitalWrite(2bot, HIGH);
			digitalWrite(3bot, LOW);
		}
		//Case 2
		else if ((hall1 == HIGH) && (hall2 == LOW) && (hall3 == LOW)){
			analogWrite(1top, spd);
			analogWrite(2top, 0);
			analogWrite(3top, 0);
			digitalWrite(1bot, LOW);
			digitalWrite(2bot, LOW);
			digitalWrite(3bot, HIGH);
		}
		//Case 3
		else if ((hall1 == HIGH) && (hall2 == HIGH) && (hall3 == LOW)){
			analogWrite(1top, 0);
			analogWrite(2top, 0);
			analogWrite(3top, spd);
			digitalWrite(1bot, LOW);
			digitalWrite(2bot, HIGH);
			digitalWrite(3bot, HIGH);
		}
		//Case 4
		else if ((hall1 == LOW) && (hall2 == HIGH) && (hall3 == LOW)){
			analogWrite(1top, spd);
			analogWrite(2top, 0);
			analogWrite(3top, 0);
			digitalWrite(1bot, HIGH);
			digitalWrite(2bot, HIGH);
			digitalWrite(3bot, LOW);
		}
		//Case 5
		else if ((hall1 == LOW) && (hall2 == HIGH) && (hall3 == HIGH)){
			analogWrite(1top, spd);
			analogWrite(2top, 0);
			analogWrite(3top, 0);
			digitalWrite(1bot, HIGH);
			digitalWrite(2bot, LOW);
			digitalWrite(3bot, HIGH);
		}
		//Case 6
		else if ((hall1 == LOW) && (hall2 == LOW) && (hall3 == HIGH)){
			analogWrite(1top, 0);
			analogWrite(2top, spd);
			analogWrite(3top, 0);
			digitalWrite(1bot, LOW);
			digitalWrite(2bot, HIGH);
			digitalWrite(3bot, HIGH);
		}
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
	pinMode(A1top,OUTPUT);
	pinMode(A2top,OUTPUT);
	pinMode(A3top,OUTPUT);
	pinMode(A1bot,OUTPUT);
	pinMode(A2bot,OUTPUT);
	pinMode(A3bot,OUTPUT);


	pinMode(B1top,OUTPUT);
	pinMode(B2top,OUTPUT);
	pinMode(B3top,OUTPUT);
	pinMode(B1bot,OUTPUT);
	pinMode(B2bot,OUTPUT);
	pinMode(B3bot,OUTPUT);

	pinMode(C1top,OUTPUT);
	pinMode(C2top,OUTPUT);
	pinMode(C3top,OUTPUT);
	pinMode(C1bot,OUTPUT);
	pinMode(C2bot,OUTPUT);
	pinMode(C3bot,OUTPUT);


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

	nh.initNode();
	nh.subscribe(motor_sub);
}

void loop(){
  // ros necesisity
  nh.spinOnce();
  delay(1);
}

void A(){
	hallA1 = digitalRead(ha1);
	hallA2 = digitalRead(ha2);
	hallA3 = digitalRead(ha3);

	time = millis();
	curspeedA = 1/(3 * (time - timeold));
	timeoldA = time;
	oldspeedA = curspeedA;

	commute(1, hallC1, hallC2, hallC3, 1);	
}

void B() {
	hallB1 = digitalRead(hb1);
	hallB2 = digitalRead(hb2);
	hallB3 = digitalRead(hb3);

	time = millis();
	curspeedB = 1/(3 * (time - timeold));
	timeoldB = time;
	oldspeedB = curspeedB;

	commute(2, hallC1, hallC2, hallC3, 1);
}

void C() {
	hallC1 = digitalRead(hc1);
	hallC2 = digitalRead(hc2);
	hallC3 = digitalRead(hc3);

	time = millis();
	curspeedC = 1/(3 * (time - timeold));
	timeoldC = time;
	oldspeedC = curspeedC;
	
	commute(3, hallC1, hallC2, hallC3, 1);
}