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



//Pin definitions
//Motor stuff, enum class
const int MotorA = 67;
const int MotorB = 68;
const int MotorC = 69;


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

double Outputa = 0, Outputb = 0, Outputc = 0;
volatile double Outputa_temp = 0, Outputb_temp = 0, Outputc_temp = 0;

double Kp = 1, Kd = 0.25, Ki = .05;

PID aPID(&Inputa, &Outputa, &Setpointa, Kp, Kd, Ki, DIRECT);
PID bPID(&Inputb, &Outputb, &Setpointb, Kp, Kd, Ki, DIRECT);
PID cPID(&Inputc, &Outputc, &Setpointc, Kp, Kd, Ki, DIRECT);

// ROS Stuff
ros::NodeHandle  nh;

std_msgs::String state_msg;
ros::Publisher pub_state("launcher/state", &state_msg);

// Takes the current state from the interrupt service routine and 
//  decides on the next commute state
//
// Speed has to be volatile bc needs to be persistent through service
//   routines
//
// Each speed is actually output from pid
void commute(int motor, int hall1, int hall2, int hall3, int sdir){
        
	switch (motor) {
	case MotorA:
		top_1 = A1top;
		top_2 = A2top;
		top_3 = A3top;
		bot_1 = A1bot;
		bot_2 = A2bot;
		bot_3 = A3bot;
		spd = Outputa_temp;
		break;
	case MotorB:
		top_1 = B1top;
		top_2 = B2top;
		top_3 = B3top;
		bot_1 = B1bot;
		bot_2 = B2bot;
		bot_3 = B3bot;
		spd = Outputb_temp;
		break;
	case MotorC:
		top_1 = C1top;
		top_2 = C2top;
		top_3 = C3top;
		bot_1 = C1bot;
		bot_2 = C2bot;
		bot_3 = C3bot;
		spd = Outputc_temp;
		break;
	}

	if ( dir == 1 ){ //Forward
		//Case 1
		if ((hall1 == HIGH) && (hall2 == LOW) && (hall3 == HIGH)){
			analogWrite(top_1, spd);
			analogWrite(top_2, 0);
			analogWrite(top_3, 0);
			digitalWrite(bot_1, HIGH);
			digitalWrite(bot_2, HIGH);
			digitalWrite(bot_3, LOW);
		}
		//Case 2
		else if ((hall1 == HIGH) && (hall2 == LOW) && (hall3 == LOW)){
			analogWrite(top_1, spd);
			analogWrite(top_2, 0);
			analogWrite(top_3, 0);
			digitalWrite(bot_1, HIGH);
			digitalWrite(bot_2, LOW);
			digitalWrite(bot_3, HIGH);
		}
		//Case 3
		else if ((hall1 == HIGH) && (hall2 == HIGH) && (hall3 == LOW)){
			analogWrite(top_1, 0);
			analogWrite(top_2, spd);
			analogWrite(top_3, 0);
			digitalWrite(bot_1, LOW);
			digitalWrite(bot_2, HIGH);
			digitalWrite(bot_3, HIGH);
		}
		//Case 4
		else if ((hall1 == LOW) && (hall2 == HIGH) && (hall3 == LOW)){
			analogWrite(top_1, 0);
			analogWrite(top_2, spd);
			analogWrite(top_3, 0);
			digitalWrite(bot_1, HIGH);
			digitalWrite(bot_2, HIGH);
			digitalWrite(bot_3, LOW);
		}
		//Case 5
		else if ((hall1 == LOW) && (hall2 == HIGH) && (hall3 == HIGH)){
			analogWrite(top_1, 0);
			analogWrite(top_2, 0);
			analogWrite(top_3, spd);
			digitalWrite(bot_1, HIGH);
			digitalWrite(bot_2, LOW);
			digitalWrite(bot_3, HIGH);
		}
		//Case 6
		else if ((hall1 == LOW) && (hall2 == LOW) && (hall3 == HIGH)){
			analogWrite(top_1, 0);
			analogWrite(top_2, 0);
			analogWrite(top_3, spd);
			digitalWrite(bot_1, LOW);
			digitalWrite(bot_2, HIGH);
			digitalWrite(bot_3, HIGH);
		}
	}
	else{ //Reverse
		//Case 1
		if ((hall1 == HIGH) && (hall2 == LOW) && (hall3 == HIGH)){
			analogWrite(top_1, 0);
			analogWrite(top_2, spd);
			analogWrite(top_3, 0);
			digitalWrite(bot_1, HIGH);
			digitalWrite(bot_2, HIGH);
			digitalWrite(bot_3, LOW);
		}
		//Case 2
		else if ((hall1 == HIGH) && (hall2 == LOW) && (hall3 == LOW)){
			analogWrite(top_1, spd);
			analogWrite(top_2, 0);
			analogWrite(top_3, 0);
			digitalWrite(bot_1, LOW);
			digitalWrite(bot_2, LOW);
			digitalWrite(bot_3, HIGH);
		}
		//Case 3
		else if ((hall1 == HIGH) && (hall2 == HIGH) && (hall3 == LOW)){
			analogWrite(top_1, 0);
			analogWrite(top_2, 0);
			analogWrite(top_3, spd);
			digitalWrite(bot_1, LOW);
			digitalWrite(bot_2, HIGH);
			digitalWrite(bot_3, HIGH);
		}
		//Case 4
		else if ((hall1 == LOW) && (hall2 == HIGH) && (hall3 == LOW)){
			analogWrite(top_1, spd);
			analogWrite(top_2, 0);
			analogWrite(top_3, 0);
			digitalWrite(bot_1, HIGH);
			digitalWrite(bot_2, HIGH);
			digitalWrite(bot_3, LOW);
		}
		//Case 5
		else if ((hall1 == LOW) && (hall2 == HIGH) && (hall3 == HIGH)){
			analogWrite(top_1, spd);
			analogWrite(top_2, 0);
			analogWrite(top_3, 0);
			digitalWrite(bot_1, HIGH);
			digitalWrite(bot_2, LOW);
			digitalWrite(bot_3, HIGH);
		}
		//Case 6
		else if ((hall1 == LOW) && (hall2 == LOW) && (hall3 == HIGH)){
			analogWrite(top_1, 0);
			analogWrite(top_2, spd);
			analogWrite(top_3, 0);
			digitalWrite(bot_1, LOW);
			digitalWrite(bot_2, HIGH);
			digitalWrite(bot_3, HIGH);
		}
	}
}

void stop_motor(int motor){
  // Select motor variables based off motor
  switch (motor) {
	case MotorA:
		top_1 = A1top;
		top_2 = A2top;
		top_3 = A3top;
		bot_1 = A1bot;
		bot_2 = A2bot;
		bot_3 = A3bot;
		spd = Outputa_temp;
		break;
	case MotorB:
		top_1 = B1top;
		top_2 = B2top;
		top_3 = B3top;
		bot_1 = B1bot;
		bot_2 = B2bot;
		bot_3 = B3bot;
		spd = Outputb_temp;
		break;
	case MotorC:
		top_1 = C1top;
		top_2 = C2top;
		top_3 = C3top;
		bot_1 = C1bot;
		bot_2 = C2bot;
		bot_3 = C3bot;
		spd = Outputc_temp;
		break;
	}

    // Turn everything off
    analogWrite(top_1, 0);
    analogWrite(top_2, 0);
    analogWrite(top_3, 0);
    digitalWrite(bot_1, LOW);
    digitalWrite(bot_2, LOW);
    digitalWrite(bot_3, LOW);
}

void motor_controls_cb(const geometry_msgs::Vector3& cmd_msg){

	// publish command saying that we are adjusting
	std_msgs::String str;
	str.data = "adjusting";
	pub_state.publish(&str);

	// Get motor speed A, B, and C from the message and set
        Setpointa = cmd_msg.x;
        Setpointb = cmd_msg.y;
        Setpointc = cmd_msg.z;

	// verification done in loop, running constantly through PID library

	// publish command alerting the master system that we are ready
	//  std_msgs::String str;
	str.data = "ready";
	pub_state.publish(&str);
}

ros::Subscriber<geometry_msgs::Vector3> motor_sub("launcher/motor_vel/", motor_controls_cb);

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

ros::Subscriber<geometry_msgs::Vector3> pid_sub("launcher/pid_val/", pid_controls_cb);

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
  
}



void A(){
        // Turn off motor output
        stop_motor(MotorA);
  
        // Updating speed based off current hall reading
	hallA1 = digitalRead(ha1);
	hallA2 = digitalRead(ha2);
	hallA3 = digitalRead(ha3);
	time = millis();
	curSpeedA = 1/(3 * (time - timeoldA));
	timeoldA = time;
	oldSpeedA = curSpeedA;

        // Potentially have the rest of this code in the loop as well
        
        // delay in isr, kind of dangerous
        delayMicroseconds(1);
         
        // commuting motor for motor velocity
        commute(MotorA, hallA1, hallA2, hallA3, 1);
}

void B() {
        // Turn off motor output
        stop_motor(MotorB);
  
        // Updating speed based off current hall readings
	hallB1 = digitalRead(hb1);
	hallB2 = digitalRead(hb2);
	hallB3 = digitalRead(hb3);

	time = millis();
	curSpeedB = 1/(3 * (time - timeoldB));
	timeoldB = time;
	oldSpeedB = curSpeedB;

        // delay in isr, kind of dangerous
        delayMicroseconds(1);
         
        // commuting motor for motor velocity
        commute(MotorB, hallB1, hallB2, hallB3, 1);
}

void C() {
        // Turn off motor output
        stop_motor(MotorC);
        
        // Updating speed based off current hall reading
	hallC1 = digitalRead(hc1);
	hallC2 = digitalRead(hc2);
	hallC3 = digitalRead(hc3);

	time = millis();
	curSpeedC = 1/(3 * (time - timeoldC));
	timeoldC = time;
	oldSpeedC = curSpeedC;

        // delay in isr, kind of dangerous
        delayMicroseconds(1);	

        // commuting motor for motor velocity
	commute(MotorC, hallC1, hallC2, hallC3, 1);
}

