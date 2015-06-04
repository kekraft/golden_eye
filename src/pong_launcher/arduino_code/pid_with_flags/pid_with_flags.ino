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

//Speed calc variables
double curSpeedA = 0.0, curSpeedB = 0.0, curSpeedC = 0.0;
unsigned long timeoldA = 0, timeoldB = 0, timeoldC = 0, time = 0;

// hall ctr to send speed msg
volatile int hall_a_ctr = 0, hall_b_ctr = 0, hall_c_ctr = 0;
unsigned long last_calc_time_a = 0, last_calc_time_b = 0, last_calc_time_c = 0;
unsigned long calc_time = 0, calc_dt = 100; //Millis
unsigned long last_report_time = 0, report_dt = 1000; //millis


// PID variables
double setPointA = 0, setPointB = 0, setPointC = 0;
double Outputa = 0, Outputb = 0, Outputc = 0;
double min_out = 25, max_out = 180;
//Motor A
double kp_A = 0.001, kd_A = 0.05;
double kp_low_A = 0.01, kd_low_A = 0;
//Motor B
double kp_B = 0.002, kd_B = 0.05;
double kp_low_B = 0.05, kd_low_B = 0;
//Motor C
double kp_C = 0.002, kd_C = 0.05;
double kp_low_C = 0.05, kd_low_C = 0;

//PID function variables
double set_point = 0.0, current_speed = 0.0;
double last_speed_A = 0.0, last_speed_B = 0.0, last_speed_C = 0.0;
double p = 0, d = 0, i = 0;
int unsigned dt = 0;
//double i_thresh = 10;

// ROS Stuff
ros::NodeHandle  nh;

std_msgs::String state_msg;
ros::Publisher pub_state("/launcher/state", &state_msg);
geometry_msgs::Vector3 speed_msg;
ros::Publisher pub_speed("/launcher/speed", &speed_msg);

void motor_controls_cb(const geometry_msgs::Vector3& cmd_msg){
    
    // publish command saying that we are adjusting
    std_msgs::String str;
    //	str.data = "Adjusting Motor Vals";
    //	pub_state.publish(&str);
    
    setPointA = cmd_msg.x;
    setPointB = cmd_msg.y;
    setPointC = cmd_msg.z;
    
    // publish command alerting the master system that we are ready
    //  std_msgs::String str;
    str.data = "Motor Vals Set";
    pub_state.publish(&str);
}

ros::Subscriber<geometry_msgs::Vector3> motor_sub("/launcher/motor_vel", motor_controls_cb);

void pid_controls_cb(const geometry_msgs::Vector3& pid_msg){
    
    // publish command saying that we are adjusting
    std_msgs::String str;
    //	str.data = "Reading PID";
    //	pub_state.publish(&str);
    
    // Get motor speed A, B, and C from the message
    /*
    Kp = pid_msg.x;
    Ki = pid_msg.y;
    Kd = pid_msg.z;
    */
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
    motorA.write(15);
    motorB.write(0);
    motorC.write(0);
    //Long delay so enable
    nh.spinOnce();
    //delay 3 second,  some speed controllers may need longer
    delay(5000);
}

//double pid_calc(double current_speed,double set_point,double last_speed,unsigned int dt){
//    if(current_speed > 0){
//        p = (set_point - current_speed) * Kp;
//        d = ((current_speed - last_speed)/(double)dt) * Kd;
//        double out = p + d;
//        return out;
//    }
//    else{ 
//        p = (set_point - current_speed) * Kp_low;
//        d = ((current_speed - last_speed)/(double)dt) * Kd_low;
//        double out = p + d;
//        return out;
//    }
//}

void setup(){
    noInterrupts();
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
    
    // Arm servos (set speeds to 0);
    arm_servos();
    interrupts();
}

void loop(){
     // ros necesisity
     nh.spinOnce();
    /*
    This will recalculate a new speed value every 9 readings (9/14th rotation)
    */
    calc_time = millis();
    if (calc_time >= calc_dt + (double)timeoldA/1000.00 || hall_a_ctr >=4){        
        time = micros();
        /// hall counter in the numerator is there to integrate number of ticks
        // 1,000,000 in numerator is to convert microseconds to seconds
        // 14 is there because there are 14 hall readings per rotation
        dt = time - timeoldA;
        curSpeedA = ((double)hall_a_ctr * 1000000.0)/(14.0 * ((double)dt));
        if(curSpeedA > 0){
            p = (setPointA - curSpeedA) * kp_A;
            d = ((curSpeedA - last_speed_A)/(double)dt) * kd_A;
            Outputa += p + d;
        }
        else{ 
            p = (setPointA - curSpeedA) * kp_low_A;
            d = ((curSpeedA - last_speed_A)/(double)dt) * kd_low_A;
            Outputa += p + d;
        }
        Outputa = constrain(Outputa, 15, 173);
        hall_a_ctr = 0;
        last_speed_A = curSpeedA;
        timeoldA = time;        
    }
    
    //Update B speed and PID
    if (calc_time >= calc_dt + (double)timeoldB/1000.00 || hall_b_ctr >=3){
        time = micros();
        dt = time - timeoldB;
        curSpeedB = ((double)hall_b_ctr * 1000000.0)/(14.0 * ((double)dt));
        if(curSpeedB > 0){
            p = (setPointB - curSpeedB) * kp_B;
            d = ((curSpeedB - last_speed_B)/(double)dt) * kd_B;
            Outputb += p + d;
        }
        else{ 
            p = (setPointB - curSpeedB) * kp_low_B;
            d = ((curSpeedB - last_speed_B)/(double)dt) * kd_low_B;
            Outputb += p + d;
        }
        Outputb = constrain(Outputb, 0, 179);
        timeoldB = time;
        hall_b_ctr = 0;
        last_speed_B = curSpeedB;
    }
    
    if (calc_time >= calc_dt + (double)timeoldC/1000.00 || hall_c_ctr >=3){
        time = micros();
        dt = time - timeoldC;
        curSpeedC = ((double)hall_c_ctr * 1000000.0)/(14.0 * ((double)dt));
        if(curSpeedC > 0){
            p = (setPointC - curSpeedC) * kp_C;
            d = ((curSpeedC - last_speed_C)/(double)dt) * kd_C;
            Outputc += p + d;
        }
        else{ 
            p = (setPointC - curSpeedC) * kp_low_C;
            d = ((curSpeedC - last_speed_C)/(double)dt) * kd_low_C;
            Outputc += p + d;
        }
        Outputc = constrain(Outputc, 0, 180);
        timeoldC = time;
        hall_c_ctr = 0;
        last_speed_C = curSpeedC;
    }
    
    if (calc_time >= last_report_time + report_dt){
        speed_msg.x = curSpeedA;
        speed_msg.y = curSpeedB;
        speed_msg.z = curSpeedC;
        pub_speed.publish(&speed_msg);
        last_report_time = calc_time;
    }
    /*
    motorA.write(setPointA);
    motorB.write(setPointB);
    motorC.write(setPointC);
    */
    motorA.write(Outputa);
    motorB.write(Outputb);
    motorC.write(Outputc);
    
}

void motor_a_hall_change(){
    hall_a_ctr ++;
}

void motor_b_hall_change() {
    hall_b_ctr ++;
}

void motor_c_hall_change() {
    hall_c_ctr++;
}

