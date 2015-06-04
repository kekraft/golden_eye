#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

int state_switch = 2;
int dryer_relay = 8;

int time_old = 0;

ros::NodeHandle nh;

std_msgs::String state_msg;
std_msgs::Bool side_msg;
ros::Publisher state_pub("/game_side/state", &state_msg);
ros::Publisher side_pub("/game_side/offense", &side_msg);

void dryer_cmd_cb(const std_msgs::Bool& cmd){
   bool dryer_on = cmd.data;
  
   if (dryer_on) {
     // turn dryer on
     digitalWrite(dryer_relay,HIGH);     
   } else {
     // turn dryer off 
     digitalWrite(dryer_relay,LOW);     
   }
    
}

ros::Subscriber<std_msgs::Bool> dryer_sub("/dryer/cmd", dryer_cmd_cb);

//void game_state_change(){
//  if (HIGH == digitalRead(state_switch)) {
//     // OFFENSE
//     state_msg.data = "OFFENSE";
//     side_msg.data = true;
//          
//  } else {
//     state_msg.data = "DEFENSE";
//     side_msg.data = false;
//  }
//  
//  state_pub.publish(&state_msg);
//  side_pub.publish(&side_msg);  
//}




void setup() {
  //ROS stuff
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(state_pub);
  nh.subscribe(dryer_sub);
  
  pinMode(state_switch, INPUT);
  pinMode(dryer_relay, OUTPUT);
  
  // start dryer low
  digitalWrite(dryer_relay,HIGH);
  
  // attach an interrupt to get game state
//  attachInterrupt(0, game_state_change, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
//  if (millis() - time_old > 500){
//    if (HIGH == digitalRead(state_switch)) {
//       // OFFENSE
//       state_msg.data = "OFFENSE";
//       side_msg.data = true;
//            
//    } else {
//       state_msg.data = "DEFENSE";
//       side_msg.data = false;
//    }
//    
//    state_pub.publish(&state_msg);
//    side_pub.publish(&side_msg); 
//  
//    time_old = millis();
//  }
  
  nh.spinOnce();
}
