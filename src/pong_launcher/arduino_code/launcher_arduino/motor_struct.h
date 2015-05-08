// Structure for motor definitions

// Messing with structs for motor definitions
struct Motor_Struct {
  int top1;
  int top2;
  int top3;
  int bot1;
  int bot2;
  int bot3;
  
  int h1;
  int h2;
  int h3;
  
  volatile double curSpeed;
  
  volatile int timeold;
  
  volatile int hall1, hall2, hall3;
  
  double pid_setpoint, pid_input, pid_output;
};
