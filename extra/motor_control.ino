#include <PID_v1.h>

int MotorA = 6;
int MotorB = 9;
int MotorC = 10;
int val = 0;
char motorSelect = 0;

double Setpointa, Inputa, Outputa;
double Setpointb, Inputb, Outputb;
double Setpointc, Inputc, Outputc;

volatile double veca, vecb, vecc;

double Kp = 1, Kd = 0.25, Ki = .05;
PID aPID(&Inputa, &Outputa, &Setpointa, Kp, Ki, Kd, DIRECT);
PID bPID(&Inputb, &Outputb, &Setpointb, Kp, Ki, Kd, DIRECT);
PID cPID(&Inputc, &Outputc, &Setpointc, Kp, Ki, Kd, DIRECT);

void setup(){
	pinMode(MotorA,OUTPUT);
	pinMode(MotorB,OUTPUT);
	pinMode(MotorC,OUTPUT);

	Serial.begin(9600);

	Setpointa = 0;
	Setpointb = 0;
	Setpointc = 0;

	aPID.SetMode(AUTOMATIC);
	bPID.SetMode(AUTOMATIC);
	cPID.SetMode(AUTOMATIC);

	attachInterrupt(, function, mode);
}

void loop(){
	if( Serial.available() >= 4 ) {
		char motorSelect = Serial.read();
    
    	char dec;
    	val = 0;
    	for (int i=100; i>=1; i/=10) {
      	dec = Serial.read() - '0';
      	val += dec * i;
    	}
    	//Serial.write("Got ");Serial.println(val,DEC);
    	Serial.flush(); // get rid of excess
  	}

  	Inputa = veca;
	Inputb = vecb;
	Inputc = vecc;

  	switch (motorSelect) {
  		case 'a':
  			Setpointa = val;
  			break;
	  	case 'b':
	  		Setpointb = val;
	  		break;
	  	case 'c':
	  		Setpointc = val;
	  		break;
	  	default:
	  		motorSelect = 0;
	}
	aPID.Compute();
	bPID.Compute();
	cPID.Compute();
}

void hall_isr(){

}