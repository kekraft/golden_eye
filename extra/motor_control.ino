#include <PID_v1.h>
#define int2 2
#define int3 3
#define int4 4

int MotorA = 6;
int MotorB = 9;
int MotorC = 10;
int val = 0;
//Hall effect sensor variables
int halla_old = 0, hallb_old = 0, hallc_old = 0;
double rota = 0, rotb = 0, rotc =0;
int timeold = 0;
volatile int halla = 0, hallb = 0, hallc = 0;
volatile int ina, inb, inc;
//PID variables
char motorSelect = 0;
double Setpointa, Inputa, Outputa;
double Setpointb, Inputb, Outputb;
double Setpointc, Inputc, Outputc;
double Kp = 1, Kd = 0.25, Ki = .05;
PID aPID(&Inputa, &Outputa, &Setpointa, Kp, Kd, Ki, DIRECT);
PID bPID(&Inputb, &Outputb, &Setpointb, Kp, Kd, Ki, DIRECT);
PID cPID(&Inputc, &Outputc, &Setpointc, Kp, Kd, Ki, DIRECT);

void setup(){
	//Pinmodes for the motor's PWM
	pinMode(MotorA,OUTPUT);
	pinMode(MotorB,OUTPUT);
	pinMode(MotorC,OUTPUT);
	//Defining the interupt pins as interrupt, required by the Teensy 3.1
	pinMode(int2, INPUT);
	pinMode(int3, INPUT);
	pinMode(int4, INPUT);

	Serial.begin(9600);

	Setpointa = 0;
	Setpointb = 0;
	Setpointc = 0;

	aPID.SetMode(AUTOMATIC);
	bPID.SetMode(AUTOMATIC);
	cPID.SetMode(AUTOMATIC);

	attachInterrupt(int2, hall_isra, RISING);
	attachInterrupt(int3, hall_isrb, RISING);
	attachInterrupt(int4, hall_isrc, RISING);

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

	if( millis() - timeold > 20){
		rota = (halla - halla_old)/(millis()-timeold);
		rotb = (hallb - hallb_old)/(millis()-timeold);
		rotc = (hallc - hallc_old)/(millis()-timeold);
		halla_old = halla;
		hallb_old = hallb;
		hallc_old = hallc;
	}

  	Inputa = rota;
	Inputb = rotb;
	Inputc = rotc;
}

void hall_isra(){
	if (ina > (millis()+1)){
	halla++;
}
}

void hall_isrb(){
	if (inb > (millis()+1)){
	hallb++;
}
}

void hall_isrc(){
	if (inc > (millis()+1)){
	hallc++;
}
}