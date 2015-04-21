int MotorA = 9;
int MotorB = 10;
int val = 0;
char motorSelectA = 0;
char motorSelectB = 0;

void setup(){
pinMode(MotorA,OUTPUT);
pinMode(MotorB,OUTPUT);
Serial.begin(9600);
}

void loop(){
  if( Serial.available() >= 4 ) {
    char prefix = Serial.read();
    motorSelectA = (prefix == 'a');
    motorSelectB = (prefix == 'b');
    
    char dec;
    val = 0;
    for (int i=100; i>=1; i/=10) {
      dec = Serial.read() - '0';
      val += dec * i;
    }
    
    Serial.write("Got ");Serial.println(val,DEC);
    Serial.flush(); // get rid of excess
  }
  
  if (motorSelectA) analogWrite(MotorA,val);
  if (motorSelectB) analogWrite(MotorB,val);
  motorSelectA = motorSelectB = 0;
}
