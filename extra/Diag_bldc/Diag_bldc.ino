int p_ha1 = 3;
int p_ha2 = 4;
int p_ha3 = 5;
int spd = 255;

// motor A phase signals
int ArH = 6;
int ArL = 7;
int AyH = 8;
int AyL = 9;
int AkH = 10;
int AkL = 11;

volatile byte h = 0;
volatile bool write_flag = false;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  //Pinmodes for the motor's PWM
  pinMode(ArH,OUTPUT);
  pinMode(ArL,OUTPUT);
  pinMode(AyH,OUTPUT);
  pinMode(AyL,OUTPUT);
  pinMode(AkH,OUTPUT);
  pinMode(AkL,OUTPUT);

  pinMode(p_ha1,INPUT);
  pinMode(p_ha2,INPUT);
  pinMode(p_ha3,INPUT);

}

void hall_change_isr() {
  digitalWrite(ArL, 0);
  digitalWrite(AyL, 0);
  digitalWrite(AkL, 0);
  //  analogWrite(ArL, 0);
//  analogWrite(AyL, 0);
//analogWrite(AkL, 0);
digitalWrite(ArH, LOW);
digitalWrite(AyH, LOW);
digitalWrite(AkH, LOW);
delayMicroseconds(3);

  write_flag = true;
  h = 0;
  if(digitalRead(p_ha1) == HIGH){
    h |= 0b001; // h = h | 0b001;
  }
  if(digitalRead(p_ha2) == HIGH){
    h |= 0b010;
  }
  if(digitalRead(p_ha3) == HIGH){
    h |= 0b100;
  }
  
  // break out hall bit pattern
  if( h == 0b000 ) h=0b110;
  
                if (h==0b101){
                        digitalWrite(ArL, LOW);
			digitalWrite(AyL, HIGH);
			digitalWrite(AkL, LOW);
//			analogWrite(ArL, 0);
//			analogWrite(AyL, spd);
//			analogWrite(AkL, 0);
			digitalWrite(ArH, HIGH);
			digitalWrite(AyH, LOW);
			digitalWrite(AkH, LOW);
		}
		//Case 2
		else if (h==0b100){
                        digitalWrite(ArL, LOW);
			digitalWrite(AyL, LOW);
			digitalWrite(AkL, HIGH);
//			analogWrite(ArL, 0);
//			analogWrite(AyL, 0);
//			analogWrite(AkL, spd);
			digitalWrite(ArH, HIGH);
			digitalWrite(AyH, LOW);
			digitalWrite(AkH, LOW);
		}
		//Case 3
		else if (h==0b110){
                        digitalWrite(ArL, LOW);
			digitalWrite(AyL, LOW);
			digitalWrite(AkL, HIGH);
//			analogWrite(ArL, 0);
//			analogWrite(AyL, 0);
//			analogWrite(AkL, spd);
			digitalWrite(ArH, LOW);
			digitalWrite(AyH, HIGH);
			digitalWrite(AkH, LOW);
		}
		//Case 4
		else if (h==0b010){
			digitalWrite(ArL, HIGH);
			digitalWrite(AyL, LOW);
			digitalWrite(AkL, LOW);
//			analogWrite(ArL, spd);
//			analogWrite(AyL, 0);
//			analogWrite(AkL, 0);
			digitalWrite(ArH, LOW);
			digitalWrite(AyH, HIGH);
			digitalWrite(AkH, LOW);
		}
		//Case 5
		else if (h==0b011){
                        digitalWrite(ArL, HIGH);
			digitalWrite(AyL, LOW);
			digitalWrite(AkL, LOW);
//			analogWrite(ArL, spd);
//			analogWrite(AyL, 0);
//			analogWrite(AkL, 0);
			digitalWrite(ArH, LOW);
			digitalWrite(AyH, LOW);
			digitalWrite(AkH, HIGH);
		}
		//Case 6
		else if (h==0b001){
                        digitalWrite(ArL, LOW);
			digitalWrite(AyL, HIGH);
			digitalWrite(AkL, LOW);
//			analogWrite(ArL, 0);
//			analogWrite(AyL, spd);
//			analogWrite(AkL, 0);
			digitalWrite(ArH, LOW);
			digitalWrite(AyH, LOW);
			digitalWrite(AkH, HIGH);
		}
}



void loop() {
  hall_change_isr();
  // put your main code here, to run repeatedly:
  if (write_flag == true) {
    write_flag = false;
    Serial.println(0b10000+h,BIN);
  }
}

