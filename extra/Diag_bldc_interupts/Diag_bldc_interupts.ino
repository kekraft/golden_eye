int ha1 = A8;
int ha2 = A7;
int ha3 = A6;

int hb1 = A5;
int hb2 = A4;
int hb3 = A3;

int hc1 = A2;
int hc2 = A1;
int hc3 = A0;

volatile byte h = 0;
volatile bool write_flag = false;
volatile int hall_a_ctr = 0;

void setup() {
  Serial.begin(9600);
  Serial.print("Hi");
  
  // halls for motor 1
  pinMode(ha1,INPUT);
  pinMode(ha2,INPUT);
  pinMode(ha3,INPUT);
  
  attachInterrupt(ha1, hall_change_isr, CHANGE);
  attachInterrupt(ha2, hall_change_isr, CHANGE);
  attachInterrupt(ha3, hall_change_isr, CHANGE);
  
  // halls for motor 2
  pinMode(hb1,INPUT);
  pinMode(hb2,INPUT);
  pinMode(hb3,INPUT);
  
  attachInterrupt(hb1, hall_change_isr, CHANGE);
  attachInterrupt(hb2, hall_change_isr, CHANGE);
  attachInterrupt(hb3, hall_change_isr, CHANGE);
  
  // halls for motor 3
  pinMode(hc1,INPUT);
  pinMode(hc2,INPUT);
  pinMode(hc3,INPUT);
  
  attachInterrupt(hc1, hall_change_isr, CHANGE);
  attachInterrupt(hc2, hall_change_isr, CHANGE);
  attachInterrupt(hc3, hall_change_isr, CHANGE);
}

void hall_change_isr() {
//    Serial.println("Motor A interupt");
    
  hall_a_ctr ++;

//  write_flag = true;
  h = 0;
  if(digitalRead(ha1) == HIGH){
    h |= 0b001; // h = h | 0b001;
  }
  if(digitalRead(ha2) == HIGH){
    h |= 0b010;
  }
  if(digitalRead(ha3) == HIGH){
    h |= 0b100;
  }
  
  // break out hall bit pattern
  if( h == 0b000 ) h=0b110;
  
//  Serial.println(h);
  Serial.println("Num halls: ");
  Serial.print(hall_a_ctr, DEC);
  Serial.println();
}



void loop() {
  

  // put your main code here, to run repeatedly:
//  if (write_flag == true) {
//    write_flag = false;
//    Serial.println(0b10000+h,BIN);
//  }
}

