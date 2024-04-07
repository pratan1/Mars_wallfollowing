
#define SPEED 95


//right motor pins
const int in1R = 13;
const int in2R = 12;
const int enR = 11;

//left motor pins
const int in1L = 9;
const int in2L = 8;
const int enL = 10;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(in1L,OUTPUT);
  pinMode(in2L,OUTPUT);
  pinMode(in1R,OUTPUT);
  pinMode(in2R,OUTPUT);
  pinMode(enL,OUTPUT);
  pinMode(enR,OUTPUT);

  digitalWrite(enL, LOW);
  digitalWrite(enR, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:
    analogWrite(enL,SPEED);
    analogWrite(enR, SPEED);
    
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
    delay(3000);
    /*digitalWrite(in1L, LOW);
    digitalWrite(in2L, HIGH);
    digitalWrite(in1R, LOW);
    digitalWrite(in2R, HIGH);
    delay(3000);
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
    digitalWrite(in1R, LOW);
    digitalWrite(in2R, HIGH);
    delay(3000);
    digitalWrite(in1L, LOW);
    digitalWrite(in2L, HIGH);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
    delay(3000);  */  
    
   

}
