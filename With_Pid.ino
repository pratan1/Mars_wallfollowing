#define SPEED 60
#define INPUT_DISTANCE 15 //in cm
#define ERROR_DIST 5  // in cm
#define DIST_PROPORTIONAL_CONST 0.034/2 
#define MAX_DIFFERNCE 40
#define MAX_TURN_SPEED 10
#define SLWTURN_SPEED 20
#define CALIBRATION 3
#define MAX_ALLIGN_ANGLE 2
#define COLLISION_DISTANCE 40

float distance1, distance2, distance3, distance;
int region;
float allign_angle;

//sensor 1 pins
const int trigPin1 = 3;
const int echoPin1 = 2;

//sensor 2 pins
const int trigPin2 = 4;
const int echoPin2 = 5;

//sensor 3 pins : front sensor
const int trigPin3 = 6;
const int echoPin3 = 7;

//right motor pins
const int in1R = 13;
const int in2R = 12;
const int enR = 11;

//left motor pins
const int in1L = 9;
const int in2L = 8;
const int enL = 10;

float sensor_output(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  Serial.print("duration:");
  Serial.println(duration);

  float distance = duration * DIST_PROPORTIONAL_CONST;
  return distance; 
}

float current_distance(float read1, float read2) {
  float distance = (read1 + read2) / 2;  //taking average of two values
  return distance;
}
/*

void err_right() {
  analogWrite(enL, SPEED + SLWTURN_SPEED);
  analogWrite(enR, SPEED - SLWTURN_SPEED);
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
}

void err_left() {
  analogWrite(enL, SPEED - SLWTURN_SPEED);
  analogWrite(enR, SPEED + SLWTURN_SPEED);
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
}*/

void follow_wall() {
  analogWrite(enL, SPEED+30);
  analogWrite(enR, SPEED);
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
}

void left_turn() {
  analogWrite(enL, 20);
  analogWrite(enR, 200);
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
}

void right_turn() {
  analogWrite(enL, 200);
  analogWrite(enR, 0);
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
}

void forward() {
  analogWrite(enL, SPEED+30);
  analogWrite(enR, SPEED);
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
}

void setup() {
  Serial.begin(9600);
  pinMode(in1L, OUTPUT);
  pinMode(in2L, OUTPUT);
  pinMode(in1R, OUTPUT);
  pinMode(in2R, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);                          
  
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  digitalWrite(enL, LOW);
  digitalWrite(enR, LOW);
}

void loop() {
  distance1 = sensor_output(trigPin1, echoPin1);
  distance2 = sensor_output(trigPin2, echoPin2);
  distance3 = sensor_output(trigPin3, echoPin3);
  
  float avgdist = current_distance(distance1, distance2);
//  region = check_region(distance1, distance2);
  allign_angle = (distance1 - distance2);
  
  if (distance3 < COLLISION_DISTANCE) {
    right_turn();
  } else if (distance1 > 40){
    
    left_turn();
  } else {
    forward();
  }
}
