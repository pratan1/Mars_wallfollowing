#define DIST_PROPORTIONAL_CONST 0.034/2

//sensor 1 pins
const int trigPin1 = 3;
const int echoPin1 = 2;

//sensor 2 pins
const int trigPin2 = 4;
const int echoPin2 = 5;

const int trigPin3 = 6;
const int echoPin3 = 7;

float distance1, distance2,distance3;

float sensor_output(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);

  float distance = duration*DIST_PROPORTIONAL_CONST;
   
  return distance; 
  }

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigPin1,OUTPUT);
  pinMode(echoPin1,INPUT);
  pinMode(trigPin2,OUTPUT);
  pinMode(echoPin2,INPUT);
  pinMode(trigPin3,OUTPUT);
  pinMode(echoPin3,INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  distance1 = sensor_output(trigPin1, echoPin1);
  distance2 = sensor_output(trigPin2, echoPin2);
  distance3 = sensor_output(trigPin3, echoPin3);
  Serial.print("1:");
  Serial.println(distance1);
  Serial.print("2:");
  Serial.println(distance2);
  Serial.print("3:");
  Serial.println(distance3);
  delay(1000);

}
