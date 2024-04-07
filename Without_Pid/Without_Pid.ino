#define SENSOR_MAX 
#define DIST_PROPORTIONAL_CONST 0.034/2 //DEPENDS ON ENVIRONMENT AND SPEED OF SOUND
#define INPUT_DISTANCE 15 //in cm
#define ERROR_DIST 5  // in cm
#define SPEED 80
#define MAX_SENSOR_VALUE 10000
#define MAX_OF_SENSOR 3000
#define MAX_DIFFERNCE 40
#define MAX_TURN_SPEED 20
#define SLWTURN_SPEED 10
#define CALIBRATION 3
#define MAX_ALLIGN_ANGLE 2
#define COLLISION_DISTANCE 15 

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
const int in1L = 8;
const int in2L = 9;
const int enL = 10;

//pid
float kp_a = 3;
float kd_a = 2;
//tuning parameters
float kp_d = 1;
float kd_d = 0;
float ki_d = 0;
//distance
float errorD;
float previousErrorD = 0;
float dt = 0.1;

//
float distance1, distance2,distance3, currentDistance, integral, derivative, outputD, outputA, angle, previousAngle=0.0, allign_angle=0.0;
int region, pidDist, speedL, speedR, distance;

//reading sensor
float sensor_output(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  Serial.print("duration:");
  Serial.println(duration);
  /*if(duration==0.0||duration>=MAX_SENSOR_VALUE){
    duration = MAX_OF_SENSOR;
    }
    */

  float distance = duration*DIST_PROPORTIONAL_CONST;
   
  return distance; 
}

//////////////////////calculates distance from wall
float current_distance(float read1, float read2){
  float distance = (read1 + read2)/2;  //taking average of two values

  return distance;
  }

int check_region(float read1, float read2){
  distance = current_distance(read1, read2);
  if(abs(distance-INPUT_DISTANCE)>ERROR_DIST){
    if(distance > INPUT_DISTANCE){
        return -1; //for left movement
      }else{
        return 1; //for right movement
        }
    }else{
      return 0; //inside region
      }
  }


void reach_distance(){
  analogWrite(enL, SPEED);
  analogWrite(enR, SPEED);
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
  
  }
void follow_wall(){
  distance1 = sensor_output(trigPin1, echoPin1);
  distance2 = sensor_output(trigPin2, echoPin2);
  float angle = distance1-distance2;//if +ve turn right else turn left
  float derivativeA = angle - previousAngle;
  outputA = kp_a*angle + kd_a*derivativeA;
  previousAngle = angle;
  speedL = SPEED + CALIBRATION +outputA/4;
  speedR = SPEED - outputA/4;
  if((speedR-speedL)>SLWTURN_SPEED){
    speedL = SPEED  + (SLWTURN_SPEED)/2;
    speedR = SPEED - (SLWTURN_SPEED)/2;
    }else if((speedL-speedR)>SLWTURN_SPEED){
      speedL = SPEED - (SLWTURN_SPEED)/2;
      speedR = SPEED + (SLWTURN_SPEED)/2;
      }
  analogWrite(enL, speedL);
  analogWrite(enR, speedR);
  digitalWrite(in1L, HIGH);
  digitalWrite(in2L, LOW);
  digitalWrite(in1R, HIGH);
  digitalWrite(in2R, LOW);
  }

  bool check_collision(){
    distance3 = sensor_output(trigPin3, echoPin3);
    if(distance3<COLLISION_DISTANCE){
      return true;
    }else{
      return false;
    }
    }
void setup() {
  Serial.begin(9600);
  pinMode(in1L,OUTPUT);
  pinMode(in2L,OUTPUT);
  pinMode(in1R,OUTPUT);
  pinMode(in2R,OUTPUT);
  pinMode(enL,OUTPUT);
  pinMode(enR,OUTPUT);                          
  
  pinMode(trigPin1,OUTPUT);
  pinMode(echoPin1,INPUT);
  pinMode(trigPin2,OUTPUT);
  pinMode(echoPin2,INPUT);
  pinMode(trigPin3,OUTPUT);
  pinMode(echoPin3,INPUT);

  digitalWrite(enL, LOW);
  digitalWrite(enR, LOW);
}

void loop() {
  distance1 = sensor_output(trigPin1, echoPin1);
  distance2 = sensor_output(trigPin2, echoPin2);
  distance3 = sensor_output(trigPin3, echoPin3);
  
  float avgdist=current_distance(distance1, distance2);
  region = check_region(distance1, distance2);
  //allign_angle = abs(distance2 - distance1);
  allign_angle = (distance1 - distance2);
  Serial.print("distance1:");
  Serial.println(distance1);
  Serial.print("distance2:");
  Serial.println(distance2);
  Serial.print("region:");
  Serial.println(region);
  if(check_collision()){
    //turn right
    speedL = SPEED + MAX_DIFFERNCE;
    speedR = SPEED - MAX_DIFFERNCE;
    analogWrite(enL, 0);
    analogWrite(enR, 0);
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
  }else{
    if(region==0){
    follow_wall();
    }
    else if (region ==1){
      if (avgdist>40){
      reach_distance();
        }
     else{
      if (allign_angle>40){
    analogWrite(enL, SPEED+MAX_DIFFERNCE );
    analogWrite(enR, SPEED-MAX_DIFFERNCE);
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);}
      else{
    analogWrite(enL, SPEED+SLWTURN_SPEED );
    analogWrite(enR, SPEED-SLWTURN_SPEED );
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
      }
     }
    }
    else if (region ==-1){
     if (avgdist>40){
      reach_distance();
        }
     else{
      if (allign_angle>40){
    analogWrite(enL, SPEED-MAX_DIFFERNCE );
    analogWrite(enR, SPEED+MAX_DIFFERNCE);
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
    }
        
      else{
    analogWrite(enL, SPEED-SLWTURN_SPEED );
    analogWrite(enR, SPEED+SLWTURN_SPEED );
    digitalWrite(in1L, HIGH);
    digitalWrite(in2L, LOW);
    digitalWrite(in1R, HIGH);
    digitalWrite(in2R, LOW);
      }
      }
    

    
      }   
        }
      
    }

  
  
