#include <Servo.h>
#include <NewPing.h>

#define SERVO_PIN 9

#define M1_IN1 A1
#define M1_IN2 A2
#define M2_IN1 A3
#define M2_IN2 A4

#define COLL_DIST 30 // sets distance at which robot stops and reverses to 10cm
#define TURN_DIST COLL_DIST-10 // sets distance at which robot steers away from object (not reverse) to 20cm (10+10)

#define TRIGGER_PIN  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     6  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 300 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.


int servo_angle[3]={30,90,150};
int max_angles=3;
int servo_delay=15; //delay to reach the position
int servo_pos=0;

int Step_delay=200;
int CurDist = 0;
int maxAngle = 0;

int maxRight = 0;
int maxLeft = 0;
int maxFront = 0;
 int d1,d0,d2;

int cur_dir=0;  //Direction code : 1:forward,-1 backward,2:right,-2:left

Servo myservo;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
 myservo.attach(SERVO_PIN); 
 myservo.write(90);
delay(500);
 Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

/*
checkdirection(1);
foreward();
checkdirection(0);
 */

if(chkforward()!=1){
  if(chkleft()!=1){
    if(chkright()!=1)stepback();
    }
  }
if(maxAngle<90){right();}
 else   if(maxAngle>90){left();} 
}


//Find the next path direction

void checkdirection(int dir){
 int curLeft = 0;
 int curFront = 0;
 int curRight = 0;
 int curDist = 0;
 int j,inc;
 if(dir==1){j=0;inc=1;}
 else {j=max_angles-1;inc=-1;}
 for(int i=0;i<max_angles;i++){  
  if(cur_dir==1){foreward();}  
  curDist=scan(i);
  // if dist is less than colliasion dist, then go back 1 step and change direction.
  if(curDist<COLL_DIST){
    stepback();
    if(maxAngle<90){right();}
    if(maxAngle>90){left();}
    break;
    }
   //check distance is lessthan trun threshold , then trun
   if (curDist < TURN_DIST) {
    if(servo_angle[j]<90){right();}
    if(servo_angle[j]>90){left();}
      foreward();
      }
 //find direction with max avilable distancce      
 if(curDist>CurDist){maxAngle=servo_angle[j];}  
  CurDist=curDist;
 //delay(500);
 j=j+inc;
  }  
  Serial.print("dir:");Serial.println(cur_dir);
}

int chkforward(){
  
   d1=scan(1); 
   
  if(d1<COLL_DIST){
   // stepback();
    mstop();   
    return -1;
  } 
  else  foreward(); 
  maxAngle=servo_angle[1];       
  return 1;
}

int chkleft(){
   CurDist=scan(2);
    if (CurDist < TURN_DIST) {
        // if(cur_dir!=1)stepback();
          return -1;
      }
      else left();
       maxAngle=servo_angle[2];
       return 1;
  }
  
int chkright(){
   CurDist=scan(0);
    if (CurDist < TURN_DIST) {
        // if(cur_dir!=1)stepback();
          return -1;
      }
      else right();
      maxAngle=servo_angle[0];
       return 1;
  }
  
//back on obstacle
void stepback(){
   backward();
   delay(Step_delay);
  // mstop();  
  }


// *************Movement
//1. Foreward
void foreward(){
  digitalWrite(M1_IN1,LOW);
  digitalWrite(M1_IN2,HIGH);
  digitalWrite(M2_IN1,LOW); 
  digitalWrite(M2_IN2,HIGH);  
  cur_dir=1;
  delay(Step_delay);
  }

//2. Backward
void backward(){
  digitalWrite(M1_IN1,HIGH);
  digitalWrite(M1_IN2,LOW);
  digitalWrite(M2_IN1,HIGH);  
  digitalWrite(M2_IN2,LOW);  
  cur_dir=-1;
  delay(Step_delay);
  }  

 //3. Left
void left(){
  digitalWrite(M1_IN1,LOW);
  digitalWrite(M1_IN2,HIGH);
  digitalWrite(M2_IN1,HIGH);  
  digitalWrite(M2_IN2,LOW);  
  cur_dir=-2;
  delay(Step_delay);
  }   

 //4. Right
void right(){
  digitalWrite(M1_IN1,HIGH);
  digitalWrite(M1_IN2,LOW);
  digitalWrite(M2_IN1,LOW); 
  digitalWrite(M2_IN2,HIGH); 
  delay(Step_delay);
  cur_dir=2;
  } 
  
//5.Stop
void mstop(){
  digitalWrite(M1_IN1,LOW);
  digitalWrite(M1_IN2,LOW);
  digitalWrite(M2_IN1,LOW); 
  digitalWrite(M2_IN2,LOW); 
 // delay(Step_delay);
  cur_dir=0;
  }
  
//********************  Scaning - Obstacle Distances  
//Get Distance at each position  for a (angle- servo)
long scan(int pos){
  long dist;
  servopos(servo_angle[pos]);  
 dist=sonar.ping_cm();
 if(dist == 0) // If we timed out
  {
   pinMode(ECHO_PIN, OUTPUT); // Then we set echo pin to output mode
   digitalWrite(ECHO_PIN, LOW); // We send a LOW pulse to the echo pin
   delayMicroseconds(200);
   pinMode(ECHO_PIN, INPUT); // And finaly we come back to input mode
   dist=sonar.ping_cm();
 }
  Serial.print(servo_angle[pos]);Serial.print("<-a,d->");Serial.println(dist);
  delay(50);
  if(dist<2)dist=MAX_DISTANCE;
  return dist;
}

//**********Position the Servo to and angle 'pos'
void servopos(int pos){
   myservo.write(pos);              //tell servo to go to position in variable 'pos'
    delay(servo_delay);  
}


