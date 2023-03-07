

#include <MPU9250_WE.h>
#include <Wire.h>
#define MPU9250_ADDR 0x68

// Motor A connections
int enA = 9;
int in1 = 2;
int in2 = 3;
// Motor B connections
int enB = 10;
int in3 = 4;
int in4 = 15;

int targetAng = 0;
int actualAng;
int error=0;
int errorOld=0;
float errorChange;
float errorSlope = 0;
int errorArea = 0;

int debugSpeed=0;
int Bsp=0;


float kp = 1.2;                                                                                                       ;
float ki = 0;
float kd = 50;

int milliOld;
int milliNew;
int dt;

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

  	// Set motors to maximum speed
	// For PWM maximum possible values are 0 to 255
	// Turn on motor A & B
void forward(float speed){  
  analogWrite(enA, speed);
	analogWrite(enB, speed);
	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
//	digitalWrite(in3, HIGH);
//	digitalWrite(in4, LOW);
  return 0;

  }
	
	// Now change motor directions
void backward(float speed){
  analogWrite(enA, speed);
	analogWrite(enB, speed);
	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
//	digitalWrite(in3, LOW);
//	digitalWrite(in4, HIGH);
  return 0;

  }

  void stop(){
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
  }

void setup() {
  Serial.begin(115200);

  	// Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	


  Wire.begin();
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
 
  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");
  
  myMPU9250.setSampleRateDivider(5);

  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);

  myMPU9250.enableAccDLPF(true);

  myMPU9250.setAccDLPF(MPU9250_DLPF_6);  

  milliNew = millis();

}

void loop() {

int AbFspeed;
int AbBspeed;

  xyzFloat angles = myMPU9250.getAngles();
  
milliOld = milliNew;
milliNew = millis();
dt = milliNew - milliOld;

actualAng = angles.x;

errorOld = error;
error = targetAng - actualAng;
errorChange = error - errorOld;
errorSlope = errorChange/dt;
errorArea = errorArea + error*dt ;

int pTerm= kp*error;
int iTerm = ki*errorArea;
int dTerm = kd*errorSlope;


debugSpeed = pTerm + iTerm + dTerm;


if(debugSpeed>2){
  forward(abs(debugSpeed)+75);


}
else if(debugSpeed<-2){
  backward(abs(debugSpeed)+70);

}  

else{
  stop();
}

  Serial.print(error);
 // Serial.print(", ");
//  Serial.print(abs(AbFspeed+50));
 // Serial.print(",");
//  Serial.println(abs(AbBspeed+50));
/*
  Serial.print(errorSlope);
  Serial.print(", ");
  Serial.print(errorArea); */
  Serial.print(", ");
  Serial.print(kd*errorSlope);
  Serial.print(", ");
  Serial.println(debugSpeed);
  
  delay(100);
 
}