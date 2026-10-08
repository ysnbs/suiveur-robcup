#include <Arduino.h>

using namespace std;
void applyPIDToMotors(int pidOutput);
void setMotorSpeed(int leftSpeed, int rightSpeed);
void calculatePID();
void readSensors();
void stopMotors();
void calculateError(double type_error);
void readSensorsinv();


const int ML_F = 23;  // PWM pin for Motor A direction 1
const int ML_B = 4;  // PWM pin for Motor A direction 2  
const int MR_F = 21;  // PWM pin for Motor B direction 1
const int MR_B = 22; // PWM pin for Motor B direction 2
const float integral_limit=200;
const int led=2;
const int BUTTON_PIN=13;
bool started = false;
bool excuted =false;
bool inversed=false;



const int sensorPins[8] = {33,32,27,26,35,34,25,14}; 
float Kp = 25; 
float Ki = 0; 
float Kd = 12;   
int error = 0;                  
int previousError = 0;  
float integralTerm=0; 
int pidOutput =0;
// Motor Speed Parameters
int baseSpeed =150;     // Base speed when robot is on line (0-255)
int maxSpeed = 255;      // Maximum allowed speed
int minSpeed = 0;       // Minimum speed to overcome friction
int threshold;
// Sensor Values and Calibration
int sensorValues[8];     // Raw sensor readings
int sensorCalibrated[8]; // Calibrated sensor values
int valblanc[8];        // Minimum values during calibration
int valnoir[8];        // Maximum values during calibration
int sensorMin[8]={4095,4095,4095,4095,4095,4095,4095,4095};
int sensorMax[8]={0,0,0,0,0,0,0,0};



unsigned int t0;
const double NORMALERROR = 10;

//Line Position (0 = centered, negative = left, positive = right)
int linePosition = 0;
float weights[8] = {0 , 5, 3, 1, -1, -3, -5, 0};
void calibrateSensors() {
  int valBlanc[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  int valNoir[8]  = {0, 0, 0, 0, 0, 0, 0, 0};
  digitalWrite(led,HIGH);
  for (int j = 0; j < 50; j++) {
    for (int i = 0; i < 8; i++) {
      valBlanc[i] += abs(analogRead(sensorPins[i]));
    }
    delay(50);
  }
  digitalWrite(led,LOW);

  delay(3000);
  digitalWrite(led,HIGH);
  for (int j = 0; j < 50; j++) {
    for (int i = 0; i < 8; i++) {
      valNoir[i] +=abs(analogRead(sensorPins[i]));
    }
    delay(50);
  }
  digitalWrite(led,LOW);

  for (int i = 0; i < 8; i++) {
    sensorMin[i] = abs(valBlanc[i]) / 50;  
    sensorMax[i] = abs(valNoir[i]) / 50;  
    if (sensorMax[i] < sensorMin[i]) {  
      int temp = sensorMax[i];
      sensorMax[i] = sensorMin[i];
      sensorMin[i] = temp;
    }
  }
}
void readSensors() {
  for (int i = 0; i < 8; i++) {
    int rawValue = analogRead(sensorPins[i]);
    int calibratedValue = map(rawValue, sensorMin[i], sensorMax[i], 0, 4095);
    threshold = (sensorMin[i] + sensorMax[i]) / 2;
    sensorCalibrated[i] = (calibratedValue > threshold) ? 1 : 0; 
  }
}

void readSensorsinv(){
    for (int i = 0; i < 8; i++) {
      int rawValue = analogRead(sensorPins[i]);
      int calibratedValue = map(rawValue, sensorMin[i], sensorMax[i], 0, 4095);
      threshold = (sensorMin[i] + sensorMax[i]) / 2;
      sensorCalibrated[i] = (calibratedValue > threshold) ? 0 : 1; 
    }
  }


void calculateError(double type_error) {
  int weightedSum = 0;
  int sensorSum = 0;
  for (int i = 0; i < 8; i++) {
    weightedSum += sensorCalibrated[i] * weights[i];
    sensorSum += sensorCalibrated[i];
  }
  if (sensorSum != 0) {
    error = weightedSum;
    linePosition = weightedSum;
  } else {
    error = (previousError > 0) ? type_error : -type_error; 
  }
}   


void calculatePID() {
  float proportional = Kp * error;
  integralTerm += Ki * error;
  integralTerm = constrain(integralTerm,-integral_limit,integral_limit);

  float derivative = error - previousError;
  previousError = error;
  
  pidOutput = (int)(proportional + integralTerm + Kd*derivative);
  
  applyPIDToMotors(pidOutput);
}


void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Left Motor Control
  if (leftSpeed > 0) {
    // Forward motion
    analogWrite(ML_F, leftSpeed);
    analogWrite(ML_B, 0);
  } else {
    // Backward motion (if needed for sharp turns)
    analogWrite(ML_F, 0);
    analogWrite(ML_B, abs(leftSpeed));
  }
  
  // Right Motor Control
  if (rightSpeed > 0) {
    // Forward motion
    analogWrite(MR_F, rightSpeed);
    analogWrite(MR_B, 0);
  } else {
    // Backward motion (if needed for sharp turns)
    analogWrite(MR_F, 0);
    analogWrite(MR_B, abs(rightSpeed));
  }
}

void applyPIDToMotors(int pidOutput) {   // ----------------------updated------------

  int leftMotorSpeed = baseSpeed + pidOutput;
  int rightMotorSpeed = baseSpeed - pidOutput;
  // Constrain speeds to valid PWM range
  leftMotorSpeed = constrain(leftMotorSpeed,- maxSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed,- maxSpeed, maxSpeed);
  /*if (abs(error)>=5){
    if(rightMotorSpeed<leftMotorSpeed)
      rightMotorSpeed=0;
    else
      leftMotorSpeed=0;
  }*/
  // Set motor speeds
  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);
}


int activeCount() {
  readSensors();
  int count = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorCalibrated[i] == 1) {
      count++;
    }
  }
  return count;
}


void stopMotors() {
  analogWrite(ML_F, 0);
  analogWrite(ML_B, 0);
  analogWrite(MR_F, 0);
  analogWrite(MR_B, 0);
}

void forward(int speed){
  analogWrite(ML_F, speed);
  analogWrite(ML_B, 0);
  analogWrite(MR_F,speed);
  analogWrite(MR_B, 0);
}
void testSensors() {
    Serial.print("Sensors: ");

    for (int i = 0; i < 8; i++) {
        int value = analogRead(sensorPins[i]);

        Serial.print("S");
        Serial.print(i);
        Serial.print("=");
        Serial.print(value);

        if (i < 7) {
            Serial.print(" | ");
        }
    }

    Serial.println();
}
void setup() {
  // Initialize motor pins as OUTPUT
  pinMode(ML_F, OUTPUT);
  pinMode(ML_B, OUTPUT);
  pinMode(MR_F, OUTPUT);
  pinMode(MR_B, OUTPUT);
  pinMode(led,OUTPUT);
  pinMode(BUTTON_PIN,INPUT_PULLUP);
  // Initialize serial communication for debugging
  Serial.begin(115200);  
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }// Calibrate sensors at startup
  calibrateSensors(); 
  while(digitalRead(BUTTON_PIN)==1){}
  t0=millis();
}
void loop() {
  readSensors();
  calculateError(NORMALERROR);
  calculatePID();
  delay(5);
  }