/*
  Arduino LSM6DS3 - Simple Accelerometer

  This example reads the acceleration values from the LSM6DS3
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Uno WiFi Rev 2 or Arduino Nano 33 IoT

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#include <Arduino_LSM6DS3.h>
#include <Math.h>
#include <MadgwickAHRS.h>
#include <PID_v1.h>

int enA = 9;
int in1 = 8;
int in2 = 7;

// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

// Motor C connections
int enC = 6;
int in5 = 1;
int in6 = 2;

// Motor D connections
int enD = 10;
int in7 = 11;
int in8 = 12;
double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput;

//Define the aggressive and conservative Tuning Parameters
double consKp = 0.5, consKi = 0.05, consKd = 0.05;
int targetSpeed[4];
PID pitchPID(&rollInput, &rollOutput, &rollSetpoint, consKp, consKi, consKd, DIRECT);
PID rollPID(&pitchInput, &pitchOutput, &pitchSetpoint, consKp, consKi, consKd, DIRECT);
Madgwick filter;
void setup() {
  filter.begin(10);
  Serial.begin(9600);
  pitchInput = 0.0;
  rollInput = 0.0;  
  pitchSetpoint = 0.0;
  rollSetpoint = 0.0;
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);

  pitchPID.SetOutputLimits(-20, 20);
  rollPID.SetOutputLimits(-20, 20);
  for (int i = 0; i < 4; i++) {
    targetSpeed[i] = 0;
  } 

  pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(enC, OUTPUT);
	pinMode(enD, OUTPUT);
 
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	pinMode(in5, OUTPUT);
	pinMode(in6, OUTPUT);
	pinMode(in7, OUTPUT);
	pinMode(in8, OUTPUT);

  digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
	digitalWrite(in5, LOW);
	digitalWrite(in6, LOW);
	digitalWrite(in7, LOW);
	digitalWrite(in8, LOW);

  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");
}

void loop() {
  char buffer[5];
  float x, y, z;
  float xg, yg, zg;
  for (int i = 0; i < 4; i++) {
      targetSpeed[i] = 35;
    }
  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
    IMU.readGyroscope(xg, yg, zg);
    IMU.readAcceleration(x, y, z);
    filter.updateIMU(xg,yg,zg, x,y,z);

    //Serial.print(dtostrf(filter.getRoll(), 4, 0, buffer));
    //Serial.print(dtostrf(filter.getPitch(), 4, 0, buffer));
    //Serial.print(dtostrf(filter.getYaw(), 4, 0, buffer));
    //Serial.println("");
    String rollInputString = (dtostrf(filter.getRoll(), 4, 0, buffer));
    String pitchInputString = (dtostrf(filter.getPitch(), 4, 0, buffer));
    //Serial.println(rollInputString);
    //Serial.println(pitchInputString);
    rollInput = atof(rollInputString.c_str());
    pitchInput = atof(pitchInputString.c_str());

    
    pitchPID.Compute();
    rollPID.Compute();
    int actSpeed[4];
    stabilise (targetSpeed, actSpeed, rollOutput, pitchOutput);
  }
  
}

void stabilise (int* currSpeed, int* actSpeed, float rollDiff, float pitchDiff) {
  //actual Speed is calculated as follows +- half rollDiff +- half pitchDiff
  actSpeed[0] = (int) currSpeed[0] + (rollDiff / 2) - (pitchDiff / 2);
  
  actSpeed[1] = (int) currSpeed[1] + (rollDiff / 2) + (pitchDiff / 2);
  actSpeed[2] = (int) currSpeed[2] - (rollDiff / 2) + (pitchDiff / 2);
  actSpeed[3] = (int) currSpeed[3] - (rollDiff / 2) - (pitchDiff / 2);
  Serial.println(actSpeed[0]);
  Serial.println(actSpeed[1]);
  Serial.println(actSpeed[2]);
  Serial.println(actSpeed[3]);
  for (int i = 0; i < 4; i ++) {
    if (actSpeed[i] < 0) 
      actSpeed[i] = 0;  
  }
}