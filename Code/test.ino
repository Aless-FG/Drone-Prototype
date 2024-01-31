#include <Arduino_LSM6DS3.h>

// ------------- Motors connections -------------

// Motor A connections
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

// -----------------------------------------------

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  // Set all the motor control pins to outputs
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
 
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
	digitalWrite(in5, LOW);
	digitalWrite(in6, LOW);
	digitalWrite(in7, LOW);
	digitalWrite(in8, LOW);

  Serial.print("Accelerometer reading...");
  Serial.print("Gyroscope reading...");
  Serial.print("Temperature reading in Celsius...");
  Serial.println();

}

void loop() {
  float x_gyro, y_gyro, z_gyro;
  float x_acc, y_acc, z_acc;
  float t;

  // Turn on motors
	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	digitalWrite(in3, LOW);
	digitalWrite(in4, HIGH);
 
	digitalWrite(in5, LOW);
	digitalWrite(in6, HIGH);
	digitalWrite(in7, LOW);
	digitalWrite(in8, HIGH);

  // Accelerate from zero to maximum speed
	for (int i = 0; i < 50; i++) {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.temperatureAvailable()) {
    analogWrite(enA, i);
		analogWrite(enB, i);
		analogWrite(enC, i);
		analogWrite(enD, i);
    IMU.readAcceleration(x_acc, y_acc, z_acc);
    IMU.readGyroscope(x_gyro, y_gyro, z_gyro);
    t = (IMU.readTemperature(t)/512.0)+23;

    Serial.println(String(x_acc)+";"+String(y_acc)+";"+String(z_acc)+";"
                   +String(x_gyro)+";"+String(y_gyro)+";"+String(z_gyro)+";"
                   +String(t)+";"+String(i));
		
		delay(50);
	  }
  }

  delay(1000);
  
  // Decelerate from maximum speed to zero
	for (int i = 50; i >= 0; --i) {
		if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.temperatureAvailable()) {
    analogWrite(enA, i);
		analogWrite(enB, i);
		analogWrite(enC, i);
		analogWrite(enD, i);

    IMU.readAcceleration(x_acc, y_acc, z_acc);
    IMU.readGyroscope(x_gyro, y_gyro, z_gyro);
    t = (IMU.readTemperature(t)/512.0)+23;

    Serial.println(String(x_acc)+";"+String(y_acc)+";"+String(z_acc)+";"
                   +String(x_gyro)+";"+String(y_gyro)+";"+String(z_gyro)+";"
                   +String(t)+";"+String(i));
		
		delay(50);
	  }
  }

  // Now turn off motors
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
	digitalWrite(in5, LOW);
	digitalWrite(in6, LOW);
	digitalWrite(in7, LOW);
	digitalWrite(in8, LOW);

}

