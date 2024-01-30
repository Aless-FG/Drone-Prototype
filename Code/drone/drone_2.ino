#include <Arduino_LSM6DS3.h>

void setup() {
  Serial.begin(9600);
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

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");

  Serial.print("Temperature sensor sample rate = ");
  Serial.print(IMU.temperatureSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Temperature reading in degrees C");
  Serial.println("T");
}

void loop() {
  float x_gyro, y_gyro, z_gyro;
  float x_acc, y_acc, z_acc;
  float t;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.temperatureAvailable()) {
    IMU.readAcceleration(x_acc, y_acc, z_acc);
    IMU.readGyroscope(x_gyro, y_gyro, z_gyro);
    IMU.readTemperature(t);

    Serial.println(String(x_acc)+";"+String(y_acc)+";"+String(z_acc)+";"
                   +String(x_gyro)+";"+String(y_gyro)+";"+String(z_gyro)+";"
                   +String(t));
    }

}

