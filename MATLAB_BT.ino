#include <BluetoothSerial.h>
#include <DWMSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#define SDA_PIN 21
#define SCL_PIN 22
#define NUM_SAMPLES 200

const int PIN_CS = 5;
BluetoothSerial SerialBT;
MPU9250_asukiaaa IMU;
DwmSpi DWM;

// IMU Variabels
float ax, ay, gz;
float ax_offset, ay_offset, gz_offset;
float dt = 0;
unsigned long prevTime = 0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32");
  Serial.println("Bluetooth Initierad");

  DWM.begin();
  DWM.dwm_upd_rate_set();
  
  Wire.begin(SDA_PIN, SCL_PIN);
  IMU.setWire(&Wire);
  IMU.beginAccel();
  IMU.beginGyro();
  Serial.println("IMU Initierad");

  for (int i = 0; i < NUM_SAMPLES; i++) {
    IMU.accelUpdate();
    IMU.gyroUpdate();
    ax_offset += IMU.accelX();
    ay_offset += IMU.accelY();
    gz_offset += IMU.gyroZ();
    delay(5);
  }
  ax_offset /= NUM_SAMPLES;
  ay_offset /= NUM_SAMPLES;
  gz_offset /= NUM_SAMPLES;
  Serial.println("IMU Calibrerad");
}

void loop() {
  // Calculate timestep
  unsigned long currentTime = micros();
  dt = (currentTime - prevTime) / 1e6;
  prevTime = currentTime;

  // Get Data
  IMU.accelUpdate();
  IMU.gyroUpdate();
  DWM.dwm_pos_get();
  
  // Send data in CVS
  SerialBT.print(IMU.accelX() - ax_offset, 6); SerialBT.print(",");
  SerialBT.print(IMU.accelY() - ay_offset, 6); SerialBT.print(",");
  SerialBT.print(IMU.gyroZ() - gz_offset, 6); SerialBT.print(",");
  SerialBT.print(DWM.position.x); SerialBT.print(",");
  SerialBT.print(DWM.position.y); SerialBT.print(",");
  SerialBT.println(dt, 6);

  //Serial.print(IMU.accelX() - ax_offset, 6); Serial.print(",");
  //Serial.print(IMU.accelY() - ay_offset, 6); Serial.print(",");
  //Serial.print(IMU.gyroZ() - gz_offset, 6); Serial.print(",");
  //Serial.print(DWM.position.x); Serial.print(",");
  //Serial.print(DWM.position.y); Serial.println(",");
  //Serial.println(dt, 6);
  delay(10);
}