/*
 * v0.2 2016 Jul. 05
 *   - add calibration example
 * v0.1 2016 Jul. 03
 *   - remove Calibration()
 *   - use Interrupt
 *   - import for MPU-9250
 *   - based on the code at
 *   http://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/
 */

#include "I2Cdev.h"
#include "MPU9150_9Axis_MotionApps41.h"
#include "Wire.h"
#include <ESP8266WiFi.h>
MPU9150 accelgyro;

#define INTERRUPT_PIN 14  // ESP8266

int buffersize = 1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int16_t ax, ay, az,gx, gy, gz;

int mean_ax, mean_ay, mean_az;
int mean_gx, mean_gy, mean_gz;
int state=0;
int ax_offset, ay_offset, az_offset;
int gx_offset, gy_offset, gz_offset;

uint16_t packetSize;
uint16_t fifoCount;

void meansensors();
void showData();

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  WiFi.disconnect();
  Wire.begin();
  Wire.setClock(400000L); // 400kHz
  Serial.begin(115200);

  accelgyro.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()){
    Serial.println(F("Send any character to start sketch.\n"));
    delay(1500);
  }                
  while (Serial.available() && Serial.read()); // empty buffer again

  packetSize = accelgyro.dmpGetFIFOPacketSize();

  Serial.println("\nYour MPU-9250 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
  delay(3000);
//  // verify connection
//  Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
//  delay(1000);
}

void loop() {
  if (state==0){
    Serial.println("\nReading sensors for first time...");
    meansensors();
    state++;
    delay(1000);
  }

  if (state==1) {
    Serial.println("\nMeasuring...");
    showData(); 
    state++;
    delay(1000);
  }

  if (state==2) {
    meansensors();
    Serial.print("\nSensor readings with zero offsets:\t");
    Serial.print(mean_ax); 
    Serial.print("\t");
    Serial.print(mean_ay); 
    Serial.print("\t");
    Serial.print(mean_az); 
    Serial.print("\t");
    Serial.print(mean_gx); 
    Serial.print("\t");
    Serial.print(mean_gy); 
    Serial.print("\t");
    Serial.println(mean_gz);
    Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
    Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
    state++;
  }
}

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void meansensors(){
  long i=0;
  int buff_ax=0;
  int buff_ay=0;
  int buff_az=0;
  int buff_gx=0;
  int buff_gy=0;
  int buff_gz=0;

  while (i < (buffersize + 101) ){
    fifoCount = accelgyro.getFIFOCount();
    if (!mpuInterrupt && fifoCount < packetSize) {
      continue;
    }
    mpuInterrupt = false;
    
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
    if ( (i > 100) && i <= (buffersize + 100) ){ // First 100 discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if ( i == (buffersize + 100) ) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
  }
}

void showData()
{
  Serial.println(accelgyro.getXAccelOffset());
  Serial.println(accelgyro.getYAccelOffset());
  Serial.println(accelgyro.getZAccelOffset());
  
//  accelgyro.setXAccelOffset(0);
//  accelgyro.setYAccelOffset(0);
//  accelgyro.setZAccelOffset(0);
//  accelgyro.setXGyroOffset(0);
//  accelgyro.setYGyroOffset(0);
//  accelgyro.setZGyroOffset(0);

  for(int loop = 0; loop < 10; loop++) {
    meansensors();
    Serial.print(" ,mean_ax:");
    Serial.print(mean_ax);
    Serial.print(" ,mean_ay:");
    Serial.print(mean_ay);
    Serial.print(" ,mean_az:");
    Serial.print(mean_az);
    //
    Serial.print(" ,mean_gx:");
    Serial.print(mean_gx);
    Serial.print(" ,mean_gy:");
    Serial.print(mean_gy);
    Serial.print(" ,mean_gz:");
    Serial.print(mean_gz);
    //
    Serial.println();
  }

  accelgyro.setXAccelOffset(-5170 - 8); // divide by 8, but should be even not odd number
  accelgyro.setYAccelOffset(5390 - 38); // divide by 8, but should be even not odd number 
  accelgyro.setZAccelOffset(8424 + 16); // divide by 8, but should be even not odd number
  accelgyro.setXGyroOffset( 80 / 4); // divide by 4 
  accelgyro.setYGyroOffset( 8 / 4); // divide by 4
  accelgyro.setZGyroOffset( 286 / 4); // divide by 4

  for(int loop = 0; loop < 10; loop++) {
    meansensors();
    Serial.print(" ,mean_ax:");
    Serial.print(mean_ax);
    Serial.print(" ,mean_ay:");
    Serial.print(mean_ay);
    Serial.print(" ,mean_az:");
    Serial.print(mean_az);
    //
    Serial.print(" ,mean_gx:");
    Serial.print(mean_gx);
    Serial.print(" ,mean_gy:");
    Serial.print(mean_gy);
    Serial.print(" ,mean_gz:");
    Serial.print(mean_gz);
    //
    Serial.println();
  }
  
}

