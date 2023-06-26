#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

Adafruit_MPU6050 mpu;
MAX30105 particleSensor;

const int TCA9548A_ADDRESS = 0x70; // I2C address of the TCA9548A multiplexer
const byte CHANNEL_0 = 0;          // Channel 0 of the multiplexer mpu6050
const byte CHANNEL_1 = 1;          // Channel 1 of the multiplexer

const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;


#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid


void setup(void) {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << CHANNEL_0); // Enable channel 0
  Wire.endTransmission();
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);

}

void hr() {

}


void gyro(){
  Wire.begin();
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << CHANNEL_0); // Enable channel 0
  Wire.endTransmission();

  if(mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Print out the values */
    Serial.print("AccelX:");
    Serial.print(a.acceleration.x);
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(a.acceleration.y);
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.print(a.acceleration.z);
    Serial.print(", ");
    Serial.print("GyroX:");
    Serial.print(g.gyro.x);
    Serial.print(",");
    Serial.print("GyroY:");
    Serial.print(g.gyro.y);
    Serial.print(",");
    Serial.print("GyroZ:");
    Serial.print(g.gyro.z);
    Serial.println("");
  }
  
  
  }

void loop() {
  Wire.begin();
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << CHANNEL_0); // Enable channel 0
  Wire.endTransmission();

  if(mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Print out the values */
    Serial.print("AccelX:");
    Serial.print(a.acceleration.x);
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(a.acceleration.y);
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.print(a.acceleration.z);
    Serial.print(", ");
    Serial.print("GyroX:");
    Serial.print(g.gyro.x);
    Serial.print(",");
    Serial.print("GyroY:");
    Serial.print(g.gyro.y);
    Serial.print(",");
    Serial.print("GyroZ:");
    Serial.print(g.gyro.z);
    Serial.println("");
  }

}

void setup1(){
  Serial.begin(115200);
  // Initialize the MAX30105 sensor
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << CHANNEL_1);
  Wire.endTransmission();
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  
  
  }


void loop1(){
    Wire.begin();
  // Switch to channel 1 of the TCA9548A multiplexer
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << CHANNEL_1); // Enable channel 1
  Wire.endTransmission();

  const byte bufferLength = 100; // buffer length of 100 stores 4 seconds of samples running at 25sps
  byte sampleCount = 0; // counter for the number of samples taken

  // arrays to store samples
  uint32_t redBuffer[bufferLength];
  uint32_t irBuffer[bufferLength];

  // read the first 100 samples and determine the signal range
  for (byte i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false) // do we have new data?
      particleSensor.check(); // Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // We're finished with this sample so move to next sample
  }

  // calculate heart rate and SpO2 after the first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // continuously taking samples from MAX30102. Heart rate and SpO2 are calculated every 1 second
  while (1) {
    // dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < bufferLength; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    // take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < bufferLength; i++) {
      while (particleSensor.available() == false) // do we have new data?
        particleSensor.check(); // Check the sensor for new data

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); // We're finished with this sample so move to next sample

      sampleCount++;

      if (sampleCount == 25) {
        // After gathering 25 new samples, recalculate HR and SpO2 and print the average values
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

        // Print the average heart rate and SpO2 values
        Serial.print("Average HR: ");
        Serial.print(heartRate);
        Serial.print(", Average SpO2: ");
        Serial.println(spo2);

        // Reset the sample count
        sampleCount = 0;
      }
    }
  }
  
  
  }
