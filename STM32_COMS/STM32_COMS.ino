
#include <STM32FreeRTOS.h>
#include <Wire.h>
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MS5837.h"  // Include the Blue Robotics MS5837 library
#include "INA219.h"
#include "Servo.h"



// --------- Motor config ----------
Servo FESC1; //Forward Motor 1
Servo FESC2; //Forward Motor 2
Servo DESC1; //Dive Motor 1
Servo DESC2; //Dive motor 2
volatile int state = 0; // 0 : Descend , 1 : Ascend 
unsigned long lastToggleTime = 0;  // Global variable to store the last toggle time
//-----------IMU CONFIG--------------
MPU6050 mpu;
MS5837 sensor;
INA219 INA(0x40);


#define EARTH_GRAVITY_MS2 9.80665  // m/s²
#define RAD_TO_DEG        57.295779513082320876798154814105

bool DMPReady = false;  
uint8_t devStatus;      
uint16_t packetSize;    
uint8_t FIFOBuffer[64]; 

Quaternion q;         
VectorFloat gravity;  
float ypr[3];        
VectorInt16 aaWorld;  

// Global array to store all sensor values (yaw deg, pitch deg, roll deg, accelX m/s^2, accelY m/s^2, accelZ/m/s^2, depth/m, temperature/deg C, bus voltage,shunt voltage/mV, current/mA, power/mW)
volatile float sensorData[13] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0};  
volatile int commandData[5] = {0, 0, 0, 0, 0};  // Global array for serial command data

SemaphoreHandle_t xSensorMutex;  // Mutex for sensor data
SemaphoreHandle_t xCommandMutex;  // Mutex for command data

// Function prototypes
void SerialReceiveTask(void *pvParameters);
void ReadSensorsTask(void *pvParameters);
void MotorControlTask(void *pvParameters);
void StatusLightTask(void *pvParameters);
void printSensorData();

void setup() {
  Serial.begin(921600); 
  delay(2000);  // Allow serial to stabilize

  Wire.begin();
  Wire.setClock(400000);
  // Setup ESC pins
  FESC1.attach(PA6,1000,2000);
  FESC2.attach(PA7,1000,2000);
  DESC1.attach(PB0,1000,2000);
  DESC2.attach(PB1,1000,2000);
  pinMode(PC13, OUTPUT);
  pinMode(PB10, OUTPUT);

  while (!sensor.init()) {
    digitalWrite(PC13, 0);
    delay(5000);
  }

  if (!INA.begin() )
  {
    
  }

   INA.setMaxCurrentShunt(5, 0.002);
   INA.getBusVoltageRange();

  Serial.println("Ready");
  delay(1000);

  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997);  // Fresh water density in kg/m^3


  // Create mutex for protecting shared data
  xSensorMutex = xSemaphoreCreateMutex();
  xCommandMutex = xSemaphoreCreateMutex();

  // Create tasks
  xTaskCreate(ReadSensorsTask, "ReadSensor", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);
 xTaskCreate(MotorControlTask, "Motor Task", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);
  xTaskCreate(StatusLightTask, "StatusLight", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(SerialReceiveTask, "SerialReceive", configMINIMAL_STACK_SIZE * 2, NULL, 3, NULL);

  // Start the scheduler
  vTaskStartScheduler();
}

void loop() {
  // Empty - FreeRTOS handles tasks
}

// Task to listen to serial data and store it in a global array
void SerialReceiveTask(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    if (Serial.available() > 0) {
      String receivedData = Serial.readStringUntil('\n');
      receivedData.trim(); 

      if (receivedData == "PRINT") {
        printSensorData();  // Print the sensor data if "PRINT" command is received
      } else if (receivedData.length() > 0 && receivedData.length() <= 50) {
        char receivedCharArray[51];
        receivedData.toCharArray(receivedCharArray, sizeof(receivedCharArray));

        int tempValues[5];
        int index = 0;
        char *token = strtok(receivedCharArray, ",");
        while (token != NULL && index < 5) {
          tempValues[index++] = atoi(token);
          token = strtok(NULL, ",");
        }

        if (xSemaphoreTake(xCommandMutex, portMAX_DELAY)) {
          for (int i = 0; i < 5; i++) {
            commandData[i] = tempValues[i];
          }
          xSemaphoreGive(xCommandMutex);
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Task to read MPU6050 sensor values
void ReadSensorsTask(void *pvParameters) {
  mpu.initialize();
  if (!mpu.testConnection()) {
    vTaskDelete(NULL);
  } 

  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    vTaskDelete(NULL);
  }

  while (1) {
    sensor.read();

    if (DMPReady && mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { 
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      VectorInt16 aa;
      mpu.dmpGetAccel(&aa, FIFOBuffer);
      mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);

      if (xSemaphoreTake(xSensorMutex, portMAX_DELAY)) {
        sensorData[0] = ypr[0] * RAD_TO_DEG;  // Yaw
        sensorData[1] = ypr[1] * RAD_TO_DEG;  // Pitch
        sensorData[2] = ypr[2] * RAD_TO_DEG;  // Roll

        sensorData[3] = aaWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;  // AccelX
        sensorData[4] = aaWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;  // AccelY
        sensorData[5] = aaWorld.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;  // AccelZ


        sensorData[6] = sensor.depth();         // Depth in meters
        sensorData[7] = sensor.temperature();   // Temperature in K


        sensorData[8] = INA.getBusVoltage();
        sensorData[9] = INA.getShuntVoltage_mV()/1000;
        sensorData[10] = INA.getCurrent_mA ()/1000;
        sensorData[11] = INA.getPower_mW()/1000;
        sensorData[12] = state;
        xSemaphoreGive(xSensorMutex);
      }
    }
    printSensorData();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Task to blink LED status
void StatusLightTask(void *pvParameters) {
  while (1) {
    heartbeat();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// Function to control LED sequence
void heartbeat() {
  int sequence[] = {1, 0, 1, 0, 1, 0, 0, 0, 1, 0};  
  int sequenceLength = sizeof(sequence) / sizeof(sequence[0]);

  for (int i = 0; i < sequenceLength; i++) {  
    digitalWrite(PC13, sequence[i]);  
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Function to print sensor data
//Yaw,Pitch,Roll,Accelx,Accely,Accelz,depth,temp,bus voltage, shunt voltage, current, power,surge, y,x,corrected roll input,button
void printSensorData() {
  if (xSemaphoreTake(xSensorMutex, portMAX_DELAY)) {
     Serial.print(sensorData[0]);Serial.print(",");
     Serial.print(sensorData[1]);Serial.print(",");
     Serial.print(sensorData[2]);Serial.print(",");
     Serial.print(sensorData[3]);Serial.print(",");
     Serial.print(sensorData[4]);Serial.print(",");
     Serial.print(sensorData[5]);Serial.print(",");
     Serial.print(sensorData[6]);Serial.print(",");
     Serial.print(sensorData[7]);Serial.print(",");
     Serial.print(sensorData[8]);Serial.print(",");
     Serial.print(sensorData[9]);Serial.print(",");
     Serial.print(sensorData[10]);Serial.print(",");
     Serial.print(sensorData[11]);Serial.print(",");
     Serial.print(commandData[0]);Serial.print(",");
     Serial.print(commandData[1]);Serial.print(",");
     Serial.print(commandData[2]);Serial.print(",");
     Serial.print(commandData[3]);Serial.print(",");
     Serial.print(commandData[4]);Serial.print(",");
     Serial.println(sensorData[12]);
    xSemaphoreGive(xSensorMutex);
  }
}

// Task to control motors based on command data
void MotorControlTask(void *pvParameters) {
  (void) pvParameters;

  while (1) {
    if (xSemaphoreTake(xCommandMutex, portMAX_DELAY)) {
      // Map throttle (0-100%) to PWM values (0-255)
      int surge = map(commandData[0],  0, 100, 0, 180);
      int dirY = map(commandData[1], -100, 100, -255, 255);
      int dirX = map(commandData[2], -100, 100, -255, 255);
      int C_roll = map(commandData[3], 0, 100, 0, 255);
      int button = commandData[4]; //if 16 we are in reverse mode set reverse relay high

     if (button == 16 && (millis() - lastToggleTime >= 100)) {
           state = !state;              // Toggle state
          lastToggleTime = millis();   // Update the last toggle time
       }

  // Set PB10 HIGH or LOW based on state
      if(state) {
        digitalWrite(PB10, HIGH);
      }
      else {
        digitalWrite(PB10, LOW);
      }
    
      // Example motor control logic
      analogWrite(PA6, surge);  // Motor 1 Speed
      analogWrite(PA7, dirY);       // Motor 2 Speed
      analogWrite(PB0, dirY);       // Motor 3 Speed
      analogWrite(PB1, C_roll);       // Motor 4 Speed

      // Example: Use the button command to stop motors
      

      xSemaphoreGive(xCommandMutex);
    }
    
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Control loop delay
  }
}
