#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <vector>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29); // BNO055 uses address 0x28 by default, adjust if necessary

// BLE Service and Characteristic UUIDs - replace these with your generated UUIDs
#define SERVICE_UUID        "DF816183-E1F1-4BF9-8B7D-3B794FA03C03"
#define CHARACTERISTIC_UUID_TX "AE40D235-6AA7-4764-89AB-108562631979" // TX for transmitting data
#define CHARACTERISTIC_UUID_RX "12345678-1234-5678-1234-56789abcdef0" // RX for receiving data
#define SDA_PIN 23
#define SCL_PIN 22

enum ExerciseType { BICEP_CURL, BENCH_PRESS, SQUAT };
enum DataType { WARMUP, WORKING };
DataType currentDataType = WARMUP;  // Default to WARMUP

ExerciseType currentExercise = SQUAT; // Squat default
volatile bool shouldGenerateData = false;

struct myVector3 {
    float x, y, z;
};

// Global variables for position and velocity
myVector3 position = {0, 0, 0};
myVector3 velocity = {0, 0, 0};

// Parameters for the sine wave that simulates the motion
const float amplitude = 1.0; // Maximum range of motion
const float frequency = 0.5; // Cycles per second
const float phase = 0; // Phase shift, if needed

unsigned long startTime = 0;
float fatigueVariability = 1.0; // Initial value

BLECharacteristic *pCharacteristicTX; // For sending data
BLECharacteristic *pCharacteristicRX; // For receiving data

void resetWorkingSetVariables();

// Callback for receiving data
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (!rxValue.empty()) {
          Serial.print("Received Value: ");
          for (auto &c: rxValue) {
              Serial.print(c);
          }
          Serial.println();

          // Trim and compare
          rxValue.erase(std::remove(rxValue.begin(), rxValue.end(), '\r'), rxValue.end());
          rxValue.erase(std::remove(rxValue.begin(), rxValue.end(), '\n'), rxValue.end());

          if (rxValue == "START") {
              shouldGenerateData = true;
             // Serial.println("START command recognized, resuming data generation.");
          } else if (rxValue == "STOP") {
              shouldGenerateData = false;
              //Serial.println("STOP command recognized, halting data generation.");
              resetWorkingSetVariables();
          } else if (rxValue == "BENCH_PRESS") {
              currentExercise = BENCH_PRESS;
             // Serial.println("BENCH_PRESS recognized, shouldGenerateData set to true");
          } else if (rxValue == "BICEP_CURL") {
              currentExercise = BICEP_CURL;
             // Serial.println("BICEP_CURL recognized, shouldGenerateData set to true");
          } else if (rxValue == "SQUAT") {
              currentExercise = SQUAT;
              //Serial.println("SQUAT recognized, shouldGenerateData set to true");
          } else if (rxValue == "WARMUP") {
              currentDataType = WARMUP;
              //Serial.println("WARMUP mode set, generating warmup data.");
          } else if (rxValue == "WORKING") {
              currentDataType = WORKING;
             // Serial.println("WORKING mode set, generating working set data.");
          } else {
              //Serial.print("Unknown command received: ");
              for (char c: rxValue) {
                //  Serial.print(c, HEX);
                 // Serial.print(" ");
              }
              //Serial.println();
          }
      } else {
         // Serial.println("Received empty or null value.");
      }
  }
};

// Helper function to reset working set variables
void resetWorkingSetVariables() {
    startTime = millis();
    fatigueVariability = 1.0; // Reset variability
}

// Generates simulated data based on the exercise type
void generateSimulatedWarmupData(myVector3& pos, myVector3& vel, unsigned long currentTime, ExerciseType exercise) {

    float t = currentTime / 1000.0; // Convert time to seconds

    // Define different motion characteristics for each axis
    float amplitudeX, amplitudeY, amplitudeZ, frequencyX, frequencyY, frequencyZ;

    // Variability factor: adjust these values to control the randomness range
    float variabilityAmplitude = 0.02; // 2% variability in amplitude
    float variabilityFrequency = 0.02; // 2% variability in frequency

    // Adding a random component to amplitudes and frequencies
    // random(-100, 100) gives a random number between -100 and 100
    float randomAmplitudeFactor = 1.0 + random(-100, 100) / 1000.0 * variabilityAmplitude;
    float randomFrequencyFactor = 1.0 + random(-100, 100) / 1000.0 * variabilityFrequency;

    switch (exercise) {
        case BENCH_PRESS:
            // Adjusting amplitude and offset for each axis based on specified ranges
            amplitudeX = ((4.0 + 30.0) / 2.0) * randomAmplitudeFactor; // Half the total range for x
            amplitudeY = 4.0 * randomAmplitudeFactor; // Total range for y
            amplitudeZ = ((62.0 + 3.0) / 2.0) * randomAmplitudeFactor; // Half the total range for z

            frequencyX = frequencyY = frequencyZ = 0.33 * randomFrequencyFactor; // Assuming a complete cycle takes about 3 seconds

            // Simulating movements in x, y, and z directions with adjustments
            // x-axis: Mainly negative movement with a slight positive allowance
            pos.x = -amplitudeX * cos(TWO_PI * frequencyX * t) + (4.0 - amplitudeX);
            
            // y-axis: Slight lateral movement left and right
            pos.y = amplitudeY * sin(TWO_PI * frequencyY * t);
            
            // z-axis: Upward and downward movement, simulating bench press motion
            // Adjusted to start from +3 cm (shifted up by adding +3 to the oscillation)
            pos.z = -amplitudeZ * cos(TWO_PI * frequencyZ * t) + 3;

            // Velocity calculation as the derivative of position for each axis
            vel.x = amplitudeX * TWO_PI * frequencyX * sin(TWO_PI * frequencyX * t);
            vel.y = amplitudeY * TWO_PI * frequencyY * cos(TWO_PI * frequencyY * t);
            vel.z = amplitudeZ * TWO_PI * frequencyZ * sin(TWO_PI * frequencyZ * t);
            break;
        case BICEP_CURL:
            // Total range calculations
            amplitudeX = (35.0 / 2.0) * randomAmplitudeFactor; // Since it goes from 0 to +35 and back to 0, half the range for full amplitude
            amplitudeY = 3.0  * randomAmplitudeFactor; // Total range for y is -3 to +3 cm
            amplitudeZ = (37.0 / 2.0)  * randomAmplitudeFactor; // Total range from -1 to +36 cm is 37 cm, half for amplitude
            
            // Frequencies
            frequencyX = (0.5 * 2) * randomFrequencyFactor; // x oscillates twice as fast as the typical movement
            frequencyY = 0.5 * randomFrequencyFactor; // Standard oscillation frequency
            frequencyZ = 0.5 * randomFrequencyFactor; // Standard oscillation frequency for up and down movement

            // Adjusting formula to match the specified ranges for each axis
            pos.x = amplitudeX * sin(TWO_PI * frequencyX * t); // Faster oscillation in x
            pos.y = amplitudeY * sin(TWO_PI * frequencyY * t); // Oscillation in y

            // For z-axis: simulate the curl motion going up and then down
            pos.z = -amplitudeZ * cos(TWO_PI * frequencyZ * t) + 36.0; // Offset to start at +36cm and move down

            // Velocity calculations as the derivative of position for each axis
            vel.x = amplitudeX * TWO_PI * frequencyX * cos(TWO_PI * frequencyX * t);
            vel.y = amplitudeY * TWO_PI * frequencyY * cos(TWO_PI * frequencyY * t);
            vel.z = amplitudeZ * TWO_PI * frequencyZ * sin(TWO_PI * frequencyZ * t);
            break;
        case SQUAT:
            amplitudeX = 8.0 * randomAmplitudeFactor; // Max deviation in cm, forward/backward
            amplitudeY = 4.0 * randomAmplitudeFactor; // Max deviation in cm, left/right
            amplitudeZ = 33.0 * randomAmplitudeFactor; // Half range for movement from +1 to -65 cm
            frequencyX = frequencyY = frequencyZ = 0.25 * randomFrequencyFactor; // Assuming a complete squat takes about 4 seconds

            // Simulating slight movements in x and y directions
            pos.x = amplitudeX * sin(TWO_PI * frequencyX * t);
            pos.y = amplitudeY * sin(TWO_PI * frequencyY * t);
            
            // For z-axis: simulate standing up to squatting down and back up
            // The range is adjusted from +1 to -65 cm, which is a total movement of 66 cm.
            // Since cos(0) = 1, starting value of pos.z should be +1 cm when t=0.
            // To achieve this, we adjust the cos wave to oscillate between 0 (standing) to -66 (squatting down),
            // then we add +1 to shift the entire range up by 1 cm.
            pos.z = -amplitudeZ * cos(TWO_PI * frequencyZ * t) + 1; // Correct formula


            // Velocity calculation as the derivative of position
            vel.x = amplitudeX * TWO_PI * frequencyX * cos(TWO_PI * frequencyX * t);
            vel.y = amplitudeY * TWO_PI * frequencyY * cos(TWO_PI * frequencyY * t);
            vel.z = amplitudeZ * TWO_PI * frequencyZ * sin(TWO_PI * frequencyZ * t);
            break;
        }
    
}

// Generates simulated data based on the exercise type
// Simplified fatigue simulation in working data generation
void generateSimulatedWorkingData(myVector3& pos, myVector3& vel, unsigned long currentTime, ExerciseType exercise) {
    float t = currentTime / 1000.0; // Convert time to seconds

    // Base motion characteristics for each axis
    float amplitudeX, amplitudeY, amplitudeZ, frequencyX, frequencyY, frequencyZ;

    // Base variability for amplitude and frequency
    float baseVariabilityAmplitude = 0.05; // 5%
    float baseVariabilityFrequency = 0.05; // 5%

    // Fatigue impact: As time increases, increase both the amplitude and variability
    float timeBasedIncreaseFactor = min(t / 60.0, 1.0); // Caps at 100% increase at or beyond 1 minute
    float variabilityAmplitude = baseVariabilityAmplitude * (1.0 + timeBasedIncreaseFactor); // Increases up to 10%
    float variabilityFrequency = baseVariabilityFrequency * (1.0 + timeBasedIncreaseFactor); // Increases up to 10%
    float amplitudeIncreaseFactor = 1.0 + timeBasedIncreaseFactor; // Amplitude can double

    // Adjust amplitude and frequency based on exercise and time-based fatigue simulation
    switch (exercise) {
        case BENCH_PRESS:
            amplitudeX = 17.0 * amplitudeIncreaseFactor; // Example: average of initial range, then increase
            amplitudeY = 4.0 * amplitudeIncreaseFactor;
            amplitudeZ = 32.5 * amplitudeIncreaseFactor;
            frequencyX = frequencyY = frequencyZ = 0.33; // Keeping base frequency, could vary if desired
            break;
        case BICEP_CURL:
            amplitudeX = 17.5 * amplitudeIncreaseFactor;
            amplitudeY = 3.0 * amplitudeIncreaseFactor;
            amplitudeZ = 18.5 * amplitudeIncreaseFactor;
            frequencyX = frequencyY = frequencyZ = 0.5;
            break;
        case SQUAT:
            amplitudeX = 8.0 * amplitudeIncreaseFactor;
            amplitudeY = 4.0 * amplitudeIncreaseFactor;
            amplitudeZ = 33.0 * amplitudeIncreaseFactor;
            frequencyX = frequencyY = frequencyZ = 0.25;
            break;
    }

    // Calculate position and velocity with added variability and simulate movements
    pos.x = amplitudeX * sin(TWO_PI * frequencyX * t * (1 + random(-100, 100) / 1000.0 * variabilityFrequency));
    pos.y = amplitudeY * sin(TWO_PI * frequencyY * t * (1 + random(-100, 100) / 1000.0 * variabilityFrequency));
    pos.z = amplitudeZ * cos(TWO_PI * frequencyZ * t * (1 + random(-100, 100) / 1000.0 * variabilityFrequency));
    // Note: Adjusting cos to sin or vice versa depending on desired movement pattern

    // Velocity calculation: derivative of position
    vel.x = 2 * PI * frequencyX * amplitudeX * cos(TWO_PI * frequencyX * t * (1 + random(-100, 100) / 1000.0 * variabilityFrequency));
    vel.y = 2 * PI * frequencyY * amplitudeY * cos(TWO_PI * frequencyY * t * (1 + random(-100, 100) / 1000.0 * variabilityFrequency));
    vel.z = -2 * PI * frequencyZ * amplitudeZ * sin(TWO_PI * frequencyZ * t * (1 + random(-100, 100) / 1000.0 * variabilityFrequency));
}


void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);


  BLEDevice::init("ESP32_BNO055_BLE");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Characteristic for sending data
  pCharacteristicTX = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_READ | 
                        BLECharacteristic::PROPERTY_NOTIFY
                      );

  // Characteristic for receiving data
  pCharacteristicRX = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_WRITE
                      );

  // Assign the callback to the characteristic for receiving data
  pCharacteristicRX->setCallbacks(new MyCallbacks());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("BLE service started, broadcasting...");
}

// Helper function to calculate a simple checksum
uint8_t calculateChecksum(const uint8_t *data, size_t len) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// Transmit position or velocity data
// 'type' can be 'P' for position or 'V' for velocity
// 'x', 'y', 'z' are the data points
// 'timestamp' is the current time in milliseconds
void sendData(char type, float x, float y, float z) {
    uint8_t dataBuffer[16];
    uint8_t checksum = 0;
    int index = 0;

    // Start marker
    dataBuffer[index++] = 0x02;

    // Packet header for data type ('P' or 'V')
    dataBuffer[index++] = type;
    memcpy(dataBuffer + index, &x, sizeof(x));
    index += sizeof(x);
    memcpy(dataBuffer + index, &y, sizeof(y));
    index += sizeof(y);
    memcpy(dataBuffer + index, &z, sizeof(z));
    index += sizeof(z);

  // Calculate checksum excluding the start marker
  for (int i = 1; i < index; i++) {
    checksum ^= dataBuffer[i]; // XOR for simplicity
  }

  // Append checksum and end marker
  dataBuffer[index++] = checksum;
  dataBuffer[index++] = 0x03;

  // Transmit the packet
  pCharacteristicTX->setValue(dataBuffer, index);
  pCharacteristicTX->notify();
}

void loop() {

  Serial.print("shouldGenerateData status: ");
  Serial.println(shouldGenerateData ? "true" : "false");
  

  // Print the current data type
  //Serial.print("Current Data Type: ");
  if (currentDataType == WARMUP) {
    //Serial.println("WARMUP");
  } else if (currentDataType == WORKING) {
    //Serial.println("WORKING");
  } else {
    //Serial.println("Unknown");
  }

  if (!shouldGenerateData) {
    // If we shouldn't generate data, just return and check again
    delay(100); // Short delay to prevent spamming
    return;
  }

  float currentTime = millis();

  // Generate simulated data
  myVector3 simulatedPos, simulatedVel;

  // Depending on the currentDataType, generate the appropriate set of data
  if (currentDataType == WARMUP) {
      generateSimulatedWarmupData(simulatedPos, simulatedVel, currentTime, currentExercise);
  } else if (currentDataType == WORKING) {
      generateSimulatedWorkingData(simulatedPos, simulatedVel, currentTime, currentExercise);
  }

/*
  // Print and send position data
  Serial.print("Exercise: ");
  switch (currentExercise) {
      case BENCH_PRESS: Serial.println("BENCH_PRESS"); break;
      case BICEP_CURL: Serial.println("BICEP_CURL"); break;
      case SQUAT: Serial.println("SQUAT"); break;
  }
  */
/*
  Serial.print(simulatedPos.x);
  Serial.print(" ");
  Serial.print(simulatedPos.y);
  Serial.print(" ");
  Serial.println(simulatedPos.z);
*/
  // Send position data
  sendData('P', simulatedPos.x, simulatedPos.y, simulatedPos.z);
  delay(100);
  
  Serial.print(simulatedVel.x);
  Serial.print(" ");
  Serial.print(simulatedVel.y);
  Serial.print(" ");
  Serial.println(simulatedVel.z);

  // Send position data
  sendData('V', simulatedVel.x, simulatedVel.y, simulatedVel.z);
  delay(500);
}
