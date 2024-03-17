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

struct Quaternion {
    float w, x, y, z;
};

typedef struct {
    float linAccX;
    float linAccY;
    float linAccZ;
} LinearAcceleration;

typedef struct {
    float gravX;
    float gravY;
    float gravZ;
} GravityVector;

struct Vector3 {
    float x, y, z;
};

// Global variables for position and velocity
Vector3 position = {0, 0, 0};
Vector3 velocity = {0, 0, 0};

std::vector<LinearAcceleration> accelerationSamples;


unsigned long lastSampleTime = 0; // Tracks the last time sensor data was sampled
unsigned long lastPositionUpdateTime = 0; // Tracks the last time the position was updated

const unsigned long sampleInterval = 10; // Sample data every 10ms
const unsigned long positionUpdateInterval = 100; // Calculate position every 40ms

const float stationaryThresholdX = 0.1;
const float stationaryThresholdY = 0.1;
const float stationaryThresholdZ = 0.1;

// Global variable for tracking time since last position update calculation
static unsigned long lastPositionCalculationTime = 0;

// Initialize quaternion, linear acceleration, and gravity vector structs globally
Quaternion quatStruct = {0.0, 0.0, 0.0, 0.0};
LinearAcceleration linAccStruct = {0.0, 0.0, 0.0};
GravityVector gravVecStruct = {0.0, 0.0, 0.0};

bool isSensorCalibrated = false; // Global flag to track sensor calibration status
bool initialPositionSet = false; // Flag to ensure the initial position is set once

// Previous filtered acceleration values, initialized to 0
LinearAcceleration prevFilteredAcc = {0.0, 0.0, 0.0};

// Global variables to store the bias
float accXBias = 0.0, accYBias = 0.0, accZBias = 0.0;

// all debug
bool DEBUG = false;

// less info
bool DEBUG_LITE = true;

bool isFirstMeasurement = true;
bool isFirstPositionUpdate = true;

BLECharacteristic *pCharacteristicTX; // For sending data
BLECharacteristic *pCharacteristicRX; // For receiving data

// Callback for receiving data
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        Serial.println("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);
        Serial.println();
      }
    }
};

void checkCalibration() {
  Serial.println("Checking sensor calibration...");
  uint8_t system, gyro, accel, mag = 0;
  while (gyro != 3 || accel != 3) {
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("CALIBRATION: Sys=");
    Serial.print(system, DEC);
    Serial.print(" Gyro=");
    Serial.print(gyro, DEC);
    Serial.print(" Accel=");
    Serial.print(accel, DEC);
    Serial.print(" Mag=");
    Serial.println(mag, DEC);
    delay(500);
  }
  Serial.println("Sensor calibrated successfully.");
}

void measureAndStoreBias() {
  const int samples = 1000;
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;

  Serial.println("Measuring sensor biases. Please keep the sensor stationary.");
  delay(3000);

  for (int i = 0; i < samples; i++) {
    imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    sumX += linAccel.x();
    sumY += linAccel.y();
    sumZ += linAccel.z();

    if (i == 0 || (i % 10) == 0) {
      // Print the first sample of linear acceleration as a reference
      Serial.println();
      Serial.print("Sample of Linear Accel: X=");
      Serial.print(linAccel.x(), 4);
      Serial.print(", Y=");
      Serial.print(linAccel.y(), 4);
      Serial.print(", Z=");
      Serial.println(linAccel.z(), 4);
      Serial.println();
    }

    delay(20); // Short delay between samples
  }

  accXBias = sumX / samples;
  accYBias = sumY / samples;
  accZBias = sumZ / samples;

  // Print calculated biases
  Serial.println();
  Serial.print("Calculated Biases - X: ");
  Serial.print(accXBias, 4);
  Serial.print(", Y: ");
  Serial.print(accYBias, 4);
  Serial.print(", Z: ");
  Serial.println(accZBias, 4);
  Serial.println();
}

// Function to check sensor calibration
bool checkSensorCalibration() {
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  // Now checking only gyro and accel for full calibration
  return (gyro == 3 && accel == 3);
}

// Function to perform an initial position and velocity reset
void resetPositionAndVelocity() {
  // Reset position and velocity to zero
  Serial.println("Resetting position and velocity, keep sensor still");
  delay(5000);
  position = {0, 0, 0};
  velocity = {0, 0, 0};
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (bno.begin()) {
      Serial.println("BNO055 detected successfully.");
      bno.setMode(OPERATION_MODE_NDOF);
      checkCalibration();
      measureAndStoreBias();
      resetPositionAndVelocity();
      initialPositionSet = true;

      // Print the initial position data
      Serial.print("Initial Position: x=");
      Serial.print(position.x, 4); // Print with 4 decimal places of precision
      Serial.print(", y=");
      Serial.print(position.y, 4);
      Serial.print(", z=");
      Serial.println(position.z, 4);

      // Debugging: Print initial velocity
      Serial.print("Initial Velocity: X=");
      Serial.print(velocity.x, 4);
      Serial.print(", Y=");
      Serial.print(velocity.y, 4);
      Serial.print(", Z=");
      Serial.println(velocity.z, 4);

  } else {
    Serial.println("No BNO055 detected. Check wiring or I2C address.");
    while (1);
  }

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
  lastPositionCalculationTime = millis();
}

void sendPacket(char header, float x, float y, float z) {
  uint8_t dataBuffer[16]; // FPGA uart limited
  uint8_t checksum = 0;
  int index = 0;

  // Start marker
  dataBuffer[index++] = 0x02;

  // Packet header for data type ('L' or 'G')
  dataBuffer[index++] = header;
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

// Low-pass filter applied to 3D vector data
LinearAcceleration applyLowPassFilter(LinearAcceleration currentData, LinearAcceleration previousFilteredData, float alpha) {
    LinearAcceleration filteredData;
    filteredData.linAccX = alpha * previousFilteredData.linAccX + (1 - alpha) * currentData.linAccX;
    filteredData.linAccY = alpha * previousFilteredData.linAccY + (1 - alpha) * currentData.linAccY;
    filteredData.linAccZ = alpha * previousFilteredData.linAccZ + (1 - alpha) * currentData.linAccZ;
    return filteredData;
}

void sendQuaternionPart(char part, float val1, float val2) {
  uint8_t dataBuffer[16]; // 16-byte limit on axi_uartlite
  uint8_t checksum = 0;
  int index = 0;

  dataBuffer[index++] = 0x02; // Start marker

  // Adjusted packet header for Quaternion part:
  // 'Q' for first part, 'q' for second part, based on input 'part' variable
  if (part == '1') {
      dataBuffer[index++] = 'Q'; // Use 'Q' for the first part
  } else if (part == '2') {
      dataBuffer[index++] = 'q'; // Use 'q' for the second part
  }

  memcpy(dataBuffer + index, &val1, sizeof(val1));
  index += sizeof(val1);
  memcpy(dataBuffer + index, &val2, sizeof(val2));
  index += sizeof(val2);

  // Checksum calculation, excluding start marker
  for (int i = 1; i < index; i++) {
      checksum ^= dataBuffer[i];
  }

  dataBuffer[index++] = checksum; // Append checksum
  dataBuffer[index++] = 0x03; // End marker

  pCharacteristicTX->setValue(dataBuffer, index);
  pCharacteristicTX->notify();
}

void updatePositionAndSend(Quaternion quat, LinearAcceleration linAcc, GravityVector gravity, float deltaT) {

  Serial.println(); // Add a blank line for readability

  // Debugging: Print raw quaternion data before conversion
  if(DEBUG) {
    Serial.print("Quaternion Data: w=");
    Serial.print(quat.w, 4); // Print with 4 decimal places of precision
    Serial.print(", x=");
    Serial.print(quat.x, 4);
    Serial.print(", y=");
    Serial.print(quat.y, 4);
    Serial.print(", z=");
    Serial.println(quat.z, 4);
  }

  // Convert quaternion to rotation matrix
  float R[3][3];
  quaternionToRotationMatrix(quat, R);

  if(DEBUG) {
    Serial.println(); // Add a blank line for readability
    Serial.println("Rotation Matrix:");
    for(int i = 0; i < 3; i++) {
        Serial.print("[");
        for(int j = 0; j < 3; j++) {
            Serial.print(R[i][j], 4); // Print with 4 decimal places of precision
            if (j < 2) Serial.print(", ");
        }
        Serial.println("]");
    }
  }

  // Rotate linear acceleration and gravity vector
  LinearAcceleration adjustedLinAcc = rotateLVector(linAcc, R);

  if(isFirstMeasurement) {
    prevFilteredAcc = adjustedLinAcc;
    isFirstMeasurement = false;
  }
  LinearAcceleration filteredAccel = applyLowPassFilter(adjustedLinAcc, prevFilteredAcc, 0.5);

  prevFilteredAcc = filteredAccel;

  // Debugging: Print adjusted linear acceleration and gravity vectors
  if(DEBUG) {
    Serial.println(); // Add a blank line for readability
    Serial.print("Filtered Linear Acc: X=");
    Serial.print(filteredAccel.linAccX, 4);
    Serial.print(", Y=");
    Serial.print(filteredAccel.linAccY, 4);
    Serial.print(", Z=");
    Serial.println(filteredAccel.linAccZ, 4);
  }

  // checks to find stationary axes
  bool isStationaryX = fabs(filteredAccel.linAccX) < stationaryThresholdX;
  bool isStationaryY = fabs(filteredAccel.linAccY) < stationaryThresholdY;
  bool isStationaryZ = fabs(filteredAccel.linAccZ) < stationaryThresholdZ;

  float deltaX, deltaY, deltaZ;

  // Update velocities conditionally
  if (!isStationaryX) {
      velocity.x += filteredAccel.linAccX * deltaT;
  } else {
      velocity.x = 0;
  }

  if (!isStationaryY) {
      velocity.y += filteredAccel.linAccY * deltaT;
  } else {
      velocity.y = 0;
  }

  if (!isStationaryZ) {
      velocity.z += filteredAccel.linAccZ * deltaT;
  } else {
      velocity.z = 0;
  }

  // Update positions conditionally
  if (!isStationaryX) {
      deltaX = velocity.x * deltaT + 0.5 * filteredAccel.linAccX * pow(deltaT, 2);
      position.x += velocity.x * deltaT + 0.5 * filteredAccel.linAccX * pow(deltaT, 2);
  }
  if (!isStationaryY) {
      deltaY = velocity.y * deltaT + 0.5 * filteredAccel.linAccY * pow(deltaT, 2);
      position.y += velocity.y * deltaT + 0.5 * filteredAccel.linAccY * pow(deltaT, 2);
  }
  if (!isStationaryZ) {
      deltaZ = velocity.z * deltaT + 0.5 * filteredAccel.linAccZ * pow(deltaT, 2);  
      position.z += velocity.z * deltaT + 0.5 * filteredAccel.linAccZ * pow(deltaT, 2);
  }

  if(DEBUG) {
    // Debugging: Print deltaT
    Serial.print("DeltaT: ");
    Serial.println(deltaT, 6);

    // Debugging: Print updated velocity
    Serial.print("Updated Velocity: X=");
    Serial.print(velocity.x, 4);
    Serial.print(", Y=");
    Serial.print(velocity.y, 4);
    Serial.print(", Z=");
    Serial.println(velocity.z, 4);
  }

  // Convert positions to cm for display
  float positionX_cm = position.x * 100;
  float positionY_cm = position.y * 100;
  float positionZ_cm = position.z * 100;

  if(DEBUG && !isStationaryX && !isStationaryY && !isStationaryZ) {
    // Debugging: Print intermediate math for position calculation
    Serial.print("Delta Position X: ");
    Serial.println(deltaX, 4);
    Serial.print("Delta Position Y: ");
    Serial.println(deltaY, 4);
    Serial.print("Delta Position Z: ");
    Serial.println(deltaZ, 4);
  }


 // Serial.println(); // Add a blank line for readability
  // Print the updated position data to the Serial Monitor
  Serial.print("Updated Position in cm: x=");
  Serial.print(positionX_cm, 4); // Print with 4 decimal places of precision
  Serial.print(", y=");
  Serial.print(positionY_cm, 4);
  Serial.print(", z=");
  Serial.println(positionZ_cm, 4);
  //Serial.println(); // Add a blank line for readability

  // Send updated position to FPGA
  sendPosition(position.x, position.y, position.z);
}


void sendPosition(float x, float y, float z) {
    // Prepare and send the position data packet to the FPGA via BLE
    // Similar to sendPacket function but for position data
}

// Function to calculate deltaT in seconds
float calculateDeltaTime() {
    if (isFirstPositionUpdate) {
        isFirstPositionUpdate = false;
        lastPositionCalculationTime = millis();
        return positionUpdateInterval / 1000 ; // Return initial update interval (hardcoded value)
    } else {
        unsigned long currentTime = millis();
        float deltaT = (currentTime - lastPositionCalculationTime) / 1000.0; // Convert milliseconds to seconds
        lastPositionCalculationTime = currentTime; // Update the last position calculation time
        return deltaT;
    }
}

// Function to convert quaternion orientation data into a 3x3 rotation matrix.
// This matrix can then be used to rotate vectors from the local sensor frame to the global frame.
void quaternionToRotationMatrix(Quaternion q, float R[3][3]) {
    float qw = q.w, qx = q.x, qy = q.y, qz = q.z; // Extract quaternion components for easier usage.

    // The following computations are based on quaternion-to-rotation matrix equations.
    // The resulting matrix will be used to rotate the sensor's reference frame into our reference frame, 
    // allowing us to get x, y, and z data relative to gravity instead of relative to the sensor's axes
    R[0][0] = 1 - 2*qy*qy - 2*qz*qz;
    R[0][1] = 2*qx*qy - 2*qz*qw;
    R[0][2] = 2*qx*qz + 2*qy*qw;

    R[1][0] = 2*qx*qy + 2*qz*qw;
    R[1][1] = 1 - 2*qx*qx - 2*qz*qz;
    R[1][2] = 2*qy*qz - 2*qx*qw;

    R[2][0] = 2*qx*qz - 2*qy*qw;
    R[2][1] = 2*qy*qz + 2*qx*qw;
    R[2][2] = 1 - 2*qx*qx - 2*qy*qy;
}

// Function to rotate a vector v by the rotation matrix R, effectively re-orienting the vector in 3D space into our referece frame
GravityVector rotateGVector(GravityVector v, float R[3][3]) {
    GravityVector rotated; // The result of applying the rotation to v.
    // Each component of the rotated vector is a dot product of the row of the rotation matrix with the original vector.
    rotated.gravX = R[0][0]*v.gravX + R[0][1]*v.gravY + R[0][2]*v.gravZ;
    rotated.gravY = R[1][0]*v.gravX + R[1][1]*v.gravY + R[1][2]*v.gravZ;
    rotated.gravZ = R[2][0]*v.gravX + R[2][1]*v.gravY + R[2][2]*v.gravZ;
    return rotated; // Return the rotated vector.
}

LinearAcceleration rotateLVector(LinearAcceleration v, float R[3][3]) {
    LinearAcceleration rotated; // The result of applying the rotation to v.
    // Each component of the rotated vector is a dot product of the row of the rotation matrix with the original vector.
    rotated.linAccX = R[0][0]*v.linAccX + R[0][1]*v.linAccY + R[0][2]*v.linAccZ;
    rotated.linAccY = R[1][0]*v.linAccX + R[1][1]*v.linAccY + R[1][2]*v.linAccZ;
    rotated.linAccZ = R[2][0]*v.linAccX + R[2][1]*v.linAccY + R[2][2]*v.linAccZ;
    return rotated; // Return the rotated vector.
}

Quaternion calculateAverageQ(const std::vector<Quaternion>& samples) {
    Quaternion avg = {0, 0, 0, 0};
    for (const auto& q : samples) {
        avg.w += q.w;
        avg.x += q.x;
        avg.y += q.y;
        avg.z += q.z;
    }
    int n = samples.size();
    if (n > 0) {
        avg.w /= n;
        avg.x /= n;
        avg.y /= n;
        avg.z /= n;
    }
    return avg;
}

LinearAcceleration calculateAverageL(const std::vector<LinearAcceleration>& samples) {
    LinearAcceleration avg = {0, 0, 0};
    for (const auto& acc : samples) {
        avg.linAccX += acc.linAccX;
        avg.linAccY += acc.linAccY;
        avg.linAccZ += acc.linAccZ;
    }
    int n = samples.size();
    if (n > 0) {
        avg.linAccX /= n;
        avg.linAccY /= n;
        avg.linAccZ /= n;
    }
    return avg;
}

GravityVector calculateAverageG(const std::vector<GravityVector>& samples) {
    GravityVector avg = {0, 0, 0};
    for (const auto& gv : samples) {
        avg.gravX += gv.gravX;
        avg.gravY += gv.gravY;
        avg.gravZ += gv.gravZ;
    }
    int n = samples.size();
    if (n > 0) {
        avg.gravX /= n;
        avg.gravY /= n;
        avg.gravZ /= n;
    }
    return avg;
}

void loop() {

  // Only proceed if the sensor is calibrated
  if (!isSensorCalibrated) {
    isSensorCalibrated = checkSensorCalibration(); // Check if the sensor is calibrated
    if (!isSensorCalibrated) {
      Serial.println("Waiting for sensor calibration...");
      delay(100); // Delay a bit before checking again
      return; // Skip the rest of the loop until calibrated
    } else {
      Serial.println("Sensor calibrated successfully.");
    }
  }

  unsigned long currentTime = millis();

  // Perform an initial position reset once right after calibration
  if (!initialPositionSet) {
    resetPositionAndVelocity(); // Reset position and velocity
    initialPositionSet = true; // Ensure we only do this once
    Serial.println(); // Add a blank line for readability
    Serial.println("Initial position and velocity reset.");
  }

  // Sample sensor data at a higher frequency (every 20ms)
  if (currentTime - lastSampleTime >= sampleInterval) {
      lastSampleTime = currentTime;
      
      // Read quaternion data from the BNO055
      imu::Quaternion quat = bno.getQuat();
      quatStruct = {quat.w(), quat.x(), quat.y(), quat.z()};
      
      // Read linAccel data
      imu::Vector<3> linAccelSample = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      LinearAcceleration linAccelRaw = {linAccelSample.x(), linAccelSample.y(), linAccelSample.z()};

      // Adjust raw linear acceleration data by removing the bias
      linAccStruct.linAccX = linAccelSample.x() - accXBias;
      linAccStruct.linAccY = linAccelSample.y() - accYBias;
      linAccStruct.linAccZ = linAccelSample.z() - accZBias;

      // Read grav data
      imu::Vector<3> gravityVec = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
      gravVecStruct = {gravityVec.x(), gravityVec.y(), gravityVec.z()};

      // Add sample to average accumulator
      accelerationSamples.push_back({linAccelSample.x() - accXBias, linAccelSample.y() - accYBias, linAccelSample.z() - accZBias});
      
      
      if(DEBUG) {
        // Print raw linear acceleration data
        Serial.print("Raw Linear Accel: X=");
        Serial.print(linAccelRaw.linAccX, 4);
        Serial.print(", Y=");
        Serial.print(linAccelRaw.linAccY, 4);
        Serial.print(", Z=");
        Serial.println(linAccelRaw.linAccZ, 4);
      }

      if(DEBUG) {
        // Print corrected and filtered linear acceleration data
        Serial.print("Corrected Linear Accel: X=");
        Serial.print(linAccStruct.linAccX, 4);
        Serial.print(", Y=");
        Serial.print(linAccStruct.linAccY, 4);
        Serial.print(", Z=");
        Serial.println(linAccStruct.linAccZ, 4);
      }
      

    /*
      Serial.println(); // Add a blank line for readability
      // Debugging: Print raw sensor data
      Serial.print("Raw Quaternion: w=");
      Serial.print(quatStruct.w, 4);
      Serial.print(", x=");
      Serial.print(quatStruct.x, 4);
      Serial.print(", y=");
      Serial.print(quatStruct.y, 4);
      Serial.print(", z=");
      Serial.println(quatStruct.z, 4);

      Serial.print("Raw Linear Accel: x=");
      Serial.print(linAccStruct.linAccX, 4);
      Serial.print(", y=");
      Serial.print(linAccStruct.linAccY, 4);
      Serial.print(", z=");
      Serial.println(linAccStruct.linAccZ, 4);

      Serial.println(); // Add a blank line for readability
      Serial.print("Raw Gravity Vec: x=");
      Serial.print(gravVecStruct.gravX, 4);
      Serial.print(", y=");
      Serial.print(gravVecStruct.gravY, 4);
      Serial.print(", z=");
      Serial.println(gravVecStruct.gravZ, 4);
      */
  }
  // Calculate and send position at a lower frequency (every 40ms)
  if (currentTime - lastPositionUpdateTime >= positionUpdateInterval) {
      lastPositionUpdateTime = currentTime;
      float deltaT = calculateDeltaTime();

      LinearAcceleration avgLinAcc = calculateAverageL(accelerationSamples);

      if(DEBUG_LITE) {
        // Print raw linear acceleration data
        Serial.print("AVG Linear Accel: X=");
        Serial.print(avgLinAcc.linAccX, 4);
        Serial.print(", Y=");
        Serial.print(avgLinAcc.linAccY, 4);
        Serial.print(", Z=");
        Serial.println(avgLinAcc.linAccZ, 4);
      }
      
      // Clear the vectors after averaging
      accelerationSamples.clear();
      
      updatePositionAndSend(quatStruct, avgLinAcc, gravVecStruct, deltaT);
  }

}