/**
 * @file sensorFusionBLE.cpp
 * @brief NOTE THIS FILE IS NOT FULLY FUNCTIONAL This file integrates the Adafruit BNO055 sensor with BLE functionality 
 *        on an ESP32 to perform sensor fusion and motion tracking. It utilizes the Eigen library for 
 *        linear algebra operations, particularly for implementing a Kalman filter for state estimation. The primary goal 
 *        is to estimate 3D position and velocity from sensor data, including acceleration and orientation, and transmit 
 *        these estimates over BLE.
 * 
 *        The code sets up the necessary BLE service and characteristics for data transmission and control. It includes
 *        definitions for processing quaternion data from the BNO055 to calculate orientation and uses this, along with
 *        acceleration data, to estimate motion states. The Kalman filter processes the sensor data to provide more
 *        accurate and stable estimates of position and velocity.
 * 
 *        A series of structures and global variables are defined to hold sensor data, state estimates, and Kalman filter
 *        matrices. Functions are provided for initializing the Kalman filter matrices, converting quaternion data to
 *        rotation matrices, applying low-pass filters to sensor data, and updating the position and velocity estimates.
 *        BLE characteristics callbacks handle incoming data for control commands.
 * 
 *        This file demonstrates advanced techniques for sensor fusion, state estimation, and wireless data transmission,
 *        applicable in contexts such as robotics, virtual reality tracking, and more.
 * 
 * @note Ensure that the Adafruit BNO055 sensor is properly connected and configured. Replace the UUID placeholders with
 *       your specific BLE service and characteristic UUIDs. Adjust the SDA and SCL pin assignments as per your hardware
 *       setup. The Eigen library is required for the matrix and vector operations involved in the Kalman filter
 *       implementation.
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <vector>
#include <ArduinoEigenDense.h>

using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29); // BNO055 uses address 0x28 by default, adjust if necessary

// BLE Service and Characteristic UUIDs - replace these with your generated UUIDs
#define SERVICE_UUID        "DF816183-E1F1-4BF9-8B7D-3B794FA03C03"
#define CHARACTERISTIC_UUID_TX "AE40D235-6AA7-4764-89AB-108562631979" // TX for transmitting data
#define CHARACTERISTIC_UUID_RX "12345678-1234-5678-1234-56789abcdef0" // RX for receiving data

#define SDA_PIN 23
#define SCL_PIN 22

const int STATE_SIZE = 6;          // Position (x, y, z) and velocity (vx, vy, vz) in 3D space
const int CONTROL_SIZE = 0;        // No control input
const int MEASUREMENT_SIZE = 3;    // Position measurements (x, y, z) in 3D space

Eigen::MatrixXd A(STATE_SIZE, STATE_SIZE);
Eigen::MatrixXd B(STATE_SIZE, CONTROL_SIZE);
Eigen::MatrixXd Q(STATE_SIZE, STATE_SIZE);
Eigen::MatrixXd R(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
Eigen::MatrixXd H(MEASUREMENT_SIZE, STATE_SIZE);

// Structs to hold the sensor data
struct myQuaternion {
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

// Generic vector struct
struct myVector3 {
    float x, y, z;
};

// Global variables for position and velocity
myVector3 position = {0, 0, 0};
myVector3 velocity = {0, 0, 0};

std::vector<LinearAcceleration> accelerationSamples;

unsigned long lastSampleTime = 0; // Tracks the last time sensor data was sampled
unsigned long lastPositionUpdateTime = 0; // Tracks last time the position was updated

const unsigned long sampleInterval = 10; // Sample data every 10ms
const unsigned long positionUpdateInterval = 50; // Calculate position every 50ms

const float stationaryThresholdX = 0.2;
const float stationaryThresholdY = 0.2;
const float stationaryThresholdZ = 0.2;

// Global variable for tracking time since last position update calculation
static unsigned long lastPositionCalculationTime = 0;

// Initialize myQuaternion, linear acceleration, and gravity vector structs globally
myQuaternion quatStruct = {0.0, 0.0, 0.0, 0.0};
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
bool DEBUG_LITE = false;

bool isFirstMeasurement = true;
bool isFirstPositionUpdate = true;

unsigned long stationaryStartTime = 0; // Tracks when the sensor first became stationary
bool stationaryRESET = false;

BLECharacteristic *pCharacteristicTX; // For sending data
BLECharacteristic *pCharacteristicRX; // For receiving data

void initializeKalmanMatrices();

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

class KalmanFilter {
public:
    VectorXd x; // State vector
    MatrixXd A; // State transition matrix
    MatrixXd B; // Control input matrix
    MatrixXd P; // Estimate error covariance
    MatrixXd Q; // Process noise covariance
    MatrixXd R; // Measurement noise covariance
    MatrixXd H; // Measurement matrix

    KalmanFilter(int stateSize, int controlSize, int measurementSize) {
        // Initialize state vector
        x = VectorXd::Zero(stateSize);

        double deltaT = calculateDeltaTimeD();

        // Initialize matrices with appropriate sizes
        A = MatrixXd::Identity(stateSize, stateSize);
        B = MatrixXd::Zero(stateSize, controlSize);
        P = MatrixXd::Identity(stateSize, stateSize) * 1000; // Start with high uncertainty
        Q = MatrixXd::Identity(stateSize, stateSize);
        R = MatrixXd::Identity(measurementSize, measurementSize);
        H = MatrixXd::Zero(measurementSize, stateSize);


        A(0, 1) = deltaT; // Position update includes velocity component
        A(2, 3) = deltaT;
        A(4, 5) = deltaT;
  
        // Modify A matrix to incorporate constant acceleration and jerk model
        for (int i = 0; i < stateSize / 3 - 1; ++i) {
            int row = i * 3;

            // Update position with velocity, acceleration, and jerk components
            A(row, row + 1) = deltaT;                    // Update position with velocity
            A(row, row + 2) = 0.5 * deltaT * deltaT;     // Update position with acceleration
            A(row, row + 3) = (1.0 / 6.0) * deltaT * deltaT * deltaT; // Update position with jerk

            // Update velocity with acceleration and jerk components
            A(row + 1, row + 2) = deltaT;                // Update velocity with acceleration
            A(row + 1, row + 3) = 0.5 * deltaT * deltaT; // Update velocity with jerk

            // Update acceleration with jerk component
            A(row + 2, row + 3) = deltaT;                // Update acceleration with jerk
        }

        // Adjust Q, R based on system's characteristics
        // Q: Process noise. Increase for more uncertainty in model
        // R: Measurement noise. Increase for more uncertainty in measurements
    }

    void setTransitionMatrix(const MatrixXd& newA) {
        A = newA;
    }

    void setControlInputModel(const MatrixXd& newB) {
        B = newB;
    }

    void setProcessNoiseCovariance(const MatrixXd& newQ) {
        Q = newQ;
    }

    void setMeasurementNoiseCovariance(const MatrixXd& newR) {
        R = newR;
    }

    void setMeasurementModel(const MatrixXd& newH) {
        H = newH;
    }

    void predict() {
        x = A * x; // Predict the state
        P = A * P * A.transpose() + Q; // Predict estimate covariance
    }

    void update(const VectorXd &z) {
        MatrixXd S = H * P * H.transpose() + R; // Residual covariance
        MatrixXd K = P * H.transpose() * S.inverse(); // Kalman gain
        
        VectorXd y = z - H * x; // Measurement pre-fit residual
        x = x + K * y; // Update the state
        
        int size = x.size();
        MatrixXd I = MatrixXd::Identity(size, size);
        P = (I - K * H) * P; // Update estimate covariance
    }
private:
    // Function to calculate deltaT in seconds as a double
    double calculateDeltaTimeD() {
          static bool isFirstCalculation = true;
          static unsigned long lastCalculationTime = 0;

          if (isFirstCalculation) {
              isFirstCalculation = false;
              lastCalculationTime = millis();
              return static_cast<double>(positionUpdateInterval) / 1000.0; // Return initial update interval
          } else {
              unsigned long currentTime = millis();
              double deltaT = (currentTime - lastCalculationTime) / 1000.0; // Convert milliseconds to seconds
              lastCalculationTime = currentTime; // Update the last position calculation time
              return deltaT;
          }
      }
};

KalmanFilter kf(STATE_SIZE, CONTROL_SIZE, MEASUREMENT_SIZE);

// Checks the sensor calibration and prompts to calibrate if necessary. To calibrate
// move the sensor through 6 axes of motion, smoothly, pausing briefly at each new axis.
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

// Function to collect and store average of measurements at rest to use as bias
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
      //Serial.println();
      Serial.print("Sample of Linear Accel: X=");
      Serial.print(linAccel.x(), 4);
      Serial.print(", Y=");
      Serial.print(linAccel.y(), 4);
      Serial.print(", Z=");
      Serial.println(linAccel.z(), 4);
     // Serial.println();
    }

    delay(20); // Short delay between samples
  }

  accXBias = sumX / samples;
  accYBias = sumY / samples;
  accZBias = sumZ / samples;

  // Print calculated biases
 // Serial.println();
  Serial.print("Calculated Biases - X: ");
  Serial.print(accXBias, 4);
  Serial.print(", Y: ");
  Serial.print(accYBias, 4);
  Serial.print(", Z: ");
  Serial.println(accZBias, 4);
 // Serial.println();
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

  initializeKalmanMatrices(); // Call the initialization function

  // Initialize Kalman filter with the matrices
  //kf.setTransitionMatrix(A);
  kf.setControlInputModel(B);
  kf.setProcessNoiseCovariance(Q);
  kf.setMeasurementNoiseCovariance(R);
  kf.setMeasurementModel(H);

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

void updatePositionAndSend(myQuaternion quat, LinearAcceleration linAcc, GravityVector gravity, float deltaT) {

  //Serial.println(); // Add a blank line for readability

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
  LinearAcceleration filteredAccel = applyLowPassFilter(adjustedLinAcc, prevFilteredAcc, 0.01);

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

  // Determine if the sensor is currently stationary
  bool currentlyStationary = fabs(filteredAccel.linAccX) < stationaryThresholdX &&
                             fabs(filteredAccel.linAccY) < stationaryThresholdY &&
                             fabs(filteredAccel.linAccZ) < stationaryThresholdZ;


  // Check if the sensor has just become stationary
  if (currentlyStationary && !stationaryRESET) {
      stationaryRESET = true;
      stationaryStartTime = millis(); // Record the time it became stationary
  } else if (!currentlyStationary) {
      stationaryRESET = false;
  }

  // Check if the sensor has been stationary for more than 10 seconds
  if (stationaryRESET && (millis() - stationaryStartTime) > 10000) {
      // Reset position and velocity
      position = {0, 0, 0};
      velocity = {0, 0, 0};

      // Reset the stationary start time to avoid repeated resets
      stationaryStartTime = millis();

      if (DEBUG) {
          Serial.println("Sensor has been stationary for more than 10 seconds. Resetting position.");
      }
  }

  // Estimate change in position and velocity from filtered acceleration

  float velX = filteredAccel.linAccX * deltaT;
  float velY = filteredAccel.linAccY * deltaT;
  float velZ = filteredAccel.linAccZ * deltaT;
  float deltaX = 0.5 * filteredAccel.linAccX * deltaT * deltaT;
  float deltaY = 0.5 * filteredAccel.linAccY * deltaT * deltaT;
  float deltaZ = 0.5 * filteredAccel.linAccZ * deltaT * deltaT;

  // Apply stationary checks to modify the estimated changes if needed
  if (fabs(filteredAccel.linAccX) < stationaryThresholdX) {
      deltaX = 0;
      velX = 0;

  }
  if (fabs(filteredAccel.linAccY) < stationaryThresholdY) {
      deltaY = 0;
      velY = 0;
  }

  if (fabs(filteredAccel.linAccZ) < stationaryThresholdZ) {
      deltaZ = 0;
      velZ = 0;
  }
  
  // Extract and apply the updated state from the Kalman filter
  velocity.x = velX;
  velocity.y = velY;
  velocity.z = velZ;

  position.x += deltaX;
  position.y += deltaY;
  position.z += deltaZ;


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

// Function to calculate deltaT in seconds as a float
float calculateDeltaTimeF() {
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
void quaternionToRotationMatrix(myQuaternion q, float R[3][3]) {
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

myQuaternion calculateAverageQ(const std::vector<myQuaternion>& samples) {
    myQuaternion avg = {0, 0, 0, 0};
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
   // Serial.println(); // Add a blank line for readability
    Serial.println("Initial position and velocity reset.");
  }

  VectorXd u = VectorXd::Zero(CONTROL_SIZE); // No control input in this case
  kf.predict();

  // Sample sensor data at a higher frequency (every 20ms)
  if (currentTime - lastSampleTime >= sampleInterval) {
      lastSampleTime = currentTime;
      
      sensors_event_t event;

      // Read myQuaternion data from the BNO055
      imu::Quaternion quat = bno.getQuat();
      quatStruct = {quat.w(), quat.x(), quat.y(), quat.z()};
      
      // Read linAccel data
      bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);
      LinearAcceleration linAccelRaw = {event.acceleration.x, event.acceleration.y, event.acceleration.z};

      // Adjust raw linear acceleration data by removing the bias
      linAccStruct.linAccX = event.acceleration.x - accXBias;
      linAccStruct.linAccY = event.acceleration.y - accYBias;
      linAccStruct.linAccZ = event.acceleration.z - accZBias;

      // Read grav data
      bno.getEvent(&event, Adafruit_BNO055::VECTOR_GRAVITY);
      gravVecStruct = {event.acceleration.x, event.acceleration.y, event.acceleration.z};

      // Add sample to average accumulator
      accelerationSamples.push_back({event.acceleration.x - accXBias, event.acceleration.y - accYBias, event.acceleration.z - accZBias});
      
      
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
  }
  // Calculate and send position at a lower frequency (every 40ms)
  if (currentTime - lastPositionUpdateTime >= positionUpdateInterval) {
      lastPositionUpdateTime = currentTime;
      float deltaT = calculateDeltaTimeF();

     // LinearAcceleration avgLinAcc = calculateAverageL(accelerationSamples);

      if(DEBUG_LITE) {
        // Print raw linear acceleration data
        Serial.print("Instantaneous Linear Accel: X=");
        Serial.print(linAccStruct.linAccX, 4);
        Serial.print(", Y=");
        Serial.print(linAccStruct.linAccY, 4);
        Serial.print(", Z=");
        Serial.println(linAccStruct.linAccZ, 4);
      }
      
      // Clear the vectors after averaging
      accelerationSamples.clear();
      
      updatePositionAndSend(quatStruct, linAccStruct, gravVecStruct, deltaT);
  }

}
void initializeKalmanMatrices() {
  // Now inside a function, we can use the Eigen comma-initializer syntax
  A << 1, 0, 0, 1, 0, 0,
       0, 1, 0, 0, 1, 0,
       0, 0, 1, 0, 0, 1,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 1;
       
       
Q << 50, 0,   0,   0,   0,   0,   
     0,   50, 0,   0,   0,   0,   
     0,   0,   50,   0,   0,   0,   
     0,   0,   0,   50,   0,   0,  
     0,   0,   0,   0,   50,  0,   
     0,   0,   0,   0,   0,   100;  
       
  R << 0.1, 0, 0,
       0, 0.1, 0,
       0, 0, 0.1;
       
  H << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 1, 0, 0, 0;
}
