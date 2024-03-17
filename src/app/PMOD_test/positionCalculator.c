/*
 * File: positionCalculator.c
 * ---------------------------
 * This file implements the position calculation based on sensor data,
 * including quaternion orientation, linear acceleration, and gravity vector.
 * The implementation covers converting quaternion data to a rotation matrix,
 * applying the rotation to acceleration data, compensating for gravity,
 * and updating the object's position in 3D space.
 *
 * The core function, `get3Dposition`, updates the passed Position structure
 * with the latest calculated position, demonstrating a modular approach
 * to sensor data processing and 3D position tracking.
 * 
 * This file should be used in conjunction with `positionCalculator.h`.
 *
 * Author: Robert Wilcox
 * Version: 1.0
 * Date: 3.12.2024
 */
#include <stdio.h>
#include "positionCalculator.h"
#include "esp32_BLE_uart.h"

// Static global variables to maintain the current position and velocity of the object.
// These are initialized to zero
static Vector3 position = {0.0, 0.0, 0.0}; // Current position in 3D space.
static Vector3 velocity = {0.0, 0.0, 0.0}; // Current velocity in 3D space.
static const float delta_t = 0.01; // Time step in seconds, NEED TO UPDATE BASED ON ESP32 RATE

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
Vector3 rotateVector(Vector3 v, float R[3][3]) {
    Vector3 rotated; // The result of applying the rotation to v.
    // Each component of the rotated vector is a dot product of the row of the rotation matrix with the original vector.
    rotated.x = R[0][0]*v.x + R[0][1]*v.y + R[0][2]*v.z;
    rotated.y = R[1][0]*v.x + R[1][1]*v.y + R[1][2]*v.z;
    rotated.z = R[2][0]*v.x + R[2][1]*v.y + R[2][2]*v.z;
    return rotated; // Return the rotated vector.
}

// Function to update the object's position based on its current orientation, acceleration data, and the effect of gravity.
void updatePositionWithGravity(Quaternion sensorOrientation, Vector3 raw_acceleration, Vector3 sensorGravity) {

    float rotationMatrix[3][3]; // Matrix to hold the rotation derived from the quaternion.

    // Convert the quaternion to a rotation matrix, which can then be used to rotate the sensor data into our reference frame
    quaternionToRotationMatrix(sensorOrientation, rotationMatrix); 

    // Rotate the sensor-supplied gravity vector into the global frame
    Vector3 globalGravity = rotateVector(sensorGravity, rotationMatrix);

    // Rotate the raw acceleration vector to align it with the global frame as well
    Vector3 global_acceleration = rotateVector(raw_acceleration, rotationMatrix);
    
    // Now subtract the globally-aligned gravity vector from the globally-aligned acceleration
    // This gives the true acceleration in the global frame, compensating for gravity
    global_acceleration.x -= globalGravity.x;
    global_acceleration.y -= globalGravity.y;
    global_acceleration.z -= globalGravity.z;

    // Integrate the true global acceleration to update velocity and position
    velocity.x += global_acceleration.x * delta_t;
    velocity.y += global_acceleration.y * delta_t;
    velocity.z += global_acceleration.z * delta_t;

    position.x += velocity.x * delta_t;
    position.y += velocity.y * delta_t;
    position.z += velocity.z * delta_t;
}

// The primary function to be called from external code.
// It updates a passed Position structure with the latest 3D position calculated from sensor data.
void get3Dposition(Position* pos) {

    readData(); // Populate the structs

    // Extract the fetched data into local variables for ease of use
    Quaternion sensorOrientation = {qData.w, qData.x, qData.y, qData.z};
    Vector3 raw_acceleration = {lData.x, lData.y, lData.z};
    Vector3 sensorGravity = {gData.x, gData.y, gData.z};

    // Proceed with the position update logic as before
    updatePositionWithGravity(sensorOrientation, raw_acceleration, sensorGravity);

    // Update the passed Position struct with the new calculated position
    pos->xPos = position.x;
    pos->yPos = position.y;
    pos->zPos = position.z;
}


