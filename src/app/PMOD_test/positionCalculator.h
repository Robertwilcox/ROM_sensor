/*
 * File: positionCalculator.h
 * ---------------------------
 * This header file defines the interface for the position calculator,
 * which includes functions for updating and retrieving the 3D position
 * based on quaternion orientation, linear acceleration, and gravity vector.
 * 
 * The core functionality involves converting quaternion orientation data
 * into a rotation matrix, rotating acceleration vectors into the world frame,
 * and then using this data to update the object's velocity and position in 3D space.
 *
 * This module provides an abstraction layer for position tracking, allowing
 * for easy integration into larger systems or applications requiring 3D spatial tracking.
 * 
 * Author: Robert Wilcox
 * Version: 1.0
 * Date: 3.12.2024
 */
#ifndef POSITION_TRACKER_H
#define POSITION_TRACKER_H

typedef struct {
    float w, x, y, z; // Quaternion orientation
} Quaternion;

typedef struct {
    float x, y, z; // Vector for acceleration, gravity, position, etc.
} Vector3;

typedef struct {
    float xPos, yPos, zPos; // Position struct for external use
} Position;

// Function prototypes
void get3Dposition(Position* pos);

#endif
