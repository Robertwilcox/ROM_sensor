/**
 * @file updataPandV.h
 * @brief header file for updatting global struct with position and velocity
 * data from packetReader.h
 *
 * @author Robert Wilcox, Ibrahim Binmahfood
 * @version 1.0
 * @date 3.21.2024
 */

#ifndef UPDATE_P_AND_V_H
#define UPDATE_P_AND_V_H

typedef struct {
	float x_pos;
	float y_pos;
	float z_pos;

	float x_veloc;
	float y_veloc;
	float z_veloc;
} DataPoint, *DataPointPtr;

void getPositionAndVelocity(DataPoint *dataPoint);
#endif // UPDATE_P_AND_V_h
