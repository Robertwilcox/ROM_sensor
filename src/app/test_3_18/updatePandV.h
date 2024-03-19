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

void getPositionAndValocity(DataPoint *dataPoint);
#endif // UPDATE_P_AND_V_h
