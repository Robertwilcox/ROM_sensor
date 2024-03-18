#ifndef __WORKSET_H
#define __WORKSET_H

typedef struct {
	int16_t   gyroOffset_x;
	int16_t   gyroOffset_y;
	int16_t   gyroOffset_z;
	uint8_t   reps;
	uint8_t   stop_cntr;
	float     rep_time;
} WorkSet, *WorkSetPtr;

void WorkSet_init(WorkSet* inst_ptr);

int WorkSet_isMove(WorkSet* inst_ptr);

void WorkSet_getAvgPositions(WorkSet* inst_ptr);

void WorkSet_LogRep(WorkSet* inst_ptr);

#endif //__WORKSET_H
