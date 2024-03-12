#ifndef __WARMUPSET_H
#define __WARMUPSET_H

typedef struct {
	int16_t   gyroOffset_x;
	int16_t   gyroOffset_y;
	int16_t   gyroOffset_z;
	uint8_t   reps;
	uint8_t   stop_cntr;
	float     rep_time;
} WarmUpSet, *WarmUpSetPtr;

int WarmUpSet_init(WarmUpSet* inst_ptr);

int WarmUpSet_isMove(WarmUpSet* inst_ptr);

void WarmUpSet_getAvgPositions(WarmUpSet* inst_ptr);

void WarmUpSet_LogRep(WarmUpSet* inst_ptr);

#endif //__WARMUPSET_H
