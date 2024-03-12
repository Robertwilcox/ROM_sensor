#include "WarmUpSet.h"

int WarmUpSet_init(WarmUpSet* inst_ptr) {
	inst_ptr->gyroOffset_x = 0;
	inst_ptr->gyroOffset_y = 0;
	inst_ptr->gyroOffset_z = 0;
	inst_ptr->rep_time     = 0.0;
	inst_ptr->reps         = 0;
	inst_ptr->stop_cntr    = 0;


	return XST_SUCCESS;
}

int WarmUpSet_isMove(WarmUpSet* inst_ptr) {

}

void WarmUpSet_getAvgPositions(WarmUpSet* inst_ptr) {

}

void WarmUpSet_LogRep(WarmUpSet* inst_ptr) {
	TickType_t prev_tick = xTaskGetTickCount();

	while (inst_ptr->stop_cntr != 4) {
		if (!(WarmUpSet_isMove(inst_ptr))) {
			WarmUpSet_getAvgPositions(inst_ptr);
			inst_ptr->reps++;
			inst_ptr->stop_cntr++;
		}
	}

	TickType_t curr_tick = xTaskGetTickCount();
	inst_ptr->rep_time = (curr_tick - prev_tick) / ((float)configTICK_RATE_HZ);
}
