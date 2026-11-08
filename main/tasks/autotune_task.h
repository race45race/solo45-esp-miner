#ifndef AUTOTUNE_TASK_H_
#define AUTOTUNE_TASK_H_

#include "cJSON.h"

// Solo45 auto-tuner. Started and stopped through the solo45TuneMode setting
// (0 = stop/idle, 1 = most hashrate within the power limit, 2 = best J/TH).
// Trial settings live only in RAM; the saved frequency and voltage change once,
// at the end, after the chosen setting has passed a 10-minute check.
void AUTOTUNE_task(void * pvParameters);

// Adds the "solo45Tune" status object to an /api/system/info reply.
void AUTOTUNE_add_json(cJSON * root);

#endif
