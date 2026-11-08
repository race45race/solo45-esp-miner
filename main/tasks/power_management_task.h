#ifndef POWER_MANAGEMENT_TASK_H_
#define POWER_MANAGEMENT_TASK_H_

typedef struct GlobalState GlobalState;

typedef struct
{
    float fan_perc;
    uint16_t fan_rpm;
    uint16_t fan2_rpm;
    float chip_temp_avg;
    float chip_temp2_avg;
    float vr_temp;
    float voltage;
    float frequency_value;
    float actual_frequency;    
    float expected_hashrate;
    float power;
    float current;
    float core_voltage;
    float throttle_mhz; // Solo45 heat throttle: MHz below the set frequency right now (0 = not throttling)
    // Solo45 auto-tuner: while tune_active, these RAM-only values replace the
    // saved frequency and voltage (and the heat throttle stands aside).
    bool tune_active;
    float tune_freq;
    uint16_t tune_mv;
} PowerManagementModule;

void POWER_MANAGEMENT_init_frequency(GlobalState * GLOBAL_STATE);

void POWER_MANAGEMENT_task(void * pvParameters);

#endif
