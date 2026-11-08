#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "global_state.h"
#include "nvs_config.h"
#include "vcore.h"
#include "thermal.h"
#include "power.h"
#include "asic.h"
#include "utils.h"
#include "asic_init.h"
#include "asic_reset.h"
#include "driver/uart.h"

#define POLL_RATE 100
#define MAX_TEMP 90.0
#define THROTTLE_TEMP 75.0
#define SAFE_TEMP 45.0

#define VOLTAGE_START_THROTTLE 4900
#define VOLTAGE_MIN_THROTTLE 3500
#define VOLTAGE_RANGE (VOLTAGE_START_THROTTLE - VOLTAGE_MIN_THROTTLE)

#define TPS546_THROTTLE_TEMP 105.0
#define TPS546_MAX_TEMP 145.0

#define ASIC_REDUCTION 100.0

// One bad sensor read must not trigger safe mode: the chip has to read hot for this long.
#define ASIC_OVERHEAT_CONFIRM_MS 3000
#define ASIC_OVERHEAT_CONFIRM_POLLS (ASIC_OVERHEAT_CONFIRM_MS / POLL_RATE)

// Solo45 heat throttle. When the auto fan is flat out and the chip is still
// over its target (or the regulator runs hot), run 25 MHz below the set
// frequency, one step at a time; step back up once it has been cool for a
// while. The frequency saved in NVS is never changed, so the user's setting
// comes back by itself. This works well below the 75 C / 105 C safe-mode trips.
#define THROTTLE_STEP_MHZ 25.0f
#define THROTTLE_MAX_STEPS 8                   // at most 200 MHz below the setting
#define THROTTLE_FLOOR_MHZ 400.0f              // never throttle below this
#define THROTTLE_FAN_FULL 95.0f                // fan % that counts as "nothing left"
#define THROTTLE_HOT_MARGIN 1.5f               // chip this far over target = hot
// The auto fan holds the chip at its target by turning down, so "cool" is
// read from the fan: plenty of fan left with the chip at or under target.
#define THROTTLE_COOL_MARGIN 1.0f              // chip at most this far over target...
#define THROTTLE_FAN_EASY 75.0f                // ...with the fan below this = cool
#define THROTTLE_VR_HOT 95.0f                  // regulator hot, whatever the fan does
#define THROTTLE_VR_COOL 85.0f
#define THROTTLE_DOWN_POLLS (45000 / POLL_RATE)  // ~45 s hot before each step down
#define THROTTLE_UP_POLLS (300000 / POLL_RATE)   // 5 min cool before each step up

static const char * TAG = "power_management";

static void mining_stop(GlobalState * GLOBAL_STATE)
{
    ESP_LOGI(TAG, "Stopping mining");

    // Wind frequency down to 50 MHz before cutting power. This also updates
    // the transition tracker so the ramp starts from 50 MHz on next start,
    // rather than the stale pre-reset frequency.
    GLOBAL_STATE->POWER_MANAGEMENT_MODULE.frequency_value = 50;
    GLOBAL_STATE->POWER_MANAGEMENT_MODULE.expected_hashrate = 0;

    ASIC_set_frequency(GLOBAL_STATE);
    ASIC_set_nonce_space(GLOBAL_STATE);

    // Cut ASIC power and hold in reset
    VCORE_set_voltage(GLOBAL_STATE, 0.0f);
    asic_hold_reset_low();

    // Mark uninitialized immediately so tasks stop issuing UART commands
    GLOBAL_STATE->ASIC_initalized = false;

    // Give tasks time to complete any in-progress UART operation
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Flush any stale data from the UART buffers
    uart_flush(UART_NUM_1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Mining stopped");
}

static uint8_t mining_start(GlobalState * GLOBAL_STATE)
{
    ESP_LOGI(TAG, "Starting mining");

    // Restore voltage from NVS
    uint16_t voltage = nvs_config_get_u16(NVS_CONFIG_ASIC_VOLTAGE);
    VCORE_set_voltage(GLOBAL_STATE, (double) voltage / 1000.0);

    // Wait for voltage to stabilize before touching the ASIC
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Clear any accumulated UART garbage before init
    uart_flush(UART_NUM_1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    POWER_MANAGEMENT_init_frequency(GLOBAL_STATE);
    // Stabilization delay of 2000ms prevents race conditions where tasks are
    // just starting to use the ASIC while power management tries to change frequency
    uint8_t chip_count = asic_initialize(GLOBAL_STATE, ASIC_INIT_RECOVERY, 2000);

    if (chip_count > 0) {
        ESP_LOGI(TAG, "Mining started successfully (%d chip(s))", chip_count);
    } else {
        ESP_LOGE(TAG, "Mining start failed - ASIC not detected");
    }

    return chip_count;
}

static float expected_hashrate(GlobalState * GLOBAL_STATE)
{
    return GLOBAL_STATE->POWER_MANAGEMENT_MODULE.frequency_value * GLOBAL_STATE->DEVICE_CONFIG.family.asic.small_core_count * GLOBAL_STATE->DEVICE_CONFIG.family.asic_count / 1000.0;
}

void POWER_MANAGEMENT_init_frequency(GlobalState * GLOBAL_STATE)
{
    float frequency = nvs_config_get_float(NVS_CONFIG_ASIC_FREQUENCY);

    GLOBAL_STATE->POWER_MANAGEMENT_MODULE.frequency_value = frequency;
    GLOBAL_STATE->POWER_MANAGEMENT_MODULE.actual_frequency = 50.0;
    GLOBAL_STATE->POWER_MANAGEMENT_MODULE.expected_hashrate = expected_hashrate(GLOBAL_STATE);
    
    char expected_hashrate_str[16] = {0};
    suffixString(GLOBAL_STATE->POWER_MANAGEMENT_MODULE.expected_hashrate * 1e6, expected_hashrate_str, sizeof(expected_hashrate_str), 0);
    ESP_LOGI(TAG, "ASIC Frequency: %g MHz, Expected hashrate: %sH/s", frequency, expected_hashrate_str);
}

void POWER_MANAGEMENT_task(void * pvParameters)
{
    ESP_LOGI(TAG, "Starting");

    GlobalState * GLOBAL_STATE = (GlobalState *) pvParameters;

    PowerManagementModule * power_management = &GLOBAL_STATE->POWER_MANAGEMENT_MODULE;
    SystemModule * sys_module = &GLOBAL_STATE->SYSTEM_MODULE;

    POWER_MANAGEMENT_init_frequency(GLOBAL_STATE);
    
    float last_asic_frequency = power_management->frequency_value;

    vTaskDelay(500 / portTICK_PERIOD_MS);
    uint16_t last_core_voltage = 0.0;

    uint16_t last_known_asic_voltage = 0;
    float last_known_asic_frequency = 0.0;
    bool is_paused = false;
    uint16_t asic_overheat_polls = 0;

    uint8_t throttle_steps = 0;
    uint16_t throttle_hot_polls = 0;
    uint16_t throttle_cool_polls = 0;
    float throttle_set_frequency = 0.0f; // the NVS frequency the steps were counted from
    power_management->throttle_mhz = 0.0f;

    while (1) {
        if (GLOBAL_STATE->SELF_TEST_MODULE.is_finished) {
            ESP_LOGI(TAG, "Stopped");
            vTaskDelete(NULL);
            return;
        }

        power_management->voltage = Power_get_input_voltage(GLOBAL_STATE);
        Power_get_output(GLOBAL_STATE, &power_management->power, &power_management->current);
        power_management->core_voltage = VCORE_get_voltage_mv(GLOBAL_STATE);

        power_management->chip_temp_avg = Thermal_get_chip_temp(GLOBAL_STATE);
        power_management->chip_temp2_avg = Thermal_get_chip_temp2(GLOBAL_STATE);

        power_management->vr_temp = Power_get_vreg_temp(GLOBAL_STATE);
        // User pause, hardware fault, or all pools unreachable
        bool wants_stop = sys_module->mining_paused || sys_module->hardware_fault || sys_module->pools_unavailable;
        if (wants_stop && !is_paused) {
            mining_stop(GLOBAL_STATE);
            is_paused = true;
        } else if (!wants_stop && is_paused) {
            mining_start(GLOBAL_STATE);
            // mining_start ran the chip at the full NVS frequency; note that,
            // so a throttle still in force is applied again below.
            last_asic_frequency = power_management->frequency_value;
            is_paused = false;
        }

        // If we've paused or have a hardware fault, skip doing anything else
        if (is_paused || sys_module->hardware_fault) {
            vTaskDelay(POLL_RATE / portTICK_PERIOD_MS);
            continue;
        }

        bool asic_overheat =
            power_management->chip_temp_avg > THROTTLE_TEMP
            || power_management->chip_temp2_avg > THROTTLE_TEMP;

        if (!asic_overheat) {
            if (asic_overheat_polls > 0) {
                ESP_LOGW(TAG, "Ignored a brief high chip reading (%u polls)", asic_overheat_polls);
            }
            asic_overheat_polls = 0;
        } else if (asic_overheat_polls < ASIC_OVERHEAT_CONFIRM_POLLS) {
            asic_overheat_polls++;
        }
        bool asic_overheat_confirmed = asic_overheat_polls >= ASIC_OVERHEAT_CONFIRM_POLLS;

        if ((power_management->vr_temp > TPS546_THROTTLE_TEMP || asic_overheat_confirmed) && (power_management->frequency_value > 50 || power_management->voltage > 1000)) {
            if (power_management->chip_temp2_avg > 0) {
                ESP_LOGE(TAG, "OVERHEAT! VR: %fC ASIC1: %fC ASIC2: %fC", power_management->vr_temp, power_management->chip_temp_avg, power_management->chip_temp2_avg);
            } else {
                ESP_LOGE(TAG, "OVERHEAT! VR: %fC ASIC: %fC", power_management->vr_temp, power_management->chip_temp_avg);
            }

            asic_overheat_polls = 0;
            last_known_asic_voltage = nvs_config_get_u16(NVS_CONFIG_ASIC_VOLTAGE);
            last_known_asic_frequency = nvs_config_get_float(NVS_CONFIG_ASIC_FREQUENCY);
            nvs_config_set_bool(NVS_CONFIG_AUTO_FAN_SPEED, false);
            nvs_config_set_u16(NVS_CONFIG_MANUAL_FAN_SPEED, 100);
            nvs_config_set_bool(NVS_CONFIG_OVERHEAT_MODE, true);
            ESP_LOGW(TAG, "Entering safe mode due to overheat condition. System operation halted.");
            mining_stop(GLOBAL_STATE);
            
            // Note: ASIC temperature readings are invalid when ASIC is powered down (returns -1)
            // For 600-series boards that use ASIC thermal diode, we rely on VR temp and fixed cooling time
            // For boards with EMC internal temp sensor, readings remain valid
            bool asic_temp_valid = GLOBAL_STATE->DEVICE_CONFIG.emc_internal_temp;
            int cooling_cycles = 0;
            const int MIN_COOLING_CYCLES = 6; // Minimum 30 seconds cooling
            
            while (cooling_cycles < MIN_COOLING_CYCLES || power_management->vr_temp > TPS546_THROTTLE_TEMP - 10) {
                vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait 5 seconds
                cooling_cycles++;
                
                power_management->vr_temp = Power_get_vreg_temp(GLOBAL_STATE);
                
                // Only check ASIC temps if they're valid (not using ASIC thermal diode)
                if (asic_temp_valid) {
                    power_management->chip_temp_avg = Thermal_get_chip_temp(GLOBAL_STATE);
                    power_management->chip_temp2_avg = Thermal_get_chip_temp2(GLOBAL_STATE);
                    ESP_LOGW(TAG, "Safe mode active (cycle %d) - VR: %.1f°C ASIC1: %.1f°C ASIC2: %.1f°C",
                             cooling_cycles, power_management->vr_temp, power_management->chip_temp_avg, power_management->chip_temp2_avg);
                    
                    // Continue if ASIC temps still too high
                    if (power_management->chip_temp_avg >  SAFE_TEMP || power_management->chip_temp2_avg > SAFE_TEMP) {
                        cooling_cycles = 0; // Reset cycle count if still hot
                    }
                } else {
                    // For boards using ASIC thermal diode (600 series), rely on VR temp and time
                    ESP_LOGW(TAG, "Safe mode active (cycle %d/%d) - VR: %.1f°C (ASIC temps unavailable while powered down)",
                             cooling_cycles, MIN_COOLING_CYCLES, power_management->vr_temp);
                }
            }
            ESP_LOGI(TAG, "Temperature normalized after %d cooling cycles. Reinitializing ASIC...", cooling_cycles);
            
            uint16_t reduced_voltage = last_known_asic_voltage > ASIC_REDUCTION ? last_known_asic_voltage - ASIC_REDUCTION : 1000;
            float reduced_asic_frequency = last_known_asic_frequency > ASIC_REDUCTION ? last_known_asic_frequency - ASIC_REDUCTION : 400.0;
            
            nvs_config_set_u16(NVS_CONFIG_ASIC_VOLTAGE, reduced_voltage);
            nvs_config_set_float(NVS_CONFIG_ASIC_FREQUENCY, reduced_asic_frequency);
            
            ESP_LOGI(TAG, "Restoring at reduced settings: %umV (was %umV), %.0f MHz (was %.0f MHz)",
                     reduced_voltage, last_known_asic_voltage, reduced_asic_frequency, last_known_asic_frequency);

            uint8_t chip_count = mining_start(GLOBAL_STATE);
            last_asic_frequency = power_management->frequency_value;

            if (chip_count > 0) {
                // Frequency reduction will now be applied by normal power management loop
                nvs_config_set_bool(NVS_CONFIG_OVERHEAT_MODE, false);
                ESP_LOGI(TAG, "Resuming normal operation. Reduced frequency (%.0f MHz) will be applied automatically.", reduced_asic_frequency);
            }
        }

        uint16_t core_voltage = GLOBAL_STATE->SELF_TEST_MODULE.is_active
                                 ? GLOBAL_STATE->DEVICE_CONFIG.family.asic.default_voltage_mv
                                 : nvs_config_get_u16(NVS_CONFIG_ASIC_VOLTAGE);
        float asic_frequency = GLOBAL_STATE->SELF_TEST_MODULE.is_active
                                 ? GLOBAL_STATE-> DEVICE_CONFIG.family.asic.default_frequency_mhz
                                 : nvs_config_get_float(NVS_CONFIG_ASIC_FREQUENCY);

        // ---- Solo45 heat throttle ----
        if (asic_frequency != throttle_set_frequency) {
            // A new setting (from the user, or the safe-mode reduction) starts clean.
            throttle_set_frequency = asic_frequency;
            throttle_steps = 0;
            throttle_hot_polls = 0;
            throttle_cool_polls = 0;
        }
        bool auto_fan = nvs_config_get_bool(NVS_CONFIG_AUTO_FAN_SPEED);
        if (GLOBAL_STATE->SELF_TEST_MODULE.is_active || !auto_fan) {
            // Manual fan or self-test: the user (or the test) is in charge.
            if (throttle_steps > 0) {
                ESP_LOGI(TAG, "Heat throttle off (auto fan off or self-test)");
            }
            throttle_steps = 0;
            throttle_hot_polls = 0;
            throttle_cool_polls = 0;
        } else {
            float chip_max = power_management->chip_temp_avg > power_management->chip_temp2_avg
                                 ? power_management->chip_temp_avg
                                 : power_management->chip_temp2_avg;
            float target = (float) nvs_config_get_u16(NVS_CONFIG_TEMP_TARGET);
            bool valid = chip_max > 0;
            bool hot = valid && ((power_management->fan_perc >= THROTTLE_FAN_FULL && chip_max > target + THROTTLE_HOT_MARGIN)
                                 || power_management->vr_temp > THROTTLE_VR_HOT);
            bool cool = valid && chip_max <= target + THROTTLE_COOL_MARGIN
                        && power_management->fan_perc < THROTTLE_FAN_EASY
                        && power_management->vr_temp < THROTTLE_VR_COOL;

            if (hot) {
                throttle_cool_polls = 0;
                if (++throttle_hot_polls >= THROTTLE_DOWN_POLLS) {
                    throttle_hot_polls = 0;
                    float next = asic_frequency - (throttle_steps + 1) * THROTTLE_STEP_MHZ;
                    if (throttle_steps < THROTTLE_MAX_STEPS && next >= THROTTLE_FLOOR_MHZ) {
                        throttle_steps++;
                        ESP_LOGW(TAG, "Heat throttle: %.0f MHz (set %.0f). Chip %.1fC, target %.0fC, fan %.0f%%, VR %.1fC",
                                 next, asic_frequency, chip_max, target, power_management->fan_perc, power_management->vr_temp);
                    }
                }
            } else {
                // A hot spell is not erased by one cool reading near the line.
                if (throttle_hot_polls > 0) {
                    throttle_hot_polls--;
                }
                if (cool && throttle_steps > 0) {
                    if (++throttle_cool_polls >= THROTTLE_UP_POLLS) {
                        throttle_cool_polls = 0;
                        throttle_steps--;
                        ESP_LOGI(TAG, "Heat throttle: back up to %.0f MHz (set %.0f). Chip %.1fC, fan %.0f%%",
                                 asic_frequency - throttle_steps * THROTTLE_STEP_MHZ, asic_frequency, chip_max, power_management->fan_perc);
                    }
                } else {
                    throttle_cool_polls = 0;
                }
            }
        }
        power_management->throttle_mhz = throttle_steps * THROTTLE_STEP_MHZ;
        asic_frequency -= power_management->throttle_mhz;
        // ---- end heat throttle ----

        if (core_voltage != last_core_voltage) {
            ESP_LOGI(TAG, "setting new vcore voltage to %umV", core_voltage);
            VCORE_set_voltage(GLOBAL_STATE, (double) core_voltage / 1000.0);
            last_core_voltage = core_voltage;
        }

        if (asic_frequency != last_asic_frequency) {
            ESP_LOGI(TAG, "New ASIC frequency requested: %g MHz (current: %g MHz)", asic_frequency, last_asic_frequency);
            
            power_management->frequency_value = asic_frequency;
            power_management->expected_hashrate = expected_hashrate(GLOBAL_STATE);

            ASIC_set_frequency(GLOBAL_STATE);
            ASIC_set_nonce_space(GLOBAL_STATE);
            
            last_asic_frequency = asic_frequency;
        }

        // Check for changing of overheat mode
        bool new_overheat_mode = nvs_config_get_bool(NVS_CONFIG_OVERHEAT_MODE);
        
        if (new_overheat_mode != sys_module->overheat_mode) {
            sys_module->overheat_mode = new_overheat_mode;
            ESP_LOGI(TAG, "Overheat mode updated to: %d", sys_module->overheat_mode);
        }

        VCORE_check_fault(GLOBAL_STATE);

        // looper:
        vTaskDelay(POLL_RATE / portTICK_PERIOD_MS);
    }
}
