#include "Transducers.h"

#include "RCP_Target/RCP_Target.h"
#include "main.h"

/*
(0, 0): PA6/7, IN3 N/P, UNASSIGNED, R2, AO1
(0, 1): PB0/1, IN5 N/P, UNASSIGNED, R4, AO3
(1, 0): PF11/12, IN2 N/P, UNASSIGNED, R1, AO0
(1, 1): PC4/5, IN4 N/P, UNASSIGNED, R3, AO2

(2, 0): PA4, IN18, PT6, R3, AO6
(2, 1): PA2, IN14, PT3, R6, AO9
(3, 0): PA3, IN15, PT1, R5, AO8
(3, 1): PA5, IN19, UNASSIGNED, R4, AO7



(3, 2): PF13, IN2, PT5, R1, AO4
(4, 2): PF14, IN6, PT10 (calib), R2, AO5
*/


namespace Transducers {
    namespace {
        struct TData {
            uint8_t aoff;
            bool ainmode; // true = single ended, false = diff
            const float psi_per_v;
            float voffset;
        };

        // clang-format off
        TData transducers[NUM_TRANSDUCERS] = {
            { // PT5
                .aoff = 4,
                .ainmode = true,
                // .psi_per_v = 636.9016584f,
                // .voffset = -244.0416529f
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT6
                .aoff = 6,
                .ainmode = true,
                // .psi_per_v = 859.1315104f,
                // .voffset = -8.316495865
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT7
                .aoff = 8,
                .ainmode = true,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT8
                .aoff = 7,
                .ainmode = true,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT1
                .aoff = 8,
                .ainmode = true,
                // .psi_per_v = 997.9170082f,
                // .voffset = -38.48342555
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT2
                .aoff = 3,
                .ainmode = false,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT3
                .aoff = 9,
                .ainmode = true,
                // .psi_per_v = 673.2834322f,
                // .voffset = -98.98520437f
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT4
                .aoff = 2,
                .ainmode = false,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT9
                .aoff = 4,
                .ainmode = true,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT10
                .aoff = 5,
                .ainmode = true,
                .psi_per_v = 503.598385527986f,
                .voffset = -197.81374604647237
            }
        };
        // clang-format on

        [[gnu::section(".ADC_RAW")]] uint32_t data[NUM_TRANSDUCERS] = {};
        uint32_t timeLastLogged = 0;
    } // namespace

    void init() {
        HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED);
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
        HAL_ADC_Start_DMA(&hadc1, data, 4);
        HAL_ADC_Start_DMA(&hadc2, data + 4, 6);

        RCPDebug("[TRANSDUCERS] Initialized");
    }

    void yield() {
        if(RCP::getDataStreaming() && HAL_GetTick() - timeLastLogged > 50) {
            timeLastLogged = HAL_GetTick();
            for(int i = 0; i < NUM_TRANSDUCERS; i++) {
                RCP::sendOneFloat(RCP_DEVCLASS_PRESSURE_TRANSDUCER, i, readTransducer(i));
            }
        }
    }

    float readTransducer(uint8_t id) {
        const auto& td = transducers[id];
        // return static_cast<float>(data[td.aoff]);
        float volts = 0;
        if(td.ainmode) volts = static_cast<float>(data[td.aoff]);
        else volts = static_cast<float>(data[td.aoff] - (UINT16_MAX / 2)) * 2;

        volts *= VREF / static_cast<float>(UINT16_MAX);
        return (volts * td.psi_per_v) + td.voffset;
    }

    void tare(uint8_t id, float offset) { transducers[id].voffset -= offset; }
} // namespace Transducers
