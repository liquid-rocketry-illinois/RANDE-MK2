#include "Transducers.h"

#include "RCP_Target/RCP_Target.h"
#include "main.h"

/*


PA2: R6, AO9
PA3: R5, AO8
PA4: R3, AO6
PA5: R4, AO7
PA6/7: R2, AO1
PB0/1: R4, AO3
PC4/5: R3, AO2
PF11/12: R1, AO0
PF13: R1, AO4
PF14: R2, AO5

(2, 1): PA3, PT2
(3, 0): PA5, PT5
(3, 1): PA4, PT4
(3, 2): PF13, PT1
(4, 2): PA2, PT10
(5, 2): PF14, PT3
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
                .aoff = 7,
                .ainmode = true,
                // Old Vals
                // .psi_per_v = 758.686353,
                // .voffset = -120.6436201

                // Recalced
                .psi_per_v = 703.5993588f,
                .voffset = -3.753616816f
                // .psi_per_v = 1,
                // .voffset = 0
            },
            { // PT6
                .aoff = 6,
                .ainmode = true,
                .psi_per_v = 695.0840794f,
                .voffset = -4.774080473f
            },
            { // PT7
                .aoff = 8,
                .ainmode = true,
                .psi_per_v = 686.9490949f,
                .voffset = -2.361636292f
            },
            { // PT8
                .aoff = 1,
                .ainmode = false,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT1
                .aoff = 4,
                .ainmode = true,
                .psi_per_v = 594.6966303f,
                .voffset = -238.7113868f
            },
            { // PT2
                .aoff = 8,
                .ainmode = true,
                .psi_per_v = 686.9490949f,
                .voffset = -2.361636292f
                // .psi_per_v = 1,
                // .voffset = 0
            },
            { // PT3
                .aoff = 5,
                .ainmode = true,
                .psi_per_v = 589.6234427f,
                .voffset = -231.3814336f
            },
            { // PT4
                .aoff = 6,
                .ainmode = true,
                // .psi_per_v = 1,
                // .voffset = 0
                .psi_per_v = 695.0840794f,
                .voffset = -4.774080473f
            },
            { // PT9
                .aoff = 4,
                .ainmode = true,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT10
                .aoff = 9,
                .ainmode = true,
                // Old vals
                // .psi_per_v = 503.598385527986f,
                // .voffset = -197.81374604647237

                // Recalculated vals
                .psi_per_v = 504.2437151f,
                .voffset = -200.0827676f
            }
        };
        // clang-format on
        float ptdata[NUM_TRANSDUCERS] = {};

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

    float alpha = 0.001;

    void yield() {
        for(uint8_t i = 0; i < NUM_TRANSDUCERS; i++) {
            const auto& td = transducers[i];
            // ptdata[i] = data[td.aoff];
            // continue;
            float volts = 0;
            if(td.ainmode) volts = static_cast<float>(data[td.aoff]);
            else volts = static_cast<float>(data[td.aoff] - (UINT16_MAX / 2)) * 2;

            volts *= VREF / static_cast<float>(UINT16_MAX);
            volts = (volts * td.psi_per_v) + td.voffset;
            ptdata[i] = alpha * volts + (ptdata[i] * (1 - alpha));
        }
        if(RCP::getDataStreaming() && HAL_GetTick() - timeLastLogged > 10) {
            timeLastLogged = HAL_GetTick();
            for(int i = 0; i < NUM_TRANSDUCERS; i++) {
                RCP::sendOneFloat(RCP_DEVCLASS_PRESSURE_TRANSDUCER, i, readTransducer(i));
            }
        }
    }

    float readTransducer(uint8_t id) { return ptdata[id]; }

    void tare(uint8_t id, float offset) { transducers[id].voffset -= offset; }
} // namespace Transducers
