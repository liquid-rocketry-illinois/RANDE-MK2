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
            const uint8_t aoff;
            const float psi_per_v;
            float voffset;
            const float alpha = 0.001; // Filter value
        };

        // Array containint config data for each PT. Array index matches RCP id. aoff refers to the offset into the
        // `data` array down below that contains the data for that particular PT (i.e. which pin on the MCU the
        // PT is connected to). psi_per_v and voffset are the calibration values.
        // clang-format off
        TData transducers[NUM_TRANSDUCERS] = {
            { // PT1
                .aoff = 0,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT2
                .aoff = 0,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT3
                .aoff = 0,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT4
                .aoff = 0,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT5
                .aoff = 0,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT6
                .aoff = 0,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT7
                .aoff = 0,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT8
                .aoff = 0,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT9
                .aoff = 0,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT10
                .aoff = 9,
                .psi_per_v = 504.2437151f,
                .voffset = -200.0827676f
            }
        };
        // clang-format on

        // The unit converted, final PT values
        float ptdata[NUM_TRANSDUCERS] = {};

        // This array is written to automatically by the DMA handler for the ADCs. This array SHOULD NOT be modified
        // manually, as it is filled in the background. It is placed into the .ADC_RAW section placed at address
        // 0x30000000, and is protected from cache incoherency by the MPU
        [[gnu::section(".ADC_RAW")]] uint32_t data[NUM_TRANSDUCERS] = {};

        // Store time last auto-logged data over RCP
        uint32_t timeLastLogged = 0;
    } // namespace

    void init() {
        // Callibrate the ADCs
        HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_DIFFERENTIAL_ENDED);
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);

        // Start the DMA processing. Both ADCs are DMAd into the same array, just at different positions
        HAL_ADC_Start_DMA(&hadc1, data, 5);
        HAL_ADC_Start_DMA(&hadc2, data + 5, 5);

        RCPDebug("[TRANSDUCERS] Initialized");
    }

    void yield() {
        // For each PT:
        for(uint8_t i = 0; i < NUM_TRANSDUCERS; i++) {
            // Get the PT data from the data array
            const auto& td = transducers[i];
            // ptdata[i] = data[td.aoff];
            // continue;
            // Grab the value from the DMA data array and convert to volts
            float volts = static_cast<float>(data[td.aoff]) * VREF / static_cast<float>(UINT16_MAX);

            // Apply calibration values
            volts = (volts * td.psi_per_v) + td.voffset;

            // Apply simple filtering to smooth data
            ptdata[i] = td.alpha * volts + (ptdata[i] * (1 - td.alpha));
        }

        // Process if we should send back telemetry
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
