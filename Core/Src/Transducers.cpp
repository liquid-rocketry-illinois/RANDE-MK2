#include "Transducers.h"

#include "RCP_Target/RCP_Target.h"
#include "main.h"

/*

(-1, 0): PT10, PC4, A2
(0, 0): PT9, PF13, A5
(1, 0): PT8, PF12, A3
(2, 0): PT7, PF11, A0
(0, 1): PT6, PA7, A4
(1, 1): PT5, PA6, A1
(2, 1): PT4, PA5, A9
(0, 2): PT3, PA4, A8
(1, 2): PT2, PA3, A7
(2, 2): PT1, PA2, A6

*/


namespace Transducers {
    namespace {
        struct TData {
            const uint8_t aoff;
            const float psi_per_v;
            float voffset;
            const float alpha = 0.0001; // Filter value
        };

        // Array containint config data for each PT. Array index matches RCP id. aoff refers to the offset into the
        // `data` array down below that contains the data for that particular PT (i.e. which pin on the MCU the
        // PT is connected to). psi_per_v and voffset are the calibration values.
        // clang-format off
        TData transducers[NUM_TRANSDUCERS] = {
            { // PT1
                .aoff = 6,
                .psi_per_v = 1955.888606f,
                .voffset = 0
            },
            { // PT2
                .aoff = 7,
                .psi_per_v = 1844.895092f,
                .voffset = 0
            },
            { // PT3
                .aoff = 8,
                .psi_per_v = 623.1719912f,
                .voffset = 0,
                .alpha = 0.001
            },
            { // PT4
                .aoff = 9,
                .psi_per_v = 617.6467029f,
                .voffset = 0,
                .alpha = 0.001
            },
            { // PT5
                .aoff = 1,
                .psi_per_v = 1838.94488f,
                .voffset = 0
            },
            { // PT6
                .aoff = 4,
                .psi_per_v = 5132.8549,
                .voffset = 0
            },
            { // PT7
                .aoff = 0,
                .psi_per_v = 1822.684234f,
                .voffset = 0
            },
            { // PT8
                .aoff = 3,
                .psi_per_v = 1868.679613f,
                .voffset = 0
            },
            { // PT9
                .aoff = 5,
                .psi_per_v = 1871.12185f,
                .voffset = 0
            },
            { // PT10
                .aoff = 2,
                // .psi_per_v = 1,
                // .voffset = 0
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
