#include "Transducers.h"

#include "RCP_Target/RCP_Target.h"
#include "main.h"

namespace Transducers {
    namespace {
        struct TData {
            uint8_t aoff;
            const float psi_per_v;
            float voffset;
        };

        // clang-format off
        TData transducers[NUM_TRANSDUCERS] = {
            { // PT5
                .aoff = 4,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT6
                .aoff = 5,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT7
                .aoff = 6,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT8
                .aoff = 7,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT1
                .aoff = 0,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT2
                .aoff = 1,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT3
                .aoff = 2,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT4
                .aoff = 3,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT9
                .aoff = 8,
                .psi_per_v = 1,
                .voffset = 0
            },
            { // PT10
                .aoff = 9,
                .psi_per_v = 1,
                .voffset = 0
            }
        };
        // clang-format on

        [[gnu::section(".ADC_RAW")]] uint32_t data[NUM_TRANSDUCERS] = {};
        uint32_t timeLastLogged = 0;
    } // namespace

    void init() {
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
        float volts = static_cast<float>(data[td.aoff]) * (VREF / static_cast<float>(UINT16_MAX));
        return (volts * td.psi_per_v) + td.voffset;
    }

    void tare(uint8_t id, float offset) { transducers[id].voffset += offset; }
}
