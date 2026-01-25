#ifndef RANDE_MK2_SIMPLEACTUATORS_H
#define RANDE_MK2_SIMPLEACTUATORS_H

#include "main.h"
#include <cstdint>

namespace SimpleActuators {
    constexpr uint8_t NUM_ACTS = 8;

    // These match the Solenoid numbers from the P&ID to the RCP IDs. The RCP ID is used as the index into
    // the state and pin assignment arrays
    constexpr uint8_t EMATCH_ID = 0;
    constexpr uint8_t SOL_1_id = 2;
    constexpr uint8_t SOL_2_id = 1;
    constexpr uint8_t SOL_3_id = 6;
    constexpr uint8_t SOL_4_id = 5;
    constexpr uint8_t SOL_7_id = 3;
    constexpr uint8_t SOL_9_id = 4;
    constexpr uint8_t SOL_10_id = 7;

    struct GPIO {
        GPIO_TypeDef* port;
        uint16_t pin;
    };

    void init();
}

#endif // RANDE_MK2_SIMPLEACTUATORS_H
