#ifndef RANDE_MK2_THERMOCOUPLES_H
#define RANDE_MK2_THERMOCOUPLES_H

#include <cstdint>

namespace TC {
    static constexpr uint8_t NUM_TC = 8;

    void init();
    void yield();

    float readTC(uint8_t id);
}

#endif // RANDE_MK2_THERMOCOUPLES_H
