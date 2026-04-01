#ifndef RANDE_MK2_THERMOCOUPLES_H
#define RANDE_MK2_THERMOCOUPLES_H

#include <cstdint>

namespace TC {
    static constexpr uint8_t NUM_TC = 7;

    void init();
    void yield();

    float readTC(uint8_t id);
    void tareTC(uint8_t id, float offset);
}

#endif // RANDE_MK2_THERMOCOUPLES_H
