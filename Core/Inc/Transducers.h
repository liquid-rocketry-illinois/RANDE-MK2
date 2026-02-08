#ifndef RANDE_MK2_TRANSDUCERS_H
#define RANDE_MK2_TRANSDUCERS_H

#include <cstdint>

namespace Transducers {
    static constexpr uint8_t NUM_TRANSDUCERS = 10;
    static constexpr float VREF = 3.3f;

    void init();
    void yield();

    float readTransducer(uint8_t id);
    void tare(uint8_t id, float offset);
}

#endif // RANDE_MK2_TRANSDUCERS_H
