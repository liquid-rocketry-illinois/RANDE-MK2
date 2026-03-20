#ifndef RANDE_MK2_LOADCELLS_H
#define RANDE_MK2_LOADCELLS_H

#include <cstdint>

namespace LoadCells {
    void init();
    void yield();
    float readCell(uint8_t id);
    void tareCell(uint8_t id, float offset);
}

#endif // RANDE_MK2_LOADCELLS_H
