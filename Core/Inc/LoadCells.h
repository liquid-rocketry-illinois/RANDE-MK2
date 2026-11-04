#ifndef RANDE_MK2_LOADCELLS_H
#define RANDE_MK2_LOADCELLS_H

#include <cstdint>
#include "stm32h7xx.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_ll_exti.h"

namespace  CellsNew::AdvancedControlTimers {
    struct ChannelCell {
        // pulse count since initialization and timer output channel
        int pulseCount = 0;
        uint32_t timch;

        // clock port and pin
        GPIO_TypeDef* clkport;
        uint16_t clkpin;

        // d_in port and pin
        GPIO_TypeDef* dinport;
        uint16_t dinpin;

        // output channel mask
        uint16_t CCER_CCNE;
        uint16_t CCER_CCNP;

        // data reading variables
        uint32_t inProgressReading = 0;
        uint32_t mask = 0x00800000;
        int32_t rawLatestReading = 0;

        //RCP variables
        float offset = 0;
        float scale = 0.0001532810009f;
        float latestReading = 12.34567f;
    };

    void yield();
    float readCell(uint8_t id);
    void tareCell(uint8_t id, float offset);
    namespace T1 {
        void initTimer(ChannelCell* cell);
        void conChannel(ChannelCell* cell);
        void disChannel(ChannelCell* cell);
    }
    namespace T2 {
        void initTimer(ChannelCell* cell);
        void conChannel(ChannelCell* cell);
        void disChannel(ChannelCell* cell);
    }
}

#endif // RANDE_MK2_LOADCELLS_H
