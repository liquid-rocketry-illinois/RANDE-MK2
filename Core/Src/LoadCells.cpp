#include "LoadCells.h"

#include "stm32h7xx.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_ll_exti.h"

#include "RCP_Target/RCP_Target.h"

#include <stdio.h>
#include "main.h"
#include <list>

namespace CellsNew {
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
        float latestReading = 17.12345f;
    };
    namespace AdvancedControlTimers {

        // ChannelCell Initializations
        ChannelCell C1{0, TIM_CHANNEL_1, GPIOC, GPIO_PIN_6, CELL1_DIN_GPIO_Port, CELL1_DIN_Pin, 0x1UL << (0U), 0x1UL << (1U)};
        ChannelCell C2{0, TIM_CHANNEL_2, GPIOE, GPIO_PIN_9, CELL2_DIN_GPIO_Port, CELL2_DIN_Pin, 0x1UL << (4U), 0x1UL << (5U)};
        ChannelCell C3{0, TIM_CHANNEL_3, GPIOB, GPIO_PIN_3, CELL3_DIN_GPIO_Port, CELL3_DIN_Pin, 0x1UL << (8U), 0x1UL << (9U)};
        ChannelCell C4{0, TIM_CHANNEL_1, GPIOB, GPIO_PIN_10, CELL4_DIN_GPIO_Port, CELL4_DIN_Pin, 0x1UL << (0U), 0x1UL << (1U)};
        ChannelCell C5{0, TIM_CHANNEL_2, GPIOD, GPIO_PIN_12, CELL5_DIN_GPIO_Port, CELL5_DIN_Pin, 0x1UL << (4U), 0x1UL << (5U)};
        ChannelCell C6{0, TIM_CHANNEL_3, GPIOC, GPIO_PIN_15, CELL6_DIN_GPIO_Port, CELL6_DIN_Pin, 0x1UL << (8U), 0x1UL << (9U)};

        // doubly linked list of active cells
        std::list<ChannelCell> static activeTIM1Cells;
        std::list<ChannelCell> static activeTIM2Cells;

        namespace T1 {
            TIM_TypeDef* const timer = TIM1;

            // initializes timer
            [[maybe_unused]] void initTimer(ChannelCell* const cell) {
                // enable clocks
                __HAL_RCC_TIM1_CLK_ENABLE();

                // Deinitializes peripheral registers and resets to default state
                HAL_GPIO_DeInit(cell->clkport, cell->clkpin);
                GPIO_InitTypeDef init;

                // Set clk to SCK output pin. Din configured by CubeMX (Jacob thinks interrupts are hard)
                init.Pin = (cell->clkpin);
                init.Mode = GPIO_MODE_AF_PP;
                init.Pull = GPIO_NOPULL;
                init.Alternate = GPIO_AF1_TIM1;
                HAL_GPIO_Init(cell->clkport, &init);

                // Disable timer
                timer->CR1 &= ~TIM_CR1_CEN;

                // Clears:
                // - ARPE: We do not want ARR preloading, we want to be able to load directly
                // - CMS: Put counter in up/down counting mode
                // - DIR: Put counter in upcounting mode
                timer->CR1 &= ~(TIM_CR1_ARPE | TIM_CR1_CMS | TIM_CR1_DIR);

                // Results in a timer clock of 10MHz (t=0.1micros)
                timer->PSC = (240000000 / 10000000) - 1;

                // 20 * 0.1micros = 2micro period (t3 + t4 in datasheet)
                timer->ARR = 20;

                // Reset counter
                timer->CNT = 0;

                // Enable the moe uwu
                timer->BDTR |= TIM_BDTR_MOE;

                // Clear CCER
                timer->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1P);
                timer->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2P);
                timer->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3P);

                // Set OC1M to PWM mode 1
                timer->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
                timer->CCMR2 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
                timer->CCMR3 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;

                // Set CCR1-3 to half of ARR, for 50% duty cycle
                timer->CCR1 = 10;
                timer->CCR2 = 10;
                timer->CCR3 = 10;

                // Set CCR4 as the time to actually read the GPIO data. The reading is triggered via a interrupt
                timer->CCR4 = 20;

                // Enable capture/compare and set polarity to make output normally low
                timer->CCER |= cell->CCER_CCNE | cell->CCER_CCNP | TIM_CCER_CC4E;

                // Cause an update so that all the fun stuff gets written to the shadow registers
                timer->EGR |= TIM_EGR_UG;

                // Enable capture interrupts
                timer->DIER |= TIM_DIER_UIE | TIM_DIER_CC4IE;

                // Enable capture interrupt
                HAL_NVIC_SetPriority(TIM8_CC_IRQn, 0, 0);
                HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);

                timer->CR1 |= TIM_CR1_CEN;
            }

            // connect output channel
            [[maybe_unused]] void conChannel(ChannelCell* const cell) {
                // Deinitializes peripheral registers and resets to default state
                HAL_GPIO_DeInit(cell->clkport, cell->clkpin);
                GPIO_InitTypeDef init;

                // Set clk to SCK output pin. Din configured by CubeMX (electrical jesus says interrupts r hard)
                init.Pin = (cell->clkpin);
                init.Mode = GPIO_MODE_AF_PP;
                init.Pull = GPIO_NOPULL;
                init.Alternate = GPIO_AF1_TIM1;
                HAL_GPIO_Init(cell->clkport, &init);

                // Enable capture/compare and set polarity to make output normally low
                timer->CCER |= cell->CCER_CCNE | cell->CCER_CCNP | TIM_CCER_CC4E;

                // Send it to shadow realm
                timer->EGR |= TIM_EGR_UG;
            }

            // disconnect output channel
            [[maybe_unused]] void disChannel(ChannelCell* const cell) {
                // Disable capture/compare
                timer->CCER |= ~(cell->CCER_CCNE) | TIM_CCER_CC4E;

                // Send it to shadow realm
                timer->EGR |= TIM_EGR_UG;
            }
        } // namespace T1
        namespace T2 {
            TIM_TypeDef* const timer = TIM2;

            // initializes timer
            [[maybe_unused]] void initTimer(ChannelCell* const cell) {
                // enable clocks
                __HAL_RCC_TIM2_CLK_ENABLE();

                // Deinitializes peripheral registers and resets to default state
                HAL_GPIO_DeInit(cell->clkport, cell->clkpin);
                GPIO_InitTypeDef init;

                // Set clk to SCK output pin. Din configured by CubeMX (Jacob thinks interrupts are hard)
                init.Pin = (cell->clkpin);
                init.Mode = GPIO_MODE_AF_PP;
                init.Pull = GPIO_NOPULL;
                init.Alternate = GPIO_AF1_TIM1;
                HAL_GPIO_Init(cell->clkport, &init);

                // Disable timer
                timer->CR1 &= ~TIM_CR1_CEN;

                // Clears:
                // - ARPE: We do not want ARR preloading, we want to be able to load directly
                // - CMS: Put counter in up/down counting mode
                // - DIR: Put counter in upcounting mode
                timer->CR1 &= ~(TIM_CR1_ARPE | TIM_CR1_CMS | TIM_CR1_DIR);

                // Results in a timer clock of 10MHz (t=0.1micros)
                timer->PSC = (240000000 / 10000000) - 1;

                // 20 * 0.1micros = 2micro period (t3 + t4 in datasheet)
                timer->ARR = 20;

                // Reset counter
                timer->CNT = 0;

                // Enable the moe uwu
                timer->BDTR |= TIM_BDTR_MOE;

                // Clear CCER
                timer->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1P);
                timer->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2P);
                timer->CCER &= ~(TIM_CCER_CC3E | TIM_CCER_CC3P);

                // Set OC1M to PWM mode 1
                timer->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
                timer->CCMR2 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
                timer->CCMR3 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;

                // Set CCR1-3 to half of ARR, for 50% duty cycle
                timer->CCR1 = 10;
                timer->CCR2 = 10;
                timer->CCR3 = 10;

                // Set CCR4 as the time to actually read the GPIO data. The reading is triggered via a interrupt
                timer->CCR4 = 20;

                // Enable capture/compare and set polarity to make output normally low
                timer->CCER |= cell->CCER_CCNE | cell->CCER_CCNP | TIM_CCER_CC4E;

                // Cause an update so that all the fun stuff gets written to the shadow registers
                timer->EGR |= TIM_EGR_UG;

                // Enable capture interrupts
                timer->DIER |= TIM_DIER_UIE | TIM_DIER_CC4IE;

                // Enable capture interrupt
                HAL_NVIC_SetPriority(TIM8_CC_IRQn, 0, 0);
                HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);

                timer->CR1 |= TIM_CR1_CEN;
            }

            // connect output channel
            [[maybe_unused]] void conChannel(ChannelCell* const cell) {
                // Deinitializes peripheral registers and resets to default state
                HAL_GPIO_DeInit(cell->clkport, cell->clkpin);
                GPIO_InitTypeDef init;

                // Set clk to SCK output pin. Din configured by CubeMX (electrical jesus says interrupts r hard)
                init.Pin = (cell->clkpin);
                init.Mode = GPIO_MODE_AF_PP;
                init.Pull = GPIO_NOPULL;
                init.Alternate = GPIO_AF1_TIM1;
                HAL_GPIO_Init(cell->clkport, &init);

                // Enable capture/compare and set polarity to make output normally low
                timer->CCER |= cell->CCER_CCNE | cell->CCER_CCNP | TIM_CCER_CC4E;

                // Send it to shadow realm
                timer->EGR |= TIM_EGR_UG;
            }

            // disconnect output channel
            [[maybe_unused]] void disChannel(ChannelCell* const cell) {
                // Disable capture/compare
                timer->CCER |= ~(cell->CCER_CCNE) | TIM_CCER_CC4E;

                // Send it to shadow realm
                timer->EGR |= TIM_EGR_UG;
            }
        } // namespace T2

        uint32_t lastLogged = 0;

        float readCell(uint8_t id) {
            if(id == 0) return C1.latestReading;
            if(id == 1) return C2.latestReading;
            if(id == 2) return C3.latestReading;
            if(id == 3) return C4.latestReading;
            if(id == 4) return C5.latestReading;
            if(id == 5) return C6.latestReading;
            return 0;
        }

        void yield() {
            C1.latestReading = C1.scale * (static_cast<float>(C1.rawLatestReading)) + C1.offset;
            C2.latestReading = C2.scale * (static_cast<float>(C2.rawLatestReading)) + C2.offset;
            C3.latestReading = C3.scale * (static_cast<float>(C3.rawLatestReading)) + C3.offset;
            C4.latestReading = C4.scale * (static_cast<float>(C4.rawLatestReading)) + C4.offset;
            C5.latestReading = C5.scale * (static_cast<float>(C5.rawLatestReading)) + C5.offset;
            C6.latestReading = C6.scale * (static_cast<float>(C6.rawLatestReading)) + C6.offset;

            if(RCP::getDataStreaming() && HAL_GetTick() - lastLogged > 10) {
                lastLogged = HAL_GetTick();
                RCP::sendOneFloat(RCP_DEVCLASS_LOAD_CELL, 0, readCell(0));
                RCP::sendOneFloat(RCP_DEVCLASS_LOAD_CELL, 1, readCell(1));
                RCP::sendOneFloat(RCP_DEVCLASS_LOAD_CELL, 1, readCell(2));
                RCP::sendOneFloat(RCP_DEVCLASS_LOAD_CELL, 1, readCell(3));
                RCP::sendOneFloat(RCP_DEVCLASS_LOAD_CELL, 1, readCell(4));
                RCP::sendOneFloat(RCP_DEVCLASS_LOAD_CELL, 1, readCell(5));
            }
        }

        void tareCell(uint8_t id, float offset) {
            if(id == 0) C1.offset += offset;
            else if(id == 1) C2.offset += offset;
            else if(id == 2) C3.offset += offset;
            else if(id == 3) C4.offset += offset;
            else if(id == 4) C5.offset += offset;
            else if(id == 5) C6.offset += offset;
        }
    } // namespace AdvancedControlTimers

    // global functions here

    // GPIO interrupts
    extern "C" void LC_DIN_IRQ(void) {
        using namespace AdvancedControlTimers;
        if(__HAL_GPIO_EXTI_GET_IT(C1.dinpin)) {
            // Acknowledge the interrupt
            __HAL_GPIO_EXTI_CLEAR_IT(C1.dinpin);

            // Disable DIN for GPIO
            LL_EXTI_DisableIT_0_31(C1.dinpin);

            // if active cells is empty initialize the timer
            if(activeTIM1Cells.empty()) {
                AdvancedControlTimers::T1::initTimer(&C1);
            }
            else { // if the timer is already running connect the channel for C1
                AdvancedControlTimers::T1::conChannel(&C1);
            }

            // add C1 to active cells
            activeTIM1Cells.push_back(C1);
        }
        else if(__HAL_GPIO_EXTI_GET_IT(C2.dinpin)) {
            // Acknowledge the interrupt
            __HAL_GPIO_EXTI_CLEAR_IT(C2.dinpin);

            // Disable DIN for GPIO
            LL_EXTI_DisableIT_0_31(C2.dinpin);

            // if active cells is empty initialize the timer
            if(activeTIM1Cells.empty()) {
                AdvancedControlTimers::T1::initTimer(&C2);
            }
            else { // if the timer is already running connect the channel for C2
                AdvancedControlTimers::T1::conChannel(&C2);
            }

            // add C2 to active cells
            activeTIM1Cells.push_back(C2);
        }
        else if(__HAL_GPIO_EXTI_GET_IT(C3.dinpin)) {
            // Acknowledge the interrupt
            __HAL_GPIO_EXTI_CLEAR_IT(C3.dinpin);

            // Disable DIN for GPIO
            LL_EXTI_DisableIT_0_31(C3.dinpin);

            // if active cells is empty initialize the timer
            if(activeTIM1Cells.empty()) {
                AdvancedControlTimers::T1::initTimer(&C3);
            }
            else { // if the timer is already running connect the channel for C3
                AdvancedControlTimers::T1::conChannel(&C3);
            }

            // add C3 to active cells
            activeTIM1Cells.push_back(C3);
        }
        else if(__HAL_GPIO_EXTI_GET_IT(C4.dinpin)) {
            // Acknowledge the interrupt
            __HAL_GPIO_EXTI_CLEAR_IT(C4.dinpin);

            // Disable DIN for GPIO
            LL_EXTI_DisableIT_0_31(C4.dinpin);

            // if active cells is empty initialize the timer
            if(activeTIM2Cells.empty()) {
                AdvancedControlTimers::T2::initTimer(&C4);
            }
            else { // if the timer is already running connect the channel for C4
                AdvancedControlTimers::T2::conChannel(&C4);
            }

            // add C4 to active cells
            activeTIM2Cells.push_back(C4);
        }
        else if(__HAL_GPIO_EXTI_GET_IT(C5.dinpin)) {
            // Acknowledge the interrupt
            __HAL_GPIO_EXTI_CLEAR_IT(C5.dinpin);

            // Disable DIN for GPIO
            LL_EXTI_DisableIT_0_31(C5.dinpin);

            // if active cells is empty initialize the timer
            if(activeTIM2Cells.empty()) {
                AdvancedControlTimers::T2::initTimer(&C5);
            }
            else { // if the timer is already running connect the channel for C5
                AdvancedControlTimers::T2::conChannel(&C5);
            }

            // add C5 to active cells
            activeTIM2Cells.push_back(C5);
        }
        else if(__HAL_GPIO_EXTI_GET_IT(C6.dinpin)) {
            // Acknowledge the interrupt
            __HAL_GPIO_EXTI_CLEAR_IT(C6.dinpin);\

            // Disable DIN for GPIO
            LL_EXTI_DisableIT_0_31(C6.dinpin);

            // if active cells is empty initialize the timer
            if(activeTIM2Cells.empty()) {
                AdvancedControlTimers::T2::initTimer(&C6);
            }
            else { // if the timer is already running connect the channel for C6
                AdvancedControlTimers::T2::conChannel(&C6);
            }

            // add C6 to active cells
            activeTIM2Cells.push_back(C6);
        }
    }

    // TIM8
    extern "C" void LC1_CC_IRQ(void) {
        using namespace AdvancedControlTimers;
        for (auto& cell : activeTIM2Cells) {

            // Acknowledge the interrupt
            T2::timer->SR &= ~TIM_SR_CC2IF;


            // If the mask is still has bits left...
            if(cell.mask > 0) {
                // If the data line is high, this bit is a one
                if(cell.dinport->IDR & cell.dinpin) cell.inProgressReading |= cell.mask;

                // Shift the mask each iteration
                cell.mask >>= 1;
            }

            if(cell.pulseCount == 25) {
                //disconnect cell channel from timer
                T2::disChannel(&cell);

                //do the type conversion and set latest reading to
                if(cell.inProgressReading & 0x00800000) cell.inProgressReading += 0xFF000000;
                cell.rawLatestReading = static_cast<int32_t>(cell.inProgressReading);

                // rest pulseCount to zero for next cycle
                cell.pulseCount = 0;

                // reenable dinpin
                LL_EXTI_EnableIT_0_31(cell.dinpin);
            }
        }
    }

    // TIM1
    extern "C" void LC2_CC_IRQ(void) {
        using namespace AdvancedControlTimers;
        for (auto& cell : activeTIM1Cells) {

            // Acknowledge the interrupt
            T1::timer->SR &= ~TIM_SR_CC2IF;


            // If the mask is still has bits left...
            if(cell.mask > 0) {
                // If the data line is high, this bit is a one
                if(cell.dinport->IDR & cell.dinpin) cell.inProgressReading |= cell.mask;

                // Shift the mask each iteration
                cell.mask >>= 1;
            }

            if(cell.pulseCount == 25) {
                //disconnect cell channel from timer
                T2::disChannel(&cell);

                //do the type conversion and set latest reading to
                if(cell.inProgressReading & 0x00800000) cell.inProgressReading += 0xFF000000;
                cell.rawLatestReading = static_cast<int32_t>(cell.inProgressReading);

                // rest pulseCount to zero for next cycle
                cell.pulseCount = 0;

                // reenable dinpin
                LL_EXTI_EnableIT_0_31(cell.dinpin);
            }
        }
    }

} // namespace CellsNew


































/*
namespace LoadCells {
    namespace {
        namespace C1 {
            TIM_TypeDef* const timer = TIM8;
            uint32_t const timch = TIM_CHANNEL_1;

            GPIO_TypeDef* const clkport = GPIOC;
            uint16_t const clkpin = GPIO_PIN_6;

            volatile GPIO_TypeDef* const dinport = CELL1_DIN_GPIO_Port;
            uint16_t const dinpin = CELL1_DIN_Pin;

            uint32_t inProgressReading = 0;
            uint32_t mask = 0;
            int32_t rawLatestReading = 0;

            float offset = 0;
            float scale = 0.0001532810009f;

            float latestReading = 17.12345f;

            [[maybe_unused]] void initCell() {
                // Enable Clocks
                __HAL_RCC_GPIOB_CLK_ENABLE();
                __HAL_RCC_GPIOC_CLK_ENABLE();
                __HAL_RCC_TIM8_CLK_ENABLE();

                HAL_GPIO_DeInit(clkport, clkpin);
                GPIO_InitTypeDef init;
                // Configure clock out pin (din is configured for us by cubemx (interrupts are hard)
                init.Pin = clkpin;
                init.Mode = GPIO_MODE_AF_PP;
                init.Pull = GPIO_NOPULL;
                init.Alternate = GPIO_AF3_TIM8;
                HAL_GPIO_Init(clkport, &init);

                // Disable timer
                timer->CR1 &= ~TIM_CR1_CEN;

                // Clears:
                // - ARPE: We do not want ARR preloading, we want to be able to load directly
                // - CMS: Put counter in up/down counting mode
                // - DIR: Put counter in upcounting mode
                // - Sets OPM: Counter stops after an update event (which occurs after RCR overflows of CNT)
                timer->CR1 &= ~(TIM_CR1_ARPE | TIM_CR1_CMS | TIM_CR1_DIR);
                timer->CR1 |= TIM_CR1_OPM;

                // Results in a timer clock of 10MHz (t=0.1micros)
                timer->PSC = (240000000 / 10000000) - 1;

                // 20 * 0.1micros = 2micro period (t3 + t4 in datasheet)
                timer->ARR = 20;

                // We need 25 pulses, so RCR = 25-1
                timer->RCR = 24;

                // Reset counter
                timer->CNT = 0;

                // Enable the moe uwu
                timer->BDTR |= TIM_BDTR_MOE;

                // Make sure we are still on channel 1, compile error if user changes this
                static_assert(timch == TIM_CHANNEL_1, "Change registers to change timer channels");

                // Clear CCER
                timer->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1P);

                // Set OC1M to PWM mode 1
                timer->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

                // Set CCR to half of ARR, for 50% duty cycle
                timer->CCR1 = 10;

                // Set CCR2 as the time to actually read the GPIO data. The reading is triggered via a interrupt
                timer->CCR2 = 20;

                // Enable capture/compare and set polarity to make output normally low
                timer->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC2E;

                // Cause an update so that all the fun stuff gets written to the shadow registers
                timer->EGR |= TIM_EGR_UG;

                // Enable capture interrupts
                timer->DIER |= TIM_DIER_UIE | TIM_DIER_CC2IE;

                // Enable capture interrupt
                HAL_NVIC_SetPriority(TIM8_CC_IRQn, 0, 0);
                HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);

                // Enable Update interrupt
                HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0, 0);
                HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
            }
        } // namespace C1


        namespace C2 {
            TIM_TypeDef* const timer = TIM1;
            uint32_t const timch = TIM_CHANNEL_1;

            GPIO_TypeDef* const clkport = GPIOE;
            uint16_t const clkpin = GPIO_PIN_9;

            GPIO_TypeDef* const dinport = CELL2_DIN_GPIO_Port;
            uint16_t const dinpin = CELL2_DIN_Pin;

            uint32_t inProgressReading = 0;
            int32_t rawLatestReading = 0;
            uint32_t mask = 0;

            float offset = 0;
            float scale = 0.0001528591418f;

            float latestReading = 0;

            [[maybe_unused]] void initCell() {
                __HAL_RCC_GPIOE_CLK_ENABLE();
                __HAL_RCC_GPIOD_CLK_ENABLE();
                __HAL_RCC_TIM1_CLK_ENABLE();

                HAL_GPIO_DeInit(GPIOE, GPIO_PIN_9);

                GPIO_InitTypeDef init;
                init.Pin = clkpin;
                init.Mode = GPIO_MODE_AF_PP;
                init.Pull = GPIO_NOPULL;
                init.Alternate = GPIO_AF1_TIM1;
                HAL_GPIO_Init(clkport, &init);

                timer->CR1 &= ~TIM_CR1_CEN;
                timer->CR1 &= ~(TIM_CR1_ARPE | TIM_CR1_CMS | TIM_CR1_DIR);
                timer->CR1 |= TIM_CR1_OPM;
                timer->PSC = (240000000 / 10000000) - 1;
                timer->ARR = 20;
                timer->RCR = 24;
                timer->CNT = 0;
                timer->BDTR |= TIM_BDTR_MOE;

                // Make sure we are still on channel 1, compile error if user changes this
                static_assert(timch == TIM_CHANNEL_1, "Change registers to change timer channels");

                timer->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1P);
                timer->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
                timer->CCR1 = 10;
                timer->CCR2 = 20;
                timer->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC2E;
                timer->EGR |= TIM_EGR_UG;
                timer->DIER |= TIM_DIER_UIE | TIM_DIER_CC2IE;

                HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
                HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
                HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
                HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
            }
        } // namespace C2
        uint32_t lastLogged = 0;
    } // namespace

    void init() {
        C1::initCell();
        C2::initCell();
    }

    void yield() {
        C1::latestReading = C1::scale * (static_cast<float>(C1::rawLatestReading)) + C1::offset;
        C2::latestReading = C2::scale * (static_cast<float>(C2::rawLatestReading)) + C2::offset;

        if(RCP::getDataStreaming() && HAL_GetTick() - lastLogged > 10) {
            lastLogged = HAL_GetTick();
            RCP::sendOneFloat(RCP_DEVCLASS_LOAD_CELL, 0, readCell(0));
            RCP::sendOneFloat(RCP_DEVCLASS_LOAD_CELL, 1, readCell(1));
        }
    }

    float readCell(uint8_t id) {
        if(id == 0) return C1::latestReading;
        if(id == 1) return C2::latestReading;
        return 0;
    }

    void tareCell(uint8_t id, float offset) {
        if(id == 0) C1::offset += offset;
        else if(id == 1) C2::offset += offset;
    }


} // namespace LoadCells
*/

/*
This interrupt is placed on DIN, and detects the falling edge indicating a conversion has completed. When this
    occurs, disable the interrupt so it is not constantly triggered when we are reading data. Reset the inputting
    variables, then start the one-shot 25-pulse PWM generation through the timer.
 */
/*
extern "C" void LC_DIN_IRQ(void) {
    using namespace LoadCells;
    // If the interrupt was on the data in pin
    if(__HAL_GPIO_EXTI_GET_IT(C1::dinpin)) {
        // Acknowledge the interrupt
        __HAL_GPIO_EXTI_CLEAR_IT(C1::dinpin);

        // Disable it so it is not retriggering
        LL_EXTI_DisableIT_0_31(C1::dinpin);

        // // Reset counters and stuff
        C1::inProgressReading = 0;
        C1::timer->CNT = 0;
        C1::mask = 0x00800000;

        // Start the PWM
        C1::timer->CR1 |= TIM_CR1_CEN;
    }

    else if(__HAL_GPIO_EXTI_GET_IT(C2::dinpin)) {
        __HAL_GPIO_EXTI_CLEAR_IT(C2::dinpin);
        LL_EXTI_DisableIT_0_31(C2::dinpin);

        // LL_EXTI_DisableIT_0_31(C1::dinpin);
        C2::inProgressReading = 0;
        C2::timer->CNT = 0;
        C2::mask = 0x00800000;
        C2::timer->CR1 |= TIM_CR1_CEN;
    }
}

// This interrupt detects when the 25 pulses have finished
extern "C" void LC1_UPDATE_IRQ(void) {
    using namespace LoadCells;
    // Acknowledge the interrupt
    C1::timer->SR &= ~TIM_SR_UIF;

    // Copy the raw reading over to the latest reading variable that can be used for calculating the measurement
    // This copy performs the sign extension for 2s complement 32-bit from unsigned 24-bit
    if(C1::inProgressReading & 0x00800000) C1::inProgressReading += 0xFF000000;
    C1::rawLatestReading = static_cast<int32_t>(C1::inProgressReading);

    // Re-enable the DIN interrupt
    LL_EXTI_EnableIT_0_31(C1::dinpin);
}

extern "C" void LC2_UPDATE_IRQ(void) {
    using namespace LoadCells;
    C2::timer->SR &= ~TIM_SR_UIF;
    if(C2::inProgressReading & 0x00800000) C2::inProgressReading += 0xFF000000;
    C2::rawLatestReading = static_cast<int32_t>(C2::inProgressReading);
    LL_EXTI_EnableIT_0_31(C2::dinpin);
}

// This interrupt is fired whenever the timers CCR and CNT registers are equal, aka at half way through a pulse. If
//     we are still shifting in data (in the first 24 pulses), read the GPIO and accumulate/shift the in progress
//     variable.
extern "C" void LC1_CC_IRQ(void) {
    using namespace LoadCells;
    // Acknowledge the interrupt
    C1::timer->SR &= ~TIM_SR_CC2IF;

    // If the mask is still has bits left...
    if(C1::mask > 0) {
        // If the data line is high, this bit is a one
        if(C1::dinport->IDR & C1::dinpin) C1::inProgressReading |= C1::mask;

        // Shift the mask each iteration
        C1::mask >>= 1;
    }
}

extern "C" void LC2_CC_IRQ(void) {
    using namespace LoadCells;
    C2::timer->SR &= ~TIM_SR_CC2IF;
    if(C2::mask > 0) {
        if(C2::dinport->IDR & C2::dinpin) C2::inProgressReading |= C2::mask;
        C2::mask >>= 1;
    }
}
*/