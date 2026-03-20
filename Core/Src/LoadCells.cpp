#include "LoadCells.h"

#include "stm32h7xx.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_ll_exti.h"

#include "RCP_Target/RCP_Target.h"

#include "main.h"

namespace LoadCells {
    namespace {
        namespace C1 {
            TIM_TypeDef* const timer = TIM8;
            uint32_t const timch = TIM_CHANNEL_1;

            GPIO_TypeDef* const clkport = GPIOC;
            uint16_t const clkpin = GPIO_PIN_6;

            GPIO_TypeDef* const dinport = CELL1_DIN_GPIO_Port;
            uint16_t const dinpin = CELL1_DIN_Pin;

            uint32_t inProgressReading = 0;
            int32_t rawLatestReading = 0;
            uint32_t pulseCount = 0;

            float offset = 0;
            float scale = 0;

            float latestReading;

            void initCell() {
                // Enable Clocks
                __HAL_RCC_GPIOB_CLK_ENABLE();
                __HAL_RCC_GPIOC_CLK_ENABLE();
                __HAL_RCC_TIM8_CLK_ENABLE();

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

                // Enable output
                timer->BDTR |= TIM_BDTR_MOE;

                // Make sure we are still on channel 1, compile error if user changes this
                static_assert(timch == TIM_CHANNEL_1, "Change registers to change timer channels");

                // Clear CCER
                timer->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1P);

                // Set OC1M to PWM mode 1
                timer->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;

                // Set CCR to half of ARR, for 50% duty cycle
                timer->CCR1 = 10;

                // Enable capture/compare and set polarity to make output normally low
                timer->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P;

                // Enable capture interrupts
                timer->DIER |= TIM_DIER_CC1IE;

                // Enable capture interrupt
                HAL_NVIC_SetPriority(TIM8_CC_IRQn, 0, 0);
                HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);

                // Enable Update interrupt
                HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0, 0);
                HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
            }
        } // namespace C1

        uint32_t lastLogged = 0;
    } // namespace

    void init() { C1::initCell(); }

    void yield() {
        C1::latestReading = C1::scale * (static_cast<float>(C1::rawLatestReading) + C1::offset);

        if(RCP::getDataStreaming() && HAL_GetTick() - lastLogged > 10) {
            lastLogged = HAL_GetTick();
            RCP::sendOneFloat(RCP_DEVCLASS_LOAD_CELL, 0, readCell(0));
        }
    }

    float readCell(uint8_t id) {
        if(id == 0) return C1::latestReading;
        if(id == 1) return 1;
        return 0;
    }

    void tareCell(uint8_t id, float offset) {
        if(id == 0) C1::offset -= offset;
    }


} // namespace LoadCells

/*
This interrupt is placed on DIN, and detects the falling edge indicating a conversion has completed. When this
    occurs, disable the interrupt so it is not constantly triggered when we are reading data. Reset the inputting
    variables, then start the one-shot 25-pulse PWM generation through the timer.
 */
extern "C" void EXTI15_10_IRQHandler(void) {
    using namespace LoadCells;
    // If the interrupt was on the data in pin
    if(__HAL_GPIO_EXTI_GET_IT(C1::dinpin)) {
        // Acknowledge the interrupt
        __HAL_GPIO_EXTI_CLEAR_IT(C1::dinpin);

        // Disable it so it is not retriggering
        LL_EXTI_DisableIT_0_31(C1::dinpin);

        // // Reset counters and stuff
        C1::pulseCount = 0;
        C1::inProgressReading = 0;
        C1::timer->CNT = 0;

        // Start the PWM
        C1::timer->CR1 |= TIM_CR1_CEN;
    }
}

// This interrupt detects when the 25 pulses have finished
extern "C" void TIM8_UP_IRQHandler(void) {
    using namespace LoadCells;
    // Acknowledge the interrupt
    C1::timer->SR &= ~TIM_SR_UIF;
    // Copy the raw reading over to the latest reading variable that can be used for calculating the measurement
    // This copy performs the sign extension for 2s complement 32-bit from unsigned 24-bit
    C1::rawLatestReading =
        static_cast<int32_t>((0xFF000000 & (C1::inProgressReading & 0x00800000)) | // Sign extended portion
                             (C1::inProgressReading & 0x00FFFFFF)); // Value

    // Re-enable the DIN interrupt
    LL_EXTI_EnableIT_0_31(C1::dinpin);
}

// This interrupt is fired whenever the timers CCR and CNT registers are equal, aka at half way through a pulse. If
//     we are still shifting in data (in the first 24 pulses), read the GPIO and accumulate/shift the in progress
//     variable.
extern "C" void TIM8_UP_TIM13_IRQHandler(void) {
    using namespace LoadCells;
    // Acknowledge the interrupt
    C1::timer->SR &= ~TIM_SR_CC1IF;
    // Check if we are still in the data phase
    if(C1::pulseCount++ < 24) {
        // Read DIN, if 1 add 1 to inProgressReading
        static_assert(C1::dinpin == 0x8000, "Must change right shift value if pin changes");
        C1::inProgressReading += (C1::dinport->IDR & C1::dinpin) >> 15;

        // On every clock, left shift the value
        C1::inProgressReading <<= 1;
    }
}
