#include "BoolSensors.h"

#include "main.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal_gpio.h"

#include "RCP_Target/RCP_Target.h"

namespace BoolSensors {
    namespace {
        bool BWState = false;
    }

    void init() {
        if(HAL_GPIO_ReadPin(BURN_WIRE_GPIO_Port, BURN_WIRE_Pin) == GPIO_PIN_SET) RCPDebug("Burn wire not detected!");
        else BWState = true;
    }

    void yield() {
        BWState = HAL_GPIO_ReadPin(BURN_WIRE_GPIO_Port, BURN_WIRE_Pin) == GPIO_PIN_RESET;
    }
} // namespace BoolSensors

bool RCP::readBoolSensor([[maybe_unused]] uint8_t id) {
    return BoolSensors::BWState;
}
