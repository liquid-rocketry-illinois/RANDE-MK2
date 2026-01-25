#include "SimpleActuators.h"

#include "RCP_Target/RCP_Target.h"

namespace SimpleActuators {
    namespace {
        const GPIO ACT_PINS[NUM_ACTS] = {
            {SCH0_GPIO_Port, SCH0_Pin}, {SCH1_GPIO_Port, SCH1_Pin}, {SCH2_GPIO_Port, SCH2_Pin},
            {SCH3_GPIO_Port, SCH3_Pin}, {SCH4_GPIO_Port, SCH4_Pin}, {SCH5_GPIO_Port, SCH5_Pin},
            {SCH6_GPIO_Port, SCH6_Pin}, {SCH7_GPIO_Port, SCH7_Pin},
        };

        RCP_SimpleActuatorState states[NUM_ACTS];
        bool inited = false;
    } // namespace

    void init() {
        for(const auto& [port, pin] : ACT_PINS) {
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
        }

        inited = true;
        RCPDebug("[SOLENOIDS] Solenoids initialized");
        RCPDebug("[EMATCH] EMatch Initialized");
    }
} // namespace SimpleActuators

RCP_SimpleActuatorState RCP::simpleActuatorWrite_CLBK(uint8_t id, RCP_SimpleActuatorState state) {
    if(state == RCP_SIMPLE_ACTUATOR_TOGGLE)
        SimpleActuators::states[id] = SimpleActuators::states[id] ? RCP_SIMPLE_ACTUATOR_OFF : RCP_SIMPLE_ACTUATOR_ON;
    else SimpleActuators::states[id] = state;

    HAL_GPIO_WritePin(SimpleActuators::ACT_PINS[id].port, SimpleActuators::ACT_PINS[id].pin,
                      SimpleActuators::states[id] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return SimpleActuators::states[id];
}

RCP_SimpleActuatorState RCP::readSimpleActuator(uint8_t id) {
    return SimpleActuators::states[id];
}
