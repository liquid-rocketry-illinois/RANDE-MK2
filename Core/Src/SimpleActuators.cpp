#include "SimpleActuators.h"

#include "RCP_Target/RCP_Target.h"

// Very simple singleton namespace for handling the solenoids and ematch
namespace SimpleActuators {
    namespace {
        // This array stores pairs of gpio port and pin that encode which pin an actuator belongs to. RCP IDs correspond
        // to index into array

        // clang-format off
        const GPIO ACT_PINS[NUM_ACTS] = {
            {SCH0_GPIO_Port, SCH0_Pin},
            {SCH1_GPIO_Port, SCH1_Pin},
            {SCH2_GPIO_Port, SCH2_Pin},
            {SCH3_GPIO_Port, SCH3_Pin},
            {SCH4_GPIO_Port, SCH4_Pin},
            {SCH5_GPIO_Port, SCH5_Pin},
            {SCH6_GPIO_Port, SCH6_Pin},
            {SCH8_GPIO_Port, SCH8_Pin, true, SCH9_GPIO_Port, SCH9_Pin},
            {SCH10_GPIO_Port, SCH10_Pin, true, SCH11_GPIO_Port, SCH11_Pin}
        };
        // clang-format on

        // States for all the actuators
        RCP_SimpleActuatorState states[NUM_ACTS];

        // Simple check for initialization
        bool inited = false;
    } // namespace

    void init() {
        // Write all actuators to off
        for(const auto& [port, pin, dual, port2, pin2] : ACT_PINS) {
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
            if(dual) HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_RESET);
        }

        // The level shifter has an extra output enable, so turn that on too
        HAL_GPIO_WritePin(ACT_EN0_GPIO_Port, ACT_EN0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ACT_EN1_GPIO_Port, ACT_EN1_Pin, GPIO_PIN_SET);
        inited = true;
        RCPDebug("[EMATCH] EMatch Initialized");
        RCPDebug("[SOLENOIDS] Solenoids initialized");
    }

    // When performing reset, all the gpios for some reason go kinda crazy, so reset the output enable
    // on the level shifter so nothing actually happens
    void deinit() {
        HAL_GPIO_WritePin(ACT_EN0_GPIO_Port, ACT_EN0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ACT_EN1_GPIO_Port, ACT_EN1_Pin, GPIO_PIN_RESET);
    }
} // namespace SimpleActuators

// Callbacks for writes and reads to the actuators
RCP_SimpleActuatorState RCP::simpleActuatorWrite_CLBK(uint8_t id, RCP_SimpleActuatorState state) {
    if(state == RCP_SIMPLE_ACTUATOR_TOGGLE)
        SimpleActuators::states[id] = SimpleActuators::states[id] ? RCP_SIMPLE_ACTUATOR_OFF : RCP_SIMPLE_ACTUATOR_ON;
    else SimpleActuators::states[id] = state;

    HAL_GPIO_WritePin(SimpleActuators::ACT_PINS[id].port, SimpleActuators::ACT_PINS[id].pin,
                      SimpleActuators::states[id] ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if(SimpleActuators::ACT_PINS[id].dual)
        HAL_GPIO_WritePin(SimpleActuators::ACT_PINS[id].port2, SimpleActuators::ACT_PINS[id].pin2,
                          SimpleActuators::states[id] ? GPIO_PIN_RESET : GPIO_PIN_SET);

    return SimpleActuators::states[id];
}

RCP_SimpleActuatorState RCP::readSimpleActuator(uint8_t id) { return SimpleActuators::states[id]; }
