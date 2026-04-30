#include "main.h"

// For some reason, tinyusb header emits some warnings, surpress them
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tusb.h"
#pragma GCC diagnostic pop

#include "RCP_Target/LRIRingBuf.h"
#include "RCP_Target/RCP_Target.h"
#include "RCP_Target/procedures.h"

#include <stdio.h>
#include "BoolSensors.h"
#include "LoadCells.h"
#include "SimpleActuators.h"
#include "Thermocouples.h"
#include "Transducers.h"

tusb_rhport_init_t TUSB_INIT_DATA = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};

namespace Test {
    extern Procedure* const ESTOP;
}

// This function is declared extern C so that it can be called from the C environment. Otherwise, C++ name
// mangling would mean the C call to the function would not link
extern "C" void setup() {
    // Init tinyusb
    tud_rhport_init(BOARD_TUD_RHPORT, &TUSB_INIT_DATA);

    // Wait for the DCR line to go high (set by RCI)
    while(!(tud_cdc_get_line_state() & 0x01)) {
        tud_task_ext(5, false);
    }

    // Turn on this led after connection success, as a visual indicator
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

    // Init the various different parts of the code
    RCP::init();
    RCP::setReady(true);
    RCP::ESTOP_PROC = Test::ESTOP;
    SimpleActuators::init();
    Transducers::init();
    LoadCells::init();
    BoolSensors::init();
    TC::init();
}

// Buffer to store received data for RCP
LRI::RingBuf<uint8_t, 1024> inbuffer;

// Extra temporary buffer to read from tinyusb
uint8_t tempIn[64];

constexpr float VENT_THRESHOLD = 600;
bool prevVenting = false;

// This is also extern C for the same reasons as above
extern "C" void loop() {
    // Make sure to call the tinyusb processing function so it can do its thing. This is why
    // nothing in the code can block
    tud_task_ext(1, false);

    // Read bytes from tinyusb input
    uint32_t read = tud_cdc_read(tempIn, sizeof(tempIn) / sizeof(uint8_t));

    // Push the bytes into the buffer
    for(uint32_t i = 0; i < read; i++) {
        inbuffer.push(tempIn[i]);
    }

    // Make sure to flush output so tinyusb actually sends the data over the wire
    tud_cdc_write_flush();

    // Call various yielding functions
    RCP::yield();
    RCP::runTest();
    Transducers::yield();
    LoadCells::yield();
    BoolSensors::yield();
    TC::yield();
}

// These functions are the 4 required for minimal RCP implementation

void RCP::write(const void* data, uint8_t length) { tud_cdc_write(data, length); }

uint8_t RCP::readAvail() { return inbuffer.size(); }

uint8_t RCP::read() {
    uint8_t val;
    inbuffer.pop(val);
    return val;
}

uint32_t RCP::systime() { return HAL_GetTick(); }

// To be improved
[[noreturn]] void RCP::systemReset() { __NVIC_SystemReset(); }

// Callback for reading from sensor device. Automatic data streaming is handled inside the various singletons
RCP::Floats4 RCP::readSensor(RCP_DeviceClass devclass, uint8_t id) {
    Floats4 floats;

    // Switch on the device class, then load the floats structure with the appropriate data
    switch(devclass) {
    case RCP_DEVCLASS_LOAD_CELL:
        floats.vals[0] = LoadCells::readCell(id);
        break;

    case RCP_DEVCLASS_PRESSURE_TRANSDUCER:
        floats.vals[0] = Transducers::readTransducer(id);
        break;

    case RCP_DEVCLASS_TEMPERATURE:
        floats.vals[0] = TC::readTC(id);
        break;

    default:
        break;
    }

    return floats;
}

// Callback for processing tare requests
void RCP::writeSensorTare(RCP_DeviceClass devclass, uint8_t id, [[maybe_unused]] uint8_t dataChannel, float tareVal) {
    switch(devclass) {
    case RCP_DEVCLASS_LOAD_CELL:
        LoadCells::tareCell(id, tareVal);
        break;

    case RCP_DEVCLASS_PRESSURE_TRANSDUCER:
        Transducers::tare(id, tareVal);
        break;

    default:
        break;
    }
}

namespace Test {
    class TimedBW : public Procedure {
        uint32_t tstart = 0;

    public:
        void initialize() override {
            tstart = RCP::systime();
            RCP::writeSimpleActuator(2, RCP_SIMPLE_ACTUATOR_ON);
        }

        bool isFinished() override { return !RCP::readBoolSensor(0); }

        void end(bool interrupted) override {
            (void) interrupted;
            RCP::writeSimpleActuator(2, RCP_SIMPLE_ACTUATOR_OFF);
            char text[60];
            snprintf(text, sizeof(text), "Burn wire delay: %ldms\n", RCP::systime() - tstart);
            RCP::RCPWriteSerialString(text);
        }

        ~TimedBW() override = default;
    };

    class Hotfire : public Procedure {
        enum class State { INIT, EMATCH_WAIT, BURN_WAIT, OX_WAIT, FIRE_WAIT, END } state;
        uint32_t timer;

        void abort() {
            RCP::writeSimpleActuator(SimpleActuators::SOL_9_id, RCP_SIMPLE_ACTUATOR_OFF);
            RCP::writeSimpleActuator(SimpleActuators::SOL_7_id, RCP_SIMPLE_ACTUATOR_ON);
        }

    public:
        Hotfire() = default;

        void initialize() override { state = State::INIT; }

        void execute() override {
            using namespace SimpleActuators;

            switch(state) {
            case State::INIT:
                if(RCP::readBoolSensor(0)) {
                    RCPDebug("[HOTFIRE] Burn Wire detected");
                    RCPDebug("[HOTFIRE] Setting EMatch");
                    RCP::writeSimpleActuator(EMATCH_ID, RCP_SIMPLE_ACTUATOR_ON);

                    state = State::EMATCH_WAIT;
                    timer = HAL_GetTick();
                }

                else {
                    RCPDebug("[HOTFIRE] Burn wire not detected!");
                    state = State::END;
                }
                break;

            case State::EMATCH_WAIT:
                if(HAL_GetTick() - timer > 100) {
                    state = State::BURN_WAIT;
                    RCP::writeSimpleActuator(EMATCH_ID, RCP_SIMPLE_ACTUATOR_OFF);
                    RCPDebug("[HOTFIRE] Ignition complete");
                }
                break;

            case State::BURN_WAIT:
                if(HAL_GetTick() - timer < 5000) {
                    if(!RCP::readBoolSensor(0)) {
                        RCP::writeSimpleActuator(SOL_4_id, RCP_SIMPLE_ACTUATOR_ON);
                        timer = HAL_GetTick();
                        state = State::OX_WAIT;
                        RCPDebug("[HOTFIRE] Burn wire cut!");
                        RCPDebug("[HOTFIRE] Starting 30s burn...");
                    }
                }

                else {
                    RCPDebug("[HOTFIRE] Burn wire 5s timeout hit, aborting!");
                    timer = HAL_GetTick();
                    state = State::END;
                }
                break;

            case State::OX_WAIT:
                if(HAL_GetTick() - timer > 500) {
                    RCP::writeSimpleActuator(SOL_3_id, RCP_SIMPLE_ACTUATOR_ON);
                    RCPDebug("[HOTFIRE] Opening OX MBV");
                    timer = HAL_GetTick();
                    state = State::FIRE_WAIT;
                }

                break;

            case State::FIRE_WAIT:
            case State::END:
                break;
            }
        }

        bool isFinished() override { return state == State::END; }

        void end(bool interrupted) override {
            if(interrupted) {
                RCP::writeSimpleActuator(SimpleActuators::SOL_3_id, RCP_SIMPLE_ACTUATOR_OFF);
                RCP::writeSimpleActuator(SimpleActuators::SOL_4_id, RCP_SIMPLE_ACTUATOR_OFF);
            }
        }

        ~Hotfire() override = default;
    };

    class DanceMode : public Procedure {
        uint32_t timer;
        bool state;
        RCP_SimpleActuatorState startState;

    public:
        DanceMode() = default;
        ~DanceMode() override = default;

        void initialize() override {
            timer = HAL_GetTick();
            state = false;
            startState = RCP::readSimpleActuator(SimpleActuators::SOL_9_id);
            RCP::writeSimpleActuator(SimpleActuators::SOL_9_id, RCP_SIMPLE_ACTUATOR_OFF);
        }

        void execute() override {
            if(HAL_GetTick() - timer > 500) {
                timer = HAL_GetTick();
                RCP::writeSimpleActuator(SimpleActuators::SOL_9_id,
                                         state ? RCP_SIMPLE_ACTUATOR_OFF : RCP_SIMPLE_ACTUATOR_ON);
                state = !state;
            }
        }

        bool isFinished() override { return false; }

        void end(bool interrupted) override {
            (void) interrupted;
            RCP::writeSimpleActuator(SimpleActuators::SOL_9_id, startState);
        }
    };

    class TimedValves : public Procedure {
        uint32_t tstart = 0;

    public:
        void initialize() override {
            tstart = RCP::systime();
            RCP::writeSimpleActuator(6, RCP_SIMPLE_ACTUATOR_ON);
            RCP::writeSimpleActuator(5, RCP_SIMPLE_ACTUATOR_ON);
        }

        void end(bool interrupted) override {
            (void) interrupted;
            RCP::writeSimpleActuator(6, RCP_SIMPLE_ACTUATOR_OFF);
            RCP::writeSimpleActuator(5, RCP_SIMPLE_ACTUATOR_OFF);
            char text[60];
            snprintf(text, sizeof(text), "Valve open for: %ldms\n", RCP::systime() - tstart);
            RCP::RCPWriteSerialString(text);
        }

        bool isFinished() override { return false; }

        ~TimedValves() override = default;
    };

    Procedure* const ESTOP = new OneShot([] {
        using namespace SimpleActuators;
        RCP::writeSimpleActuator(SOL_1_id, RCP_SIMPLE_ACTUATOR_OFF);
        RCP::writeSimpleActuator(SOL_2_id, RCP_SIMPLE_ACTUATOR_OFF);
        RCP::writeSimpleActuator(SOL_3_id, RCP_SIMPLE_ACTUATOR_OFF);
        RCP::writeSimpleActuator(SOL_4_id, RCP_SIMPLE_ACTUATOR_OFF);
        RCP::writeSimpleActuator(SOL_7_id, RCP_SIMPLE_ACTUATOR_ON);
        RCP::writeSimpleActuator(SOL_9_id, RCP_SIMPLE_ACTUATOR_OFF);
        RCP::writeSimpleActuator(SOL_11_id, RCP_SIMPLE_ACTUATOR_OFF);
    });

    // Test 1 is a program to open the MBV and track the time they are open for. The time is then printed to console
    // Test 2 is a simple test to open the ball valves at the same time

    // clang-format off
    Tests tests = {
        new TimedBW(),
        new Hotfire(),
        new DanceMode(),
        new TimedValves(),
        new Procedure(),
        new Procedure(),
        new Procedure(),
        new Procedure(),
        new Procedure(),
        new Procedure(),
        new Procedure(),
        new Procedure(),
        new Procedure(),
        new Procedure(),
        new Procedure(),
        new Procedure(),
    };
    // clang-format on


    Tests& getTests() { return tests; }
} // namespace Test
