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
#include "LoadCells.h"
#include "SimpleActuators.h"
#include "Transducers.h"

tusb_rhport_init_t TUSB_INIT_DATA = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};

// This function is declared extern C so that it can be called from the C environment. Otherwise, C++ name
// mangling would mean the C call to the function would not link
extern "C" void setup() {
    // while(HAL_GPIO_ReadPin(USRBTN_GPIO_Port, USRBTN_Pin) != GPIO_PIN_SET);
    // HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    // LoadCells::init();
    // return;
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
    SimpleActuators::init();
    Transducers::init();
    LoadCells::init();
}

// Buffer to store received data for RCP
LRI::RingBuf<uint8_t, 1024> inbuffer;

// Extra temporary buffer to read from tinyusb
uint8_t tempIn[64];

// This is also extern C for the same reasons as above
extern "C" void loop() {
    // LoadCells::yield();
    // printf("Val: %f\n", LoadCells::readCell(0));
    // HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    // HAL_Delay(1000);
    // return;
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
    class TimedValves : public Procedure {
        uint32_t tstart = 0;
    public:
        void initialize() override {
            tstart = RCP::systime();
            RCP::writeSimpleActuator(6, RCP_SIMPLE_ACTUATOR_ON);
        }

        void end(bool interrupted) override {
            (void) interrupted;
            RCP::writeSimpleActuator(6, RCP_SIMPLE_ACTUATOR_OFF);
            char text[60];
            snprintf(text, sizeof(text), "Valve open for: %ldms\n", RCP::systime() - tstart);
            RCP::RCPWriteSerialString(text);
        }

        bool isFinished() override {
            return false;
        }
    };

    // Test 1 is a program to open the MBV and track the time they are open for. The time is then printed to console
    // Test 2 is a simple test to open the ball valves at the same time
    Tests tests = {
        new Procedure(),
        new TimedValves(),
        new OneShot([] {
            RCP::writeSimpleActuator(6, RCP_SIMPLE_ACTUATOR_ON);
            RCP::writeSimpleActuator(5, RCP_SIMPLE_ACTUATOR_ON);
        }),
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
        new Procedure(),
    };

    Tests& getTests() { return tests; }
} // namespace Test
