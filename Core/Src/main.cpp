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

#include "SimpleActuators.h"
#include "Transducers.h"
#include "LoadCells.h"

tusb_rhport_init_t TUSB_INIT_DATA = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};

extern "C" void setup() {
    tud_rhport_init(BOARD_TUD_RHPORT, &TUSB_INIT_DATA);

    while(!(tud_cdc_get_line_state() & 0x01)) {
        tud_task_ext(5, false);
    }

    // while(HAL_GPIO_ReadPin(USRBTN_GPIO_Port, USRBTN_Pin) == GPIO_PIN_RESET) {tud_task_ext(5, false);}
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

    RCP::init();
    RCP::setReady(true);
    SimpleActuators::init();
    Transducers::init();
    LoadCells::init();
}

LRI::RingBuf<uint8_t, 1024> inbuffer;
uint8_t tempIn[64];

uint32_t last = 0;
extern "C" void loop() {
    tud_task_ext(1, false);
    uint32_t read = tud_cdc_read(tempIn, sizeof(tempIn) / sizeof(uint8_t));
    for(uint32_t i = 0; i < read; i++) {
        inbuffer.push(tempIn[i]);
    }

    tud_cdc_write_flush();

    // for(int i = 0; i < 64; i++) {
        // uint8_t b;
        // HAL_StatusTypeDef ret = HAL_UART_Receive(&huart3, &b, 1, 1);
        // if(ret == HAL_OK) inbuffer.push(b);
        // else break;
    // }

    RCP::yield();
    RCP::runTest();
    Transducers::yield();
    LoadCells::yield();

    // if(HAL_GetTick() - last > 2500) {
    //     last = HAL_GetTick();
    //     // RCPDebug("test");
    // }
}

void RCP::write(const void* data, uint8_t length) {
    tud_cdc_write(data, length);
    // HAL_UART_Transmit(&huart3, static_cast<const uint8_t*>(data), length, HAL_MAX_DELAY);
}

uint8_t RCP::readAvail() { return inbuffer.size(); }

uint8_t RCP::read() {
    uint8_t val;
    inbuffer.pop(val);
    return val;
}

uint32_t RCP::systime() { return HAL_GetTick(); }

[[noreturn]] void RCP::systemReset() { __NVIC_SystemReset(); }

RCP::Floats4 RCP::readSensor(RCP_DeviceClass devclass, uint8_t id) {
    Floats4 floats;

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
    Tests tests = {
        new Procedure(),
        new Procedure(),
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

    Tests& getTests() {
        return tests;
    }
}