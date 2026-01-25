#include "main.h"

// For some reason, tinyusb header emits some warnings, surpress them
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tusb.h"
#pragma GCC diagnostic pop

#include "RCP_Target/RCP_Target.h"
#include "RCP_Target/LRIRingBuf.h"

#include "SimpleActuators.h"

tusb_rhport_init_t TUSB_INIT_DATA = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};

extern "C" void setup() {
    tud_rhport_init(BOARD_TUD_RHPORT, &TUSB_INIT_DATA);

    while(!(tud_cdc_get_line_state() & 0x01)) {
        tud_task_ext(5, false);
    }

    RCP::init();
    RCP::setReady(true);
    SimpleActuators::init();
}

LRI::RingBuf<uint8_t, 1024> inbuffer;
uint8_t tempIn[64];

extern "C" void loop() {
    tud_task_ext(1, false);
    uint32_t read = tud_cdc_read(tempIn, sizeof(tempIn) / sizeof(uint8_t));
    for(uint32_t i = 0; i < read; i++) {
        inbuffer.push(tempIn[i]);
    }

    RCP::yield();
    RCP::runTest();
}

void RCP::write(const void* data, uint8_t length) {
    tud_cdc_write(data, length);
    tud_cdc_write_flush();
}

uint8_t RCP::readAvail() { return inbuffer.size(); }

uint8_t RCP::read() {
    uint8_t val;
    inbuffer.pop(val);
    return val;
}

uint32_t RCP::systime() { return HAL_GetTick(); }

[[noreturn]] void RCP::systemReset() {
    __NVIC_SystemReset();
}


