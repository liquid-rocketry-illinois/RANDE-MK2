#include "Thermocouples.h"

#include <cstdio>

#include "main.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_i2c.h"

#include "RCP_Target/RCP_Target.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tusb.h"
#pragma GCC diagnostic pop

namespace TC {
    namespace {
        // The multiplexer address for each connected MCP
        constexpr uint8_t TC_ADDRS[] = {2, 1, 4, 7, 3, 0, 5, 6};

        // I2C address for the MCP
        constexpr uint16_t MCPADD = 0x60 << 1;

        // Register address for MCP hot junction temperature
        constexpr uint16_t MCP_THOT = 0x00;

        // Register address for MCP config register
        constexpr uint16_t MCP_TCONF = 0x00;

        // Register adddress for revision
        constexpr uint16_t MCP_REV = 0x20;

        // What value we should read from the revision register
        constexpr uint8_t MCP_REV_GOLD = 0x40;

        // What value needs to be written into the config register to set T type thermocouple
        constexpr uint8_t MCP_TCONF_VAL = 0x20;

        // Polling interval, in ms
        constexpr uint32_t POLL_INT = 100;

        // Address of the TCA multiplexer
        constexpr uint8_t TCA_ADDR = 0x70 << 1;

        // Sets the multiplexer to the given device
        void writeMux(uint8_t id) {
            if(id >= NUM_TC) return;

            // Get address of device
            uint8_t data = 1 << TC_ADDRS[id];

            // Set multiplexer
            HAL_I2C_Master_Transmit(&hi2c2, TCA_ADDR, &data, 1, HAL_MAX_DELAY);
        }

        bool ready[NUM_TC];
        float latestReading[NUM_TC];

        uint32_t lastPoll = 0;

        char messageBuf[60];
        constexpr size_t mBufSize = sizeof(messageBuf) / sizeof(char);
    } // namespace

    // Initializes TCA and all MCPs with their config values
    void init() {
        HAL_GPIO_WritePin(MUXA0_GPIO_Port, MUXA0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MUXA1_GPIO_Port, MUXA1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MUXA2_GPIO_Port, MUXA2_Pin, GPIO_PIN_RESET);

        // Reset TCA
        HAL_GPIO_WritePin(TCA_NRST_GPIO_Port, TCA_NRST_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(TCA_NRST_GPIO_Port, TCA_NRST_Pin, GPIO_PIN_SET);

        // Loop over all MCPs, check they are responding, check revision, and write config register
        for(uint8_t i = 0; i < NUM_TC; i++) {
            // Make sure to keep running the USB task since this loop takes long enough to cause USB issues
            tud_task_ext(1, false);
            writeMux(i);

            if(HAL_I2C_IsDeviceReady(&hi2c2, MCPADD, 3, 100) != HAL_OK) {
                snprintf(messageBuf, sizeof(messageBuf) / sizeof(char), "MCP %d did not report ready\n", i);
                RCP::RCPWriteSerialString(messageBuf);
                continue;
            }

            uint8_t data;
            HAL_I2C_Mem_Read(&hi2c2, MCPADD, MCP_REV, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

            if(data != MCP_REV_GOLD) {
                snprintf(messageBuf, mBufSize, "MCP reported revision %d instead of %d\n", data, MCP_REV_GOLD);
                RCP::RCPWriteSerialString(messageBuf);
                continue;
            }

            data = MCP_TCONF_VAL;
            HAL_I2C_Mem_Write(&hi2c2, MCPADD, MCP_TCONF, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
            ready[i] = true;
        }
    }

    void yield() {
        // Check if the polling interval has passed
        if(HAL_GetTick() - lastPoll < POLL_INT) return;
        lastPoll = HAL_GetTick();

        uint8_t data[2];

        // Loop all TCAs and grab the data
        for(uint8_t i = 0; i < NUM_TC; i++) {
            // See init
            tud_task_ext(1, false);

            if(!ready[i]) continue;
            writeMux(i);

            HAL_I2C_Mem_Read(&hi2c2, MCPADD, MCP_THOT, I2C_MEMADD_SIZE_8BIT, data, 2, 10);

            int16_t raw = static_cast<int16_t>((data[0] << 8) | data[1]);
            latestReading[i] = static_cast<float>(raw / 16) + 0.0625f * static_cast<float>(raw % 16);
        }

        if(RCP::getDataStreaming()) {
            for(uint8_t i = 0; i < NUM_TC; i++) RCP::sendOneFloat(RCP_DEVCLASS_TEMPERATURE, i, latestReading[i]);
        }
    }

    float readTC(uint8_t id) { return latestReading[id]; }
} // namespace TC
