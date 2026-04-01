#include "Thermocouples.h"

#include <cstdio>

#include "main.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_i2c.h"

#include "RCP_Target/RCP_Target.h"

namespace TC {
    namespace {
        constexpr uint16_t MCPADD = 0xC0;

        constexpr uint16_t MCP_THOT = 0x00;
        constexpr uint16_t MCP_TCONF = 0x00;
        constexpr uint16_t MCP_REV = 0x20;

        constexpr uint8_t MCP_REV_GOLD = 0x40; // May be 41 if mcpL
        constexpr uint8_t MCP_TCONF_VAL = 0x20;

        constexpr uint32_t POLL_INT = 100;

        void writeMux(uint8_t id) {
            if(id >= NUM_TC) return;

            HAL_GPIO_WritePin(MUXA0_GPIO_Port, MUXA0_Pin, id & 0x01 ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MUXA1_GPIO_Port, MUXA1_Pin, id & 0x02 ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MUXA2_GPIO_Port, MUXA2_Pin, id & 0x04 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        bool ready[NUM_TC];
        float latestReading[NUM_TC];
        float offsets[NUM_TC];

        uint32_t lastPoll = 0;

        char messageBuf[60];
        constexpr size_t mBufSize = sizeof(messageBuf) / sizeof(char);
    } // namespace
    void init() {
        for(uint8_t i = 0; i < NUM_TC; i++) {
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
        if(HAL_GetTick() - lastPoll < POLL_INT) return;
        lastPoll = HAL_GetTick();

        uint8_t data[2];

        for(uint8_t i = 0; i < NUM_TC; i++) {
            writeMux(i);

            HAL_I2C_Mem_Read(&hi2c2, MCPADD, MCP_THOT, I2C_MEMADD_SIZE_8BIT, data, 2, 10);

            int16_t raw_latest = *reinterpret_cast<int16_t*>(data);
            latestReading[i] =
                (static_cast<float>(raw_latest) / 16 + static_cast<float>(raw_latest % 16) * 0.0625f) + offsets[i];
        }

        if(RCP::getDataStreaming()) {
            for(uint8_t i = 0; i < NUM_TC; i++) RCP::sendOneFloat(RCP_DEVCLASS_TEMPERATURE, i, latestReading[i]);
        }
    }

    float readTC(uint8_t id) { return latestReading[id]; }

    void tareTC(uint8_t id, float offset) {
        offsets[id] += offset;
    }
} // namespace TC
