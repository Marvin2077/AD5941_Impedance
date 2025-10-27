#ifndef AD5941_BOARD_GLUE_H
#define AD5941_BOARD_GLUE_H

#include <Arduino.h>
#include "spi_hal.h"

// 对外配置
namespace Ad5941Glue {

struct Config {
  const SpiDevice* spi = nullptr;   // 必须：SPI 设备（内含 CS/Mode/Speed）
  uint8_t pin_reset   = 0xFF;       // 可选：硬复位脚；0xFF 表示未连接
  uint8_t pin_int     = 0xFF;       // 可选：中断脚（此版本未用）
};

/** 绑定 SPI/引脚并完成基础初始化（配置 CS 为输出并拉高） */
void setup(const Config& cfg);

/** 若连了硬复位脚，执行一次低-高脉冲；未配置则空操作 */
void hardware_reset(uint16_t low_us = 10, uint16_t settle_ms = 2);

/** 读芯片 ID 做冒烟测试（需先调用 AD5940_Initialize 之前/之后都可读取） */
uint32_t read_chip_id();

/** 返回是否已绑定有效 SPI */
bool ready();

} // namespace Ad5941Glue

#endif // AD5941_BOARD_GLUE_H
