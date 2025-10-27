#pragma once
#include <Arduino.h>
#include <SPI.h>
//为Arduino写的SPI接口
struct SpiDevice {
  uint8_t  cs_pin;
  uint32_t clk_hz;
  uint8_t  bit_order;   // MSBFIRST
  uint8_t  spi_mode;    // SPI_MODE0/1/2/3
  SPISettings settings;

  SpiDevice(uint8_t cs, uint32_t hz, uint8_t order, uint8_t mode)
  : cs_pin(cs), clk_hz(hz), bit_order(order), spi_mode(mode),
    settings(hz, order, mode) {}

  inline void begin() const { pinMode(cs_pin, OUTPUT); digitalWrite(cs_pin, HIGH); }
  inline void cs_low() const  { digitalWrite(cs_pin, LOW); }
  inline void cs_high() const { digitalWrite(cs_pin, HIGH); }

  inline void beginTxn() const { SPI.beginTransaction(settings); }
  inline void endTxn()   const { SPI.endTransaction(); }

  // 传输 n 字节
  inline void transfer(const uint8_t* tx, uint8_t* rx, size_t n) const {
    for (size_t i=0;i<n;++i){
      uint8_t out = tx ? tx[i] : 0xFF;
      uint8_t in  = SPI.transfer(out);
      if (rx) rx[i] = in;
    }
  }
};

// 板级通用函数
namespace SpiHAL {
  // 初始化总线引脚
  inline void beginBus(int sclk, int miso, int mosi) { SPI.begin(sclk, miso, mosi); }
  inline void delay_us(uint32_t us) { delayMicroseconds(us); }
  inline void delay_ms(uint32_t ms) { delay(ms); }
}
