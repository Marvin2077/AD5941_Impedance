#include "ad5941_board_glue.h"

// 用 C 方式引入 ADI 库头
extern "C" {
#include "ad5940.h"
}

namespace {
/*

g (全局结构体) 
  ↓
g.spi (指向SpiDevice的指针)
  ↓
g.spi->beginTxn() (调用SpiDevice的成员函数)
  ↓
SPI.beginTransaction(settings) (最终调用Arduino SPI库)

*/
// 全局状态，供 C 回调访问
struct State {
  const SpiDevice* spi = nullptr;
  uint8_t pin_reset = 0xFF;
  uint8_t pin_int   = 0xFF;
  bool in_txn = false;  // 是否处于一次 SPI 事务中（CsClr..CsSet 之间）
} g;

// ---- 以下为 ADI 库需要的 C 回调 ----
extern "C" void AD5940_CsClr(void) {
  if (!g.spi) return;
  g.spi->beginTxn();         // 进入一次事务
  g.in_txn = true;
  g.spi->cs_low();           // 片选有效
}

extern "C" void AD5940_CsSet(void) {
  if (!g.spi) return;
  g.spi->cs_high();          // 片选无效
  if (g.in_txn) {            // 结束事务
    g.spi->endTxn();
    g.in_txn = false;
  }
}

extern "C" void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer, unsigned char *pRecvBuff, unsigned long length) {
  if (!g.spi || length == 0) return;
  g.spi->transfer(pSendBuffer, pRecvBuff, static_cast<size_t>(length));
}

extern "C" void AD5940_Delay10us(uint32_t n) {
  // ADI 要求：单位为 10us
  if (n == 0) return;
  SpiHAL::delay_us(n * 10);
}

// 若接了硬复位脚，可以用库的复位钩子；未配置则空操作
extern "C" void AD5940_RstClr(void) {
  if (g.pin_reset == 0xFF) return;
  digitalWrite(g.pin_reset, LOW);
}
extern "C" void AD5940_RstSet(void) {
  if (g.pin_reset == 0xFF) return;
  digitalWrite(g.pin_reset, HIGH);
}

} // anonymous namespace

// ---- 对外 API 实现 ----
namespace Ad5941Glue {

void setup(const Config& cfg) {
  g.spi       = cfg.spi;
  g.pin_reset = cfg.pin_reset;
  g.pin_int   = cfg.pin_int;

  if (g.spi) {
    // 初始化 CS 引脚并默认拉高
    g.spi->begin();
  }
  if (g.pin_reset != 0xFF) {
    pinMode(g.pin_reset, OUTPUT);
    digitalWrite(g.pin_reset, HIGH);
  }
  if (g.pin_int != 0xFF) {
    pinMode(g.pin_int, INPUT_PULLUP);
  }
}

void hardware_reset(uint16_t low_us, uint16_t settle_ms) {
  if (g.pin_reset == 0xFF) return;
  digitalWrite(g.pin_reset, LOW);
  SpiHAL::delay_us(low_us);
  digitalWrite(g.pin_reset, HIGH);
  SpiHAL::delay_ms(settle_ms);
}

uint32_t read_chip_id() {
  // 可在 AD5940_Initialize() 前后调用，作为通信冒烟测试
  return AD5940_ReadReg(REG_AFECON_ADIID);
}

bool ready() {
  return g.spi != nullptr;
}

} // namespace Ad5941Glue
