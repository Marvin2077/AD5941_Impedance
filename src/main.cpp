#include <Arduino.h>
#include <SPI.h>
#include "ad5941_board_glue.h"      // 引入 Glue 层头文件
#include "spi_hal.h"                // 引入 SPI HAL 头文件
#include "impedance_service.h"

// 用 C 方式引入 ADI 库头文件
extern "C" {
#include "ad5940.h"  
}

enum class AppState {
  IDLE,                 // 空闲状态，可以触发新的测量
  TRIGGERING_IMPEDANCE,      // 正在触发电导测量
  WAITING_IMPEDANCE_RESULT,    // 已触发电导测量，正在等待结果
  TRIGGERING_TEMP,      // 正在触发温度测量
  WAITING_TEMP_RESULT,  // 正在等待温度结果
  TRIGGERING_LPDAC,     // 正在触发 LPDAC 设置
  WAITING_LPDAC_FINISH  // 正在等待 LPDAC 序列结束
};
AppState current_state = AppState::IDLE; // 当前应用状态

#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];

// === 阻抗测量相关常量和变量 ===
unsigned long last_imp_trigger_time = 0;      // 上次触发温度测量的时间戳 (ms)
const unsigned long IMP_READ_INTERVAL_MS = 2000; // 设置多久读取一次温度 (例如 2000ms = 2秒)
const unsigned long IMP_POLL_INTERVAL_MS = 50;  // 设置轮询温度结果的间隔 (例如 50ms)
const unsigned long IMP_TIMEOUT_MS = 1000;       // 设置等待温度结果的超时时间 (例如 500ms)
static unsigned long last_imp_poll_time = 0;    // 上次轮询温度的时间戳 (ms)

/* ====== 共用 SPI 总线引脚 ====== */
static const int PIN_SCLK = 14; // ESP32 SPI 时钟线
static const int PIN_MISO = 12; // ESP32 SPI MISO线
static const int PIN_MOSI = 13; // ESP32 SPI MOSI线

/* ====== AD5941 特定引脚 ====== */
static const int CS_AD5941    = 26; // AD5941 片选引脚
static const int RESET_AD5941 = 25; // AD5941 复位引脚

// 函数声明
static void ad5941_basic_probe(); // 芯片通信探测函数
static void platform_clock_init(); // 平台时钟初始化函数

// === Arduino Setup ===
void setup() {
  Serial.begin(115200); // 初始化串口
  delay(200);
  Serial.println("======================================");
  Serial.println(" AD5941 Setup Start ");
  Serial.println("======================================");

  // 1) 初始化 SPI 总线
  SpiHAL::beginBus(PIN_SCLK, PIN_MISO, PIN_MOSI);
  Serial.println("SPI bus initialized.");

  // 2) 创建 AD5941 的 SpiDevice 对象
  static SpiDevice ad5941_dev(CS_AD5941, 8000000 /* 8MHz */, MSBFIRST, SPI_MODE0);
  Serial.println("SpiDevice for AD5941 created.");

  // 3) 配置并初始化 Ad5941Glue 层
  Ad5941Glue::Config cfg;
  cfg.spi = &ad5941_dev;
  cfg.pin_reset = RESET_AD5941;
  Ad5941Glue::setup(cfg);
  Serial.println("Ad5941Glue setup complete.");

  // 4) 执行硬件复位
  Ad5941Glue::hardware_reset();
  Serial.println("AD5941 hardware reset performed.");

  // 7) 读取 ADI ID 与 CHIP ID 进行通信测试 (通过 Glue 层调用 ADI 库函数)
  ad5941_basic_probe();


}

// === Arduino Loop ===
void loop() {

unsigned long current_time = millis(); // 获取当前时间戳 (ms)

  // --- 应用状态机 ---
  
  switch (current_state) {
    case AppState::IDLE:
      // --- 在空闲状态下，检查是否需要触发温度测量 ---
      if (current_time - last_imp_trigger_time >= IMP_READ_INTERVAL_MS) {
        current_state = AppState::TRIGGERING_IMPEDANCE; // 切换到触发测量阻抗状态
      }
      break; // IDLE 状态结束
// test
    case AppState::TRIGGERING_IMPEDANCE:
      // --- 触发阻抗测量序列 ---
      Serial.println("Triggering Impedance sequence...");
      if (AD5940_Imp_Seq_Trigger() == AD5940ERR_OK) {
        current_state = AppState::WAITING_IMPEDANCE_RESULT; // 切换到等待阻抗结果状态
        last_imp_trigger_time = current_time;       // 记录触发时间
        last_imp_poll_time = current_time;          // 准备立即开始轮询
        Serial.println("Temp sequence triggered. Waiting for result...");
      } else {
        Serial.println("Trigger failed! Retrying later.");
        current_state = AppState::IDLE;              // 触发失败，返回空闲
        last_imp_trigger_time = current_time;       // 更新时间戳防止立即重试
      }
      break; // TRIGGERING_TEMP 状态结束

    case AppState::WAITING_IMPEDANCE_RESULT: 
      // --- 轮询等待温度测量结果 ---
      if (current_time - last_imp_poll_time >= IMP_POLL_INTERVAL_MS) {
        last_imp_poll_time = current_time; // 更新轮询时间

        uint32_t temp;  
        temp = APPBUFF_SIZE;
        AD5940_Imp_Seq_ReadResult(AppBuff, &temp);
        if (temp) {
        
          ImpedanceShowResult(AppBuff, temp);

          // 测量完成，转换回空闲状态，准备下一次触发
          current_state = AppState::IDLE;
        } else {
          // 结果未就绪 (序列未结束)，检查是否超时 (这部分逻辑不变)
          if (current_time - last_imp_trigger_time > IMP_TIMEOUT_MS) {
            Serial.println("Timeout waiting for impedance sequence finish! Returning to IDLE."); // 更新打印信息
            current_state = AppState::IDLE; // 超时，返回空闲状态
            last_imp_trigger_time = current_time; // 更新时间戳
            AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ); // 超时也要清标志
          }
          // 未超时则继续等待下一次轮询
        }
      }
      break; // WAITING_TEMP_RESULT / WAITING_TEMP_FINISH 状态结束
 
   
    }
}

// --- 辅助函数：读取芯片 ID --- (保持不变)
static void ad5941_basic_probe() {
  Serial.println("Probing AD5941 Registers...");
  uint32_t adiid   = AD5940_ReadReg(REG_AFECON_ADIID);     //
  uint32_t chipid  = AD5940_ReadReg(REG_AFECON_CHIPID);    //
  uint32_t afecon  = AD5940_ReadReg(REG_AFE_AFECON);       //

  Serial.printf("  ADIID   = 0x%08lX (expect 0x00004144)\n", adiid);
  Serial.printf("  CHIPID  = 0x%08lX\n", chipid);
  Serial.printf("  AFECON  = 0x%08lX (Reset: 0x00080000)\n", afecon);

  if ((adiid & 0xFFFF) != 0x4144) { //
    Serial.println("!!! WARNING: ADIID mismatch! Check SPI wiring (MOSI/MISO), mode (MODE0), or speed.");
  } else {
    Serial.println("AD5941 communication probe successful ✅");
  }
  Serial.println("--------------------------------------");
}