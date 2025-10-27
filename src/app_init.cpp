#include "app_init.h"
//#include "internal_temp_service.h" // 需要调用温度服务的初始化部分
//#include "lpdac_service.h"       // 需要调用 LPDAC 服务的初始化部分
#include "impedance_service.h"
#include <stdio.h>               // 用于打印调试信息
#include <string.h>              // 用于 memset
#include <Arduino.h>
extern "C" {
#include "ad5940.h"
}
/**
 * @brief 执行 AD5941 的统一初始化。
 * 配置时钟、FIFO、序列器内存、统一参考电压、
 * LPDAC/LP PA 基础状态，并加载温度测量序列。
 * @return 成功返回 AD5940ERR_OK，否则返回错误代码。
 */
AD5940Err App_AD5941_Init() {
  //实例化配置结构体
  AD5940Err err;
  CLKCfg_Type clk_cfg;      // 时钟配置结构体
  FIFOCfg_Type fifo_cfg;    // FIFO 配置结构体
  SEQCfg_Type seq_cfg;      //序列器配置结构体
  AFERefCfg_Type ref_cfg;   //参考电压配置结构体
  LPLoopCfg_Type lp_cfg;    //

  Serial.println("Starting Unified Initialization (from app_init.cpp)..."); // 开始初始化硬件平台
//
  /* 使用硬件复位引脚 (如果有连接) 复位 AD5940 */
  AD5940_HWReset();
  /* 初始化 AD5940 芯片，唤醒芯片并检查 SPI 通信是否正常 */
  AD5940_Initialize();

  /* 平台配置 */

  /* 步骤1. 配置时钟 */
  memset(&clk_cfg, 0, sizeof(clk_cfg)); //清空结构体成员
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;        // ADC 时钟分频：1分频
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;    // ADC 时钟源：高频振荡器 (HFOSC, 16MHz或32MHz)
  clk_cfg.SysClkDiv = SYSCLKDIV_1;        // 系统时钟分频：1分频
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;    // 系统时钟源：高频振荡器 (HFOSC)
  clk_cfg.HfOSC32MHzMode = bFALSE;        // HFOSC 32MHz 模式：关闭 (即使用 16MHz)
  clk_cfg.HFOSCEn = bTRUE;                // 使能高频振荡器
  clk_cfg.HFXTALEn = bFALSE;              // 禁能高频晶体 (HFXTAL)
  clk_cfg.LFOSCEn = bTRUE;                // 使能低频振荡器 (LFOSC, 32kHz)，用于看门狗和睡眠定时器
  AD5940_CLKCfg(&clk_cfg);                // 应用时钟配置

  /* 步骤2. 配置 FIFO 和序列器 */
  memset(&fifo_cfg, 0, sizeof(fifo_cfg));
  fifo_cfg.FIFOEn = bFALSE;               // 暂时禁能 FIFO (在配置期间)
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;      // FIFO 模式：设置为标准 FIFO 模式 (不是流模式)
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;       // FIFO 大小：设置为 4KB (总共6KB SRAM, 剩下 2KB 给序列器)

  fifo_cfg.FIFOThresh = 4;                // FIFO 阈值：设置为 4。
                                          // (DFT结果包含实部和虚部，每次测量(RCAL和Rz)产生2个复数，即4个uint32_t)
  AD5940_FIFOCfg(&fifo_cfg);              // 应用 FIFO 配置 (此时 FIFO 仍是禁能的)
  fifo_cfg.FIFOEn = bTRUE;                // 使能 FIFO
  AD5940_FIFOCfg(&fifo_cfg);              // 再次应用配置，正式使能 FIFO
  Serial.println("  FIFO configured.");

  /* 步骤3. 中断控制器 */
  // 配置中断控制器1 (INTC1)，使其能产生所有中断源的标志位，但不一定触发中断引脚
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT); // 清除所有中断标志
  // 配置中断控制器0 (INTC0)，使其在 "数据FIFO达到阈值" (DATAFIFOTHRESH) 时产生中断
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT); // 再次清除所有中断标志

  /* 步骤4. 配置序列器内存 */
  memset(&seq_cfg, 0, sizeof(seq_cfg));
  // ... (序列器内存配置代码) ...
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;  /* 2kB SRAM 用于序列器, 剩余 (4kB) 用于数据 FIFO */
  seq_cfg.SeqBreakEn = bFALSE; // 禁用断点
  seq_cfg.SeqIgnoreEn = bTRUE; // 启用"忽略"功能 (SEQ_IGNORE)
  seq_cfg.SeqCntCRCClr = bTRUE; // 清除 CRC 计数器
  seq_cfg.SeqEnable = bFALSE; // 暂时禁用序列器
  seq_cfg.SeqWrTimer = 0; // 写入定时器
  AD5940_SEQCfg(&seq_cfg); // 应用序列器配置
  Serial.println("  Sequencer memory allocated.");

  /*  步骤5. 统一配置参考电压 (使能 HP 和 LP) */
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);  
  memset(&ref_cfg, 0, sizeof(ref_cfg));
  // ... (参考电压配置代码) ...
  ref_cfg.HpBandgapEn = bTRUE;
  ref_cfg.Hp1V1BuffEn = bTRUE;
  ref_cfg.Hp1V8BuffEn = bTRUE;
  ref_cfg.Hp1V8ThemBuff = bFALSE; // 禁用 1.8V 热敏缓冲器
  ref_cfg.Hp1V8Ilimit = bFALSE;  // 禁用 1.8V 电流限制
  ref_cfg.Disc1V1Cap = bFALSE;   // 禁用 1.1V 缓冲器的大电容模式
  ref_cfg.Disc1V8Cap = bFALSE;   // 禁用 1.8V 缓冲器的大电容模式
  ref_cfg.LpBandgapEn = bTRUE;
  ref_cfg.LpRefBufEn = bTRUE;
  ref_cfg.Lp1V1BuffEn = bFALSE;  // 禁用低功耗 1.1V 缓冲器
  ref_cfg.Lp1V8BuffEn = bFALSE;  // 禁用低功耗 1.8V 缓冲器
  AD5940_REFCfgS(&ref_cfg);

  //调用温度服务的特定初始化 (配置模拟前端 + 加载序列)
 
  err = AD5940_Imp_Seq_Init(); // 这个函数现在只做温度相关的配置和加载
  if (err != AD5940ERR_OK) {
    Serial.println("  !!! Failed to configure/load impedance sequence!");
    return err; // 初始化失败，提前返回
  }
  Serial.println(" Impedance service initialized (Analog Path & Sequence).");
  return AD5940ERR_OK;
}

