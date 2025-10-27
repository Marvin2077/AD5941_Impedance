#include "impedance_service.h"
#include <stdio.h>
#include <string.h>
#include <Arduino.h>
extern "C" {
#include "ad5940.h"
}
/* LPDAC (低功耗DAC) 的默认分辨率 (使用2.5V内部参考) */
// 1个 LSB (最低有效位) 对应的电压值 (12位模式)
#define DAC12BITVOLT_1LSB   (2200.0f/4095)  //mV (注意：这里用的是 2.2V Vref, 不是 2.5V)
// 1个 LSB (最低有效位) 对应的电压值 (6位模式)。6位模式下 LSB 是 12位模式 LSB 的 64 倍
#define DAC6BITVOLT_1LSB    (DAC12BITVOLT_1LSB*64)  //mV

// 内部辅助函数的前向声明
static AD5940Err GenerateImpSequence(void);
static AD5940Err ConfigureAnalogPath_OnlyForImp(void); 

/* 应用程序配置结构体。由用户从模板指定。
  这些变量在整个应用程序中都可用。
  它包括序列器生成器的基本配置和应用相关参数。
*/
AD5940Err AppIMPCfg_init(){
  
  memset(&AppIMPCfg, 0, sizeof(AppIMPCfg));
  AppIMPCfg.bParaChanged = bFALSE;         // 参数是否被用户修改的标志
  AppIMPCfg.SeqStartAddr = 0;              // 序列器命令在 SRAM 中的起始地址
  AppIMPCfg.MaxSeqLen = 512;                 // 序列器命令的最大长度

  AppIMPCfg.SeqStartAddrCal = 0;           // (未使用) 校准序列的起始地址
  AppIMPCfg.MaxSeqLenCal = 0;              // (未使用) 校准序列的最大长度

  AppIMPCfg.ImpODR = 20.0;                 // 输出数据速率 (Output Data Rate)，单位 Hz。即每秒测量20次
  AppIMPCfg.NumOfData = -1;                // 需要采集的数据点数。-1 表示无限采集
  AppIMPCfg.SysClkFreq = 16000000.0;       // 系统时钟频率, 16MHz
  AppIMPCfg.WuptClkFreq = 32000.0;         // 唤醒定时器 (WUPT) 时钟频率, 32kHz
  AppIMPCfg.AdcClkFreq = 16000000.0;       // ADC 时钟频率, 16MHz
  AppIMPCfg.RcalVal = 1000.0;             // 校准电阻 (RCAL) 的精确值, 1k Ohm

  // 默认开关矩阵配置 (用于 Rz 测量)
  AppIMPCfg.DswitchSel = SWD_CE0;          // D 开关 (DAC输出) 连接到 CE0
  AppIMPCfg.PswitchSel = SWP_RE0;          // P 开关 (TIA 正输入) 连接到 CE0
  AppIMPCfg.NswitchSel = SWN_SE0;         // N 开关 (TIA 负输入) 连接到 AIN1
  AppIMPCfg.TswitchSel = SWT_SE0LOAD;         // T 开关 (TIA 反馈) 连接到 AIN1

  AppIMPCfg.PwrMod = AFEPWR_HP;            // AFE 电源模式：HP (High Power) 高性能模式

  AppIMPCfg.HstiaRtiaSel = HSTIARTIA_5K;   // 高速 TIA (HSTIA) 的反馈电阻 (Rtia) 选择：5k Ohm
  AppIMPCfg.ExcitBufGain = EXCITBUFGAIN_2; // 激励缓冲器增益：2 (即 0.2 * 2 = 0.4) (注：应为 x0.2)
  AppIMPCfg.HsDacGain = HSDACGAIN_1;       // 高速 DAC (HSDAC) 增益：x1
  AppIMPCfg.HsDacUpdateRate = 7;           // HSDAC 更新速率设置 (影响激励波形质量)
  AppIMPCfg.DacVoltPP = 800.0;             // DAC 输出峰峰值电压, 800mV
  AppIMPCfg.BiasVolt = -0.0f;              // 施加的直流偏置电压

  AppIMPCfg.SinFreq = 100000.0;            // 正弦波激励频率: 100kHz

  AppIMPCfg.DftNum = DFTNUM_16384;         // DFT (数字傅里叶变换) 点数：16384 点
  AppIMPCfg.DftSrc = DFTSRC_SINC3;         // DFT 数据来源：Sinc3 滤波器
  AppIMPCfg.HanWinEn = bTRUE;              // 使能汉宁窗 (Hanning Window)

  AppIMPCfg.AdcPgaGain = ADCPGA_1;         // ADC 前端 PGA 增益：x1
  AppIMPCfg.ADCSinc3Osr = ADCSINC3OSR_2;   // Sinc3 滤波器过采样率(OSR)：2
  AppIMPCfg.ADCSinc2Osr = ADCSINC2OSR_22;  // Sinc2 滤波器过采样率(OSR)：22

  AppIMPCfg.ADCAvgNum = ADCAVGNUM_16;      // ADC 平均次数：16次 (注：这似乎是Sinc2的配置，而非独立平均)

  AppIMPCfg.SweepCfg.SweepEn = bTRUE;             // 使能扫频
  AppIMPCfg.SweepCfg.SweepStart = 100;           // 扫频起始频率: 1kHz
  AppIMPCfg.SweepCfg.SweepStop = 100000.0;        // 扫频停止频率: 100kHz
  AppIMPCfg.SweepCfg.SweepPoints = 101;           // 扫频点数: 101
  AppIMPCfg.SweepCfg.SweepLog = bTRUE;           // 扫频模式：bTRUE = 对数扫频 (Logarithmic), bFALSE = 线性扫频
  AppIMPCfg.SweepCfg.SweepIndex = 0;              // 当前扫频点索引

  AppIMPCfg.FifoThresh = 4;                // FIFO 阈值。4个字 (RCAL实部, RCAL虚部, Rz实部, Rz虚部)
  AppIMPCfg.IMPInited = bFALSE;            // 阻抗应用是否已初始化标志
  AppIMPCfg.StopRequired = bFALSE;         // 是否请求停止测量的标志

  return AD5940ERR_OK;
}

/* * @brief  AFE 初始化配置
 * @details 此函数配置 AFE 的所有模拟模块 (参考, TIA, DAC, ADC, 滤波器等)
 * @return AD5940Err: 错误码
 */
  AD5940Err ConfigureAnalogPath_OnlyForImp(void){
  AD5940Err error = AD5940ERR_OK;
  HSLoopCfg_Type HsLoopCfg; // AFE 高速环路配置结构体
  DSPCfg_Type dsp_cfg; // AFE 数字信号处理 (DSP) 配置结构体
  AFERefCfg_Type ref_cfg;   //参考电压配置结构体
  FIFOCfg_Type fifo_cfg; // FIFO 配置结构体
  float sin_freq; // 正弦波频率
  
    /* 0. 修改FIFO数据源*/
  //------Impedance测试时数据FIFO数据来源为DFT--------//
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;         // FIFO 数据源：设置为 DFT (数字傅里叶变换) 结果 
  AD5940_FIFOCfg(&fifo_cfg); // 应用 FIFO 配置
  
   /* 1. 低功耗参考控制 - 关闭它们以省电*/
  //參考電壓已經于app_init配置，此處為特殊控制
  if(AppIMPCfg.BiasVolt != 0.0f)    /* 仅当需要施加直流偏置电压时才开启 */
  {
    ref_cfg.LpBandgapEn = bTRUE; // 使能低功耗带隙基准
    ref_cfg.LpRefBufEn = bTRUE;  // 使能低功耗参考缓冲器
  }
  else
  {
    ref_cfg.LpBandgapEn = bFALSE;
    ref_cfg.LpRefBufEn = bFALSE;
  }
  ref_cfg.LpRefBoostEn = bFALSE; // 禁用低功耗参考的 Boost 模式
  AD5940_REFCfgS(&ref_cfg); // 应用参考配置

    /* 2. 配置高速 (HS) 环路 */
  // 2a. 高速 DAC (HSDAC) 配置
  HsLoopCfg.HsDacCfg.ExcitBufGain = AppIMPCfg.ExcitBufGain; // 激励缓冲器增益
  HsLoopCfg.HsDacCfg.HsDacGain = AppIMPCfg.HsDacGain;       // HSDAC 增益
  HsLoopCfg.HsDacCfg.HsDacUpdateRate = AppIMPCfg.HsDacUpdateRate; // HSDAC 更新率

  // 2b. 高速 TIA (HSTIA) 配置
  HsLoopCfg.HsTiaCfg.DiodeClose = bFALSE; // 不闭合二极管 (用于保护)
	if(AppIMPCfg.BiasVolt != 0.0f)    /* 如果有直流偏置 */
		HsLoopCfg.HsTiaCfg.HstiaBias = HSTIABIAS_VZERO0; // TIA 偏置设为 VZERO0 (由 LPDAC 控制)
	else
		HsLoopCfg.HsTiaCfg.HstiaBias = HSTIABIAS_1P1; // TIA 偏置设为 1.1V
  HsLoopCfg.HsTiaCfg.HstiaCtia = 31; /* TIA 跨阻电容: 31pF + 2pF (内部固定) */
  HsLoopCfg.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN; // TIA 去饱和负载电阻：开路
  HsLoopCfg.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN; // TIA 去饱和反馈电阻：开路
  HsLoopCfg.HsTiaCfg.HstiaRtiaSel = AppIMPCfg.HstiaRtiaSel; // TIA 反馈电阻 (增益)

  // 2c. 开关矩阵 (SWMat) 配置 - 对应 Rz 测量
  HsLoopCfg.SWMatCfg.Dswitch = AppIMPCfg.DswitchSel;
  HsLoopCfg.SWMatCfg.Pswitch = AppIMPCfg.PswitchSel;
  HsLoopCfg.SWMatCfg.Nswitch = AppIMPCfg.NswitchSel;
  // T 开关：连接到 TIA 反馈 (SWT_TRTIA) 并 加上 用户选择的 T 开关 (AppIMPCfg.TswitchSel)
  HsLoopCfg.SWMatCfg.Tswitch = SWT_TRTIA|AppIMPCfg.TswitchSel;

  // 2d. 波形发生器 (WG) 配置
  HsLoopCfg.WgCfg.WgType = WGTYPE_SIN; // 波形类型：正弦波
  HsLoopCfg.WgCfg.GainCalEn = bTRUE;   // 使能增益校准
  HsLoopCfg.WgCfg.OffsetCalEn = bTRUE; // 使能失调校准

  if(AppIMPCfg.SweepCfg.SweepEn == bTRUE) // 如果使能了扫频
  {
    // 初始化扫频
    AppIMPCfg.FreqofData = AppIMPCfg.SweepCfg.SweepStart; // 记录当前数据频率
    AppIMPCfg.SweepCurrFreq = AppIMPCfg.SweepCfg.SweepStart; // 设置当前扫频频率
    // 计算下一个扫频频率点，存入 SweepNextFreq
    AD5940_SweepNext(&AppIMPCfg.SweepCfg, &AppIMPCfg.SweepNextFreq);
    sin_freq = AppIMPCfg.SweepCurrFreq; // 使用起始频率
  }
  else // 单频模式
  {
    sin_freq = AppIMPCfg.SinFreq; // 使用设定的单频
    AppIMPCfg.FreqofData = sin_freq;
  }
  // 计算波形发生器需要的频率字
  HsLoopCfg.WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(sin_freq, AppIMPCfg.SysClkFreq);
  // 计算波形发生器需要的幅度字 (基于 800mV 峰峰值 和 11位DAC)
  HsLoopCfg.WgCfg.SinCfg.SinAmplitudeWord = (uint32_t)(AppIMPCfg.DacVoltPP/800.0f*2047 + 0.5f);
  HsLoopCfg.WgCfg.SinCfg.SinOffsetWord = 0; // 直流失调为 0
  HsLoopCfg.WgCfg.SinCfg.SinPhaseWord = 0; // 相位为 0

  AD5940_HSLoopCfgS(&HsLoopCfg); // 应用高速环路配置

  /* 3. 配置低功耗 DAC (LPDAC) - 仅用于产生直流偏置 */
  if(AppIMPCfg.BiasVolt != 0.0f)    /* 如果需要直流偏置 */
  {
    LPDACCfg_Type lpdac_cfg;

    lpdac_cfg.LpdacSel = LPDAC0; // 选择 LPDAC0
    lpdac_cfg.LpDacVbiasMux = LPDACVBIAS_12BIT; // Vbias (12位) 用于调节偏置电压
    lpdac_cfg.LpDacVzeroMux = LPDACVZERO_6BIT;  // Vzero (6位) 用于 TIA 的 Vzero
    lpdac_cfg.DacData6Bit = 0x40>>1;            /* 将 Vzero (6位) 设置到中间范围 (0x20) */

    // 限制偏置电压在 LPDAC 的范围内 (约 -1100mV 到 +1100mV)
    if(AppIMPCfg.BiasVolt<-1100.0f) AppIMPCfg.BiasVolt = -1100.0f + DAC12BITVOLT_1LSB;
    if(AppIMPCfg.BiasVolt> 1100.0f) AppIMPCfg.BiasVolt = 1100.0f - DAC12BITVOLT_1LSB;
    // Vbias = (BiasVolt + 1100.0f) / LSB_Volt
    lpdac_cfg.DacData12Bit = (uint32_t)((AppIMPCfg.BiasVolt + 1100.0f)/DAC12BITVOLT_1LSB);

    lpdac_cfg.DataRst = bFALSE;      /* 不复位数据寄存器 */
    // 配置 LPDAC 开关，将 Vbias 和 Vzero 连接到 AFE 内部和外部引脚
    lpdac_cfg.LpDacSW = LPDACSW_VBIAS2LPPA|LPDACSW_VBIAS2PIN|LPDACSW_VZERO2LPTIA|LPDACSW_VZERO2PIN|LPDACSW_VZERO2HSTIA;
    lpdac_cfg.LpDacRef = LPDACREF_2P5; // LPDAC 参考电压：2.5V (内部)
    lpdac_cfg.LpDacSrc = LPDACSRC_MMR; // LPDAC 数据来源：MMR (寄存器)
    lpdac_cfg.PowerEn = bTRUE;         // 使能 LPDAC 电源
    AD5940_LPDACCfgS(&lpdac_cfg); // 应用 LPDAC 配置
  }

  /* 4. 配置 DSP (数字信号处理) 模块 */
  // 4a. ADC 基础配置
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_HSTIA_N; // ADC 负输入：HSTIA 的 N 端
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_HSTIA_P; // ADC 正输入：HSTIA 的 P 端
  dsp_cfg.ADCBaseCfg.ADCPga = AppIMPCfg.AdcPgaGain; // ADC PGA 增益

  memset(&dsp_cfg.ADCDigCompCfg, 0, sizeof(dsp_cfg.ADCDigCompCfg)); // ADC 数字补偿 (清零)

  // 4b. ADC 滤波器配置
  dsp_cfg.ADCFilterCfg.ADCAvgNum = AppIMPCfg.ADCAvgNum; // 平均滤波器 (Sinc2)
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;  /* 告诉滤波器 ADC 的时钟速率 (800kHz or 1.6MHz) */
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = AppIMPCfg.ADCSinc2Osr; // Sinc2 OSR
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppIMPCfg.ADCSinc3Osr; // Sinc3 OSR
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE; // 使能 Sinc2 之后的 50/60Hz 陷波器
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE; // 旁路 Sinc3 滤波器 (注：此处为 bFALSE，表示 *不* 旁路)
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE; // 使能 Sinc2 和 陷波器

  // 4c. DFT 配置
  dsp_cfg.DftCfg.DftNum = AppIMPCfg.DftNum; // DFT 点数
  dsp_cfg.DftCfg.DftSrc = AppIMPCfg.DftSrc; // DFT 数据源
  dsp_cfg.DftCfg.HanWinEn = AppIMPCfg.HanWinEn; // 汉宁窗使能

  memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg)); // 统计功能 (清零)
  AD5940_DSPCfgS(&dsp_cfg); // 应用 DSP 配置

  /* 5. 最终使能所有需要的 AFE 模块 */
  /* 它们会在休眠模式下自动关闭以省电 */
  if(AppIMPCfg.BiasVolt == 0.0f) // 如果没有直流偏置
    AD5940_AFECtrlS(AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|AFECTRL_WG|AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|AFECTRL_SINC2NOTCH, bTRUE);
  else // 如果有直流偏置，额外使能 DCBUFPWR (用于 LPDAC)
    AD5940_AFECtrlS(AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|AFECTRL_WG|AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|AFECTRL_SINC2NOTCH|AFECTRL_DCBUFPWR, bTRUE);

return AD5940ERR_OK;
}

/* * @brief 生成阻抗测量序列
 * @details 此函数将向AD5941写入SEQID为1的测量序列，包括利用RCAL校准与测量Rz
 * @return AD5940Err: 错误码
 */
static AD5940Err GenerateImpSequence(void)
{
  AD5940Err error = AD5940ERR_OK;
  const uint32_t *pSeqCmd; // 指向生成的序列命令缓冲区的指针
  uint32_t SeqLen; // 生成的序列命令长度

  uint32_t WaitClks; // 等待 DFT 完成所需的时钟周期数
  SWMatrixCfg_Type sw_cfg; // 开关矩阵配置
  ClksCalInfo_Type clks_cal; // 时钟计算辅助结构体

  /* 1. 计算 DFT 测量所需的等待时钟数 */
  clks_cal.DataType = DATATYPE_DFT; // 数据类型为 DFT
  clks_cal.DftSrc = AppIMPCfg.DftSrc; // DFT 数据源
  clks_cal.DataCount = 1L<<(AppIMPCfg.DftNum+2); /* DFT 点数 = 2^(DFTNUMBER+2) (注：这是 DFT 引擎的配置) */
  clks_cal.ADCSinc2Osr = AppIMPCfg.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = AppIMPCfg.ADCSinc3Osr;
  clks_cal.ADCAvgNum = AppIMPCfg.ADCAvgNum;
  // 系统时钟与 ADC 时钟的比率
  clks_cal.RatioSys2AdcClk = AppIMPCfg.SysClkFreq/AppIMPCfg.AdcClkFreq;
  // 计算需要等待的系统时钟周期数
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  /* 2. 开始生成序列 */
  AD5940_SEQGenCtrl(bTRUE);

  AD5940_SEQGpioCtrlS(AGPIO_Pin2); /* 拉高 GPIO Pin2 (用于调试或同步) */
  AD5940_SEQGenInsert(SEQ_WAIT(16*250));  /* @todo 等待 250us? (16MHz/16 = 1us per tick) */

  /* 3. --- 测量 RCAL --- */
  // 3a. 配置开关矩阵连接到 RCAL (RCAL0 和 RCAL1)
  sw_cfg.Dswitch = SWD_RCAL0;
  sw_cfg.Pswitch = SWP_RCAL0;
  sw_cfg.Nswitch = SWN_RCAL1;
  sw_cfg.Tswitch = SWT_RCAL1|SWT_TRTIA; // RCAL1 并连接到 TIA 反馈
  AD5940_SWMatrixCfgS(&sw_cfg);

  // 3b. 开启 AFE 模块 (与初始化序列中开启的模块一致)
	AD5940_AFECtrlS(AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
                AFECTRL_WG|AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|\
                AFECTRL_SINC2NOTCH, bTRUE);

  // 3c. 开启波形发生器 (WG) 和 ADC 电源
  AD5940_AFECtrlS(AFECTRL_WG|AFECTRL_ADCPWR, bTRUE);
  // 等待信号建立 (DFT_WAIT)
  AD5940_SEQGenInsert(SEQ_WAIT(16*10)); // 等待 10us

  // 3d. 启动 ADC 转换 和 DFT 计算
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);
  // 等待 DFT 完成 (等待之前计算的 WaitClks)
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));

  // 3e. 停止 ADC, DFT 和 WG
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_WG, bFALSE);

  /* 4. --- 测量 Rz (未知阻抗) --- */
  // 4a. 配置开关矩阵连接到外部 Rz (使用 AppIMPCfg 中的设置)
  sw_cfg.Dswitch = AppIMPCfg.DswitchSel;
  sw_cfg.Pswitch = AppIMPCfg.PswitchSel;
  sw_cfg.Nswitch = AppIMPCfg.NswitchSel;
  sw_cfg.Tswitch = SWT_TRTIA|AppIMPCfg.TswitchSel;
  AD5940_SWMatrixCfgS(&sw_cfg);

  // 4b. 开启 WG 和 ADC 电源
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_WG, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*10));  // 等待信号建立

  // 4c. 启动 ADC 转换 和 DFT 计算
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));  /* 等待 DFT 完成 */

  // 4d. 停止 ADC, DFT 和 WG
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_WG|AFECTRL_ADCPWR, bFALSE);

  // 4e. 关闭 AFE 模块 (与 3b 对应)
  AD5940_AFECtrlS(AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
                AFECTRL_WG|AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|\
                AFECTRL_SINC2NOTCH, bFALSE);

  AD5940_SEQGpioCtrlS(0); /* 拉低 GPIO Pin2 */

  /* 5. --- 进入休眠 --- */
  AD5940_EnterSleepS(); /* 序列执行完毕后，进入休眠 (Hibernate) 模式，Trigger函数唤醒 */

  /* 序列结束 */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen); // 获取生成的序列
  AD5940_SEQGenCtrl(bFALSE); /* 停止序列器生成器 */

  if(error == AD5940ERR_OK)
  {
    // 保存测量序列的信息
    AppIMPCfg.MeasureSeqInfo.SeqId = SEQID_IMP; // 使用序列 ID 0
    // 存储地址：紧跟在初始化序列 (InitSeqInfo) 之后
    AppIMPCfg.MeasureSeqInfo.SeqRamAddr = AppIMPCfg.InitSeqInfo.SeqRamAddr + AppIMPCfg.InitSeqInfo.SeqLen ;
    AppIMPCfg.MeasureSeqInfo.pSeqCmd = pSeqCmd;
    AppIMPCfg.MeasureSeqInfo.SeqLen = SeqLen;
    /* 将命令写入 AD5940 的 SRAM */
    AD5940_SEQCmdWrite(AppIMPCfg.MeasureSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* 出错 */
  return AD5940ERR_OK;
}

AD5940Err AD5940_Imp_Seq_Init(void)
{
  AD5940Err error_code;

  // 1. 配置模拟前端通路 
  ConfigureAnalogPath_OnlyForImp();

  // 2. 生成阻抗测量序列并加载到 SRAM
  error_code = GenerateImpSequence();
  if (error_code != AD5940ERR_OK) return error_code;

  // 4. 使能序列器模块，使其可以被触发执行
  AD5940_SEQCtrlS(bTRUE);

  return AD5940ERR_OK;

}

/* 公共函数: 触发温度测量序列 */
AD5940Err AD5940_IMP_Seq_Trigger(void)
{
  /* 通过读寄存器唤醒 AFE - 如果序列从睡眠状态开始，则必须执行 */
  if (AD5940_WakeUp(10) > 10) {
    printf("Wakeup failed in trigger!\n");
    return AD5940ERR_WAKEUP; // 唤醒失败
  }

  /* 通过写 MMR (Memory Mapped Register) 触发序列 */
  AD5940_SEQMmrTrig(SEQID_IMP);

  return AD5940ERR_OK;

}

/**
 * @brief  处理 FIFO 数据
 * @details 此函数在 ISR 中调用，
 * 1. 将 FIFO 读出的原始 18 位 DFT 数据 (实部/虚部) 转换为 32 位有符号整数。
 * 2. 使用 RCAL 和 Rz 的 DFT 结果，计算 Rz 的阻抗幅值和相位。
 * 3. (扫频) 更新扫频状态，计算 *再下一次* 的频率点。
 * @param pData: [输入] 指向原始 FIFO 数据的缓冲区 (int32_t 格式)
 * [输出] 指向处理后的阻抗数据 (fImpPol_Type 格式)
 * @param pDataCount: [输入] 原始 FIFO 数据的数量 (uint32_t 个数)
 * [输出] 处理后的阻抗数据点数量 (fImpPol_Type 个数)
 * @return int32_t: 错误码
 */
int32_t AppIMPDataProcess(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t DataCount = *pDataCount; // 原始数据个数 (N个 uint32_t)
  uint32_t ImpResCount = DataCount/4; // 阻抗结果个数 (N/4 个)

  fImpPol_Type * const pOut = (fImpPol_Type*)pData; // 输出缓冲区 (覆盖原始数据)
  iImpCar_Type * pSrcData = (iImpCar_Type*)pData; // 输入数据源 (笛卡尔坐标, 实部/虚部)

  *pDataCount = 0; // 先清零输出计数

  DataCount = (DataCount/4)*4; // 确保数据量是4的倍数

  /* 1. 将 DFT 结果 (18位有符号) 转换为 int32_t */
  for(uint32_t i=0; i<DataCount; i++)
  {
    pData[i] &= 0x3ffff; /* 屏蔽掉高位 (只保留 18 位) */
    if(pData[i]&(1L<<17)) /* 检查第17位 (符号位) */
    {
      pData[i] |= 0xfffc0000; /* 符号位扩展，变为 32 位有符号整数 */
    }
  }

  /* 2. 计算阻抗 */
  for(uint32_t i=0; i<ImpResCount; i++)
  {
    iImpCar_Type *pDftRcal, *pDftRz;

    pDftRcal = pSrcData++; // 指向 RCAL 的 (实部, 虚部)
    pDftRz = pSrcData++;   // 指向 Rz 的 (实部, 虚部)
    float RzMag,RzPhase;
    float RcalMag, RcalPhase;

    // 计算 RCAL 的幅值和相位
    RcalMag = sqrt((float)pDftRcal->Real*pDftRcal->Real+(float)pDftRcal->Image*pDftRcal->Image);
    RcalPhase = atan2(-pDftRcal->Image,pDftRcal->Real); // 相位 (atan2(y, x))

    // 计算 Rz 的幅值和相位
    RzMag = sqrt((float)pDftRz->Real*pDftRz->Real+(float)pDftRz->Image*pDftRz->Image);
    RzPhase = atan2(-pDftRz->Image,pDftRz->Real);

    // 阻抗计算公式 (2点校准): Z_rz = (V_rz / V_rcal) * Z_rcal
    RzMag = RcalMag/RzMag*AppIMPCfg.RcalVal; // 幅值
    RzPhase = RcalPhase - RzPhase; // 相位 (V_rcal的相位 - V_rz的相位)

    // 将结果存回缓冲区
    pOut[i].Magnitude = RzMag;
    pOut[i].Phase = RzPhase;
  }

  *pDataCount = ImpResCount; // 更新 pDataCount 为已处理的 *阻抗点* 数量

  /* 3. 处理扫频逻辑 */
  // 记录刚刚测量完成的频率点
  AppIMPCfg.FreqofData = AppIMPCfg.SweepCurrFreq;

  if(AppIMPCfg.SweepCfg.SweepEn == bTRUE) // 如果在扫频
  {
    AppIMPCfg.FreqofData = AppIMPCfg.SweepCurrFreq; // 存储刚测量的频率

    // 准备下一次测量
    // 将 AppIMPRegModify 中已设置到硬件的 SweepNextFreq 变为 "当前频率"
    AppIMPCfg.SweepCurrFreq = AppIMPCfg.SweepNextFreq;

    // 计算 *再下一次* 测量要用的频率，存入 SweepNextFreq
    // 这个值将在 *下一次* ISR 的 AppIMPRegModify 中被写入硬件
    AD5940_SweepNext(&AppIMPCfg.SweepCfg, &AppIMPCfg.SweepNextFreq);
  }

  return 0;
}

/**
 * @brief  阻抗数据处理函数
 * @details 由 MCU 的状态机调用。
 * @param pBuff: 指向数据缓冲区的指针 (即 AD5940Main.c 中的 AppBuff)
 * @param pCount: [输入] 缓冲区 pBuff 的最大容量 (in uint32_t)
 * [输出] 实际处理后的数据点数 (阻抗点的数量)
 * @return int32_t: 错误码
 */
int32_t  AD5940_Imp_Seq_ReadResult(void *pBuff, uint32_t *pCount)
{
  
  uint32_t BuffCount; // 缓冲区总容量
  uint32_t FifoCnt; // FIFO 中实际的数据量
  BuffCount = *pCount;

  *pCount = 0; // 默认为0

  if(AD5940_WakeUp(10) > 10)  /* 尝试唤醒 AFE */
    return AD5940ERR_WAKEUP;  /* 唤醒失败 */

  // 锁定睡眠密钥。防止序列器中的 EnterSleepS 命令在 ISR 处理期间让 AFE 意外休眠
  AD5940_SleepKeyCtrlS(SLPKEY_LOCK);

  /* 检查是否是 "FIFO 达到阈值" 中断 */
  if(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bTRUE)
  {
    /* FIFO 中应该有数据 (预期是4个) */
    FifoCnt = (AD5940_FIFOGetCnt()/4)*4; // 获取 FIFO 中的数据量，并确保是4的倍数

    if(FifoCnt > BuffCount) // 检查缓冲区是否足够
    {
      ///@todo 缓冲区空间不足
    }

    /* 1. 从 FIFO 读取数据到 pBuff */
    AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
    /* 2. 清除 FIFO 阈值中断标志 */
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);

    /* 3. 在 AFE 保持激活状态时，修改寄存器 (例如为下一次扫频设置频率) */
    AppIMPRegModify((int32_t*)pBuff, &FifoCnt);

    /* 4. 解锁睡眠密钥，允许 AFE 在序列结束时进入休眠 */
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);

    /* 5. 处理刚读取的数据 (计算阻抗, 更新扫频状态) */
    AppIMPDataProcess((int32_t*)pBuff,&FifoCnt);

    *pCount = FifoCnt; // 返回已处理的 *阻抗点* 数量
    return true;
  }

  return false;
}

/**
 * @brief  获取应用程序配置结构体的指针。
 * @details 该函数供上层控制器调用，以便在运行时读取应用参数。
 * @param pCfg: 一个指向 (AppIMPCfg_Type*) 类型指针的指针。函数会把 AppIMPCfg 的地址写入 *pCfg。
 * @return int32_t: 成功返回 AD5940ERR_OK, 失败返回 AD5940ERR_PARA。
 */
int32_t AppIMPGetCfg(void *pCfg)
{
  if(pCfg)
  {
    *(AppIMPCfg_Type**)pCfg = &AppIMPCfg;
    return AD5940ERR_OK;
  }
  return AD5940ERR_PARA;
}

/**
 * @brief  阻抗应用控制函数
 * @param Command: 控制命令 (如 START, STOP, SHUTDOWN)
 * @param pPara: 伴随命令的参数 (如 GETFREQ 时用于返回频率值)
 * @return int32_t: 错误码
 */
int32_t AppIMPCtrl(uint32_t Command, void *pPara)
{

  switch (Command)
  {
    case IMPCTRL_START: // 开始测量
    {
      WUPTCfg_Type wupt_cfg; // 唤醒定时器 (WUPT) 配置结构体

      if(AD5940_WakeUp(10) > 10)  // 尝试唤醒 AFE, 最多10次
        return AD5940ERR_WAKEUP;  // 唤醒失败
      if(AppIMPCfg.IMPInited == bFALSE) // 检查是否已初始化
        return AD5940ERR_APPERROR;  // 未初始化，返回应用错误

      /* 启动测量 */
      wupt_cfg.WuptEn = bTRUE; // 使能 WUPT
      // WUPT 结束后 (即唤醒后) 自动执行的序列：序列A (测量序列)
      wupt_cfg.WuptEndSeq = WUPTENDSEQ_A;
      wupt_cfg.WuptOrder[0] = SEQID_IMP; // 序列A -> SEQID_IMP (即 AppIMPSeqMeasureGen 生成的序列)

      // 设置 WUPT 睡眠和唤醒时间
      wupt_cfg.SeqxSleepTime[SEQID_IMP] = 4; // 序列器启动前的短暂延时 (4个 WUPT 时钟周期)
      // 总周期 = 1 / ImpODR。睡眠时间 = 总周期 - 唤醒时间(4)
      // (uint32_t)(AppIMPCfg.WuptClkFreq / AppIMPCfg.ImpODR) 是一个周期的总 WUPT 时钟数
      wupt_cfg.SeqxWakeupTime[SEQID_IMP] = (uint32_t)(AppIMPCfg.WuptClkFreq/AppIMPCfg.ImpODR)-4;

      AD5940_WUPTCfg(&wupt_cfg); // 应用 WUPT 配置

      AppIMPCfg.FifoDataCount = 0;  /* 重启 FIFO 数据计数 */
      break;
    }
    case IMPCTRL_STOPNOW: // 立即停止
    {
      if(AD5940_WakeUp(10) > 10)  // 尝试唤醒 AFE
        return AD5940ERR_WAKEUP;  // 唤醒失败
      /* 立即停止 WUPT */
      AD5940_WUPTCtrl(bFALSE); // 禁能 WUPT
      /* 这里有可能会失败，因为序列器可能在 AFE 刚唤醒后又立即让它进入了休眠。
        使用 STOPSYNC (同步停止) 是更好的选择。
      */
      AD5940_WUPTCtrl(bFALSE); // 再次尝试禁能 WUPT
      break;
    }
    case IMPCTRL_STOPSYNC: // 同步停止 (在下一次 ISR 中停止)
    {
      AppIMPCfg.StopRequired = bTRUE; // 设置停止请求标志
      break;
    }
    case IMPCTRL_GETFREQ: // 获取当前频率
      {
        if(pPara == 0) // 检查参数指针是否为空
          return AD5940ERR_PARA;
        if(AppIMPCfg.SweepCfg.SweepEn == bTRUE) // 如果在扫频模式
          *(float*)pPara = AppIMPCfg.FreqofData; // 返回上一个已测量点的频率
        else // 如果在单频模式
          *(float*)pPara = AppIMPCfg.SinFreq; // 返回设定的单频频率
      }
    break;
    case IMPCTRL_SHUTDOWN: // 关断
    {
      AppIMPCtrl(IMPCTRL_STOPNOW, 0);  /* 首先立即停止测量 (如果正在运行) */

      /* 关闭那些在休眠模式下不会自动关闭的 LP (低功耗) 环路相关模块 */
      AFERefCfg_Type aferef_cfg;
      LPLoopCfg_Type lp_loop;

      // 清空参考配置结构体并应用 (关闭所有参考)
      memset(&aferef_cfg, 0, sizeof(aferef_cfg));
      AD5940_REFCfgS(&aferef_cfg);
      // 清空低功耗环路配置结构体并应用 (关闭 LP 环路)
      memset(&lp_loop, 0, sizeof(lp_loop));
      AD5940_LPLoopCfgS(&lp_loop);

      AD5940_EnterSleepS();  /* 进入休眠 (Hibernate) 模式 */
    }
    break;
    default:
    break;
  }
  return AD5940ERR_OK;
}

/* * @brief  (代码片段) 获取当前频率
 * @return float: 当前测量频率
 */
float AppIMPGetCurrFreq(void)
{
  if(AppIMPCfg.SweepCfg.SweepEn == bTRUE) // 如果在扫频
    return AppIMPCfg.FreqofData; // 返回上一个已测量点的频率
  else // 如果单频
    return AppIMPCfg.SinFreq; // 返回设定的单频频率
}

/**
 * @brief  在 AFE 唤醒时修改测量参数？
 * @details 此函数在 ISR 从 FIFO 读数 *之后*，AFE 进入休眠 *之前* 调用。
 * 主要用于扫频模式下更新下一次测量的频率。
 * @param pData: (未使用) 指向 FIFO 数据的指针
 * @param pDataCount: (未使用) FIFO 数据的数量
 * @return int32_t: 错误码
 */
int32_t AppIMPRegModify(int32_t * const pData, uint32_t *pDataCount)
{
  // 检查是否达到了所需的数据点数 (如果 NumOfData > 0)
  if(AppIMPCfg.NumOfData > 0)
  {
    AppIMPCfg.FifoDataCount += *pDataCount/4; // 每次中断有 pDataCount/4 个阻抗点
    if(AppIMPCfg.FifoDataCount >= AppIMPCfg.NumOfData)
    {
      AD5940_WUPTCtrl(bFALSE); // 达到点数，停止 WUPT
      return AD5940ERR_OK;
    }
  }

  // 检查是否收到了同步停止请求
  if(AppIMPCfg.StopRequired == bTRUE)
  {
    AD5940_WUPTCtrl(bFALSE); // 停止 WUPT
    return AD5940ERR_OK;
  }

  // 如果在扫频模式
  if(AppIMPCfg.SweepCfg.SweepEn)
  {
    /* 将 *下一次* 测量要用的频率 (SweepNextFreq) 写入波形发生器寄存器 */
    /* SweepNextFreq 是在 AppIMPDataProcess 中计算的 */
    AD5940_WGFreqCtrlS(AppIMPCfg.SweepNextFreq, AppIMPCfg.SysClkFreq);
  }
  return AD5940ERR_OK;
}

int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;

  fImpPol_Type *pImp = (fImpPol_Type*)pData;
  AppIMPCtrl(IMPCTRL_GETFREQ, &freq);

  Serial.printf("Freq:%.2f ", freq);
  /*Process data*/
  for(int i=0;i<DataCount;i++)
  {
    Serial.printf("RzMag: %f Ohm , RzPhase: %f \n",pImp[i].Magnitude,pImp[i].Phase*180/MATH_PI);
  }
  return 0;
}


