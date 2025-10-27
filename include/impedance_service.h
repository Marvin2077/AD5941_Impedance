#pragma once
#include <stdint.h>
#include <Arduino.h> 

extern "C" {
#include "ad5940.h" // 引入 AD5940/1 库头文件
} // extern "C"
/* -------- 基于序列器的温度读取 -------- */

/* 定义我们将用于温度测量的序列ID (Sequence ID) */
#define SEQID_IMP SEQID_1
/* 定义 FIFO 阈值 - 对于单次读取，设置为1 */
#define IMP_FIFO_THRESH 4


/**
 * @brief 初始化 AD5941 用于基于序列器的内部温度测量。
 * 配置模拟路径，生成并加载序列，配置 FIFO 和内部标志位。
 * 在 setup() 函数中调用一次。
 * @return 成功返回 AD5940ERR_OK，否则返回错误代码。
 */
AD5940Err AD5940_Imp_Seq_Init(void);


/**
 * @brief 触发预加载的温度测量序列 (SEQID_TEMP)。
 * 当你想开始一次新的测量时调用此函数。
 * @return 成功返回 AD5940ERR_OK，否则返回错误代码 (例如，唤醒失败)。
 */
AD5940Err AD5940_Imp_Seq_Trigger(void);

/**
 * @brief 轮询 AD5941 以检查 FIFO 中是否有温度结果准备就绪。
 * 如果准备就绪，则读取结果，清除内部标志位，计算温度，并返回 true。
 * 在触发测量后，在你的主循环 (loop) 中重复调用此函数。
 * @param[out] temp_c 指向浮点数的指针，用于存储计算出的摄氏温度。只有当函数返回 true 时，该值才有效。
 * @return 如果成功读取结果则返回 true，如果数据尚未准备好则返回 false。
 */
int32_t  AD5940_Imp_Seq_ReadResult(void *pBuff, uint32_t *pCount);

/**
 * @brief 修改阻抗测量应用（AppIMP）的测量参数。
 * @param[out] pCfg 指向用户提供的配置结构体缓存，类型应为 AppIMPCfg_Type*。
 *                  函数会将内部维护的当前配置拷贝到该结构体中。
 * @return int32_t  返回状态码：
 *                  - AD5940ERR_OK        成功，将当前配置写入 *pCfg；
 *                  - AD5940ERR_PARA      入参为空（pCfg == NULL）或类型不匹配；
 */
int32_t AppIMPGetCfg(void *pCfg);

int32_t AppIMPGetCfg(void *pCfg);
int32_t AppIMPISR(void *pBuff, uint32_t *pCount);
int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount);
int32_t AppIMPRegModify(int32_t * const pData, uint32_t *pDataCount);

/* 控制命令（给 AppIMP 控制函数使用） */
#define IMPCTRL_START          0   /* 立即启动测量 */
#define IMPCTRL_STOPNOW        1   /* 立刻停止测量（异步立即停止） */
#define IMPCTRL_STOPSYNC       2   /* 同步停止（通常在下个安全点停止） */
#define IMPCTRL_GETFREQ        3   /* 从 ISR 获取当前返回数据对应的频率 */
#define IMPCTRL_SHUTDOWN       4   /* 关闭所有模块并让 AFE 进入休眠/下电（此处的“SHUTDOWN”专指该含义） */



AD5940Err AppIMPCfg_init(void);

// 用于配置Impedance测量参数的结构体
typedef struct
{
/* —— 通用：适用于各类应用 —— */
  BoolFlag bParaChanged;        /* 参数已改变的标志；为真时需要重新生成序列。AppBIAInit 会自动清零 */
  uint32_t SeqStartAddr;        /* 初始化序列在 AD5940 片上 SRAM 的起始地址 */
  uint32_t MaxSeqLen;           /* 允许的最大序列长度 */
  uint32_t SeqStartAddrCal;     /* 测量序列在 AD5940 片上 SRAM 的起始地址 */
  uint32_t SeqWaitAddr[2];      /* 序列中等待（Wait）位置的地址（通常用于同步/暂停） */
  uint32_t MaxSeqLenCal;        /* 允许的最大测量序列长度 */

/* —— 应用相关参数 —— */
  float    ImpODR;              /* 输出数据率（Impedance ODR），单位 Hz */
  int32_t  NumOfData;           /* 期望获取的数据组数。默认 -1 表示“永不停止”；>=0 则采够该数量后停止 */
  float    WuptClkFreq;         /* 唤醒定时器时钟频率，通常约 32 kHz（若用软件校准会用到） */
  float    SysClkFreq;          /* 系统时钟的实际频率 */
  float    AdcClkFreq;          /* ADC 时钟的实际频率 */
  float    RcalVal;             /* 参考电阻 Rcal，单位欧姆（外部校准电阻） */

  /* —— 开关矩阵配置 —— */
  uint32_t DswitchSel;          /* D 端开关选择（见 SWD_ 常量） */
  uint32_t PswitchSel;          /* P 端开关选择（见 SWP_ 常量） */
  uint32_t NswitchSel;          /* N 端开关选择（见 SWN_ 常量） */
  uint32_t TswitchSel;          /* T 端开关选择（见 SWT_ 常量） */

  uint32_t PwrMod;              /* 功耗/带宽模式（LP/HP），见 AFEPWR_* / AFEBW_* */
  uint32_t HstiaRtiaSel;        /* 内部 RTIA 选择：RTIA_INT_200/1K/5K/10K/20K/40K/80K/160K 等 */
  uint32_t ExcitBufGain;        /* 激励缓冲增益：EXCTBUFGAIN_2 或 EXCTBUFGAIN_0P25 */
  uint32_t HsDacGain;           /* 高速 DAC 增益：HSDACGAIN_1 或 HSDACGAIN_0P2 */
  uint32_t HsDacUpdateRate;     /* 高速 DAC 更新分频（速率） */

  float    DacVoltPP;           /* DAC 交流峰峰值，单位 mVpp；最大约 800 mVpp（激励幅值） */
  float    BiasVolt;            /* 直流偏置电压，单位 mV；AC 叠加在该 DC 上。0 表示无 DC 偏置 */
  float    SinFreq;             /* 正弦激励频率，单位 Hz */

  uint32_t DftNum;              /* DFT 点数（DFTNUM_*） */
  uint32_t DftSrc;              /* DFT 数据源（DFTSRC_*：SINC3 / SINC2+Notch / AVG / RAW 等） */
  BoolFlag HanWinEn;            /* 启用汉宁窗 */

  uint32_t AdcPgaGain;          /* ADC 前置 PGA 增益：GNPGA_1/1_5/2/4/9；需保证±1.5V 输入范围内 */
  uint8_t  ADCSinc3Osr;         /* SINC3 过采样比 */
  uint8_t  ADCSinc2Osr;         /* SINC2 过采样比 */
  uint8_t  ADCAvgNum;           /* （给 DFT 的）平均次数，当 DFTSRC=AVG 时生效 */

  /* —— 频扫控制 —— */
  SoftSweepCfg_Type SweepCfg;   /* 软频扫配置（起止频率、点数、线性/对数等） */

  uint32_t FifoThresh;          /* FIFO 阈值（应为 4 的整数倍） */

  /* —— 内部使用的私有变量 —— */
  float    SweepCurrFreq;       /* 当前频点（内部运行态） */
  float    SweepNextFreq;       /* 下一个频点（内部运行态） */
  float    FreqofData;          /* 最近一次数据对应的频率（ISR 返回频率） */
  BoolFlag IMPInited;           /* 是否已完成首次初始化/序列生成 */
  SEQInfo_Type InitSeqInfo;     /* 初始化序列信息（地址/长度等） */
  SEQInfo_Type MeasureSeqInfo;  /* 测量序列信息 */
  BoolFlag StopRequired;        /* FIFO 就绪后是否需要停止测量序列 */
  uint32_t FifoDataCount;       /* 已测得的阻抗数据组计数 */
} AppIMPCfg_Type;

extern AppIMPCfg_Type AppIMPCfg; 


