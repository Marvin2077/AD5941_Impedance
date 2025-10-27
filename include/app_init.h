#ifndef APP_INIT_H
#define APP_INIT_H

extern "C" {
#include "ad5940.h" // 需要包含 AD5940 库头文件以使用 AD5940Err 类型

/**
 * @brief 执行 AD5941 的统一初始化。
 * 配置时钟、FIFO、序列器内存、统一参考电压、
 * LPDAC/LP PA 基础状态，并加载温度测量序列。
 * @return 成功返回 AD5940ERR_OK，否则返回错误代码。
 */
AD5940Err App_AD5941_Init();
}

#endif // APP_INIT_H