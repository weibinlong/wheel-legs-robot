#pragma once
#include <HardwareSerial.h>

// extern HardwareSerial AISerial; // 声明外部串口实例
 
// AI指令处理
// 删除原有的HardwareSerial声明
// 修改为：
extern void handleAICommands(); // 只保留函数声明