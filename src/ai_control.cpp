#include "ai_control.h"
#include "main.h" // 包含主头文件以访问wrobot结构体

  
// 删除所有AISerial相关定义 # 发送AI控制命令 #SPEED:0.5
void handleAICommands() {
  static String aiBuffer;
  while(Serial.available()) {
    char c = Serial.read();
    if(c == '\n') {
      if(aiBuffer.startsWith("#")) {  // AI指令标识
        // 解析AI指令
        aiBuffer = aiBuffer.substring(1);
        int sepIndex = aiBuffer.indexOf(':');
        if(sepIndex != -1) {
          String cmd = aiBuffer.substring(0, sepIndex);
          float value = aiBuffer.substring(sepIndex+1).toFloat();
          
          if(cmd == "SPEED") {
            wrobot.joyy = constrain(value, -1.0, 1.0);
          } 
          else if(cmd == "ROLL") {
            wrobot.roll = constrain(value, -45, 45);
          }
          else if(cmd == "HEIGHT") {
            wrobot.height = constrain(value, 30, 80);
          }
        }
      }
      aiBuffer = "";
    } else {
      aiBuffer += c;
    }
  }
}
