#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <SimpleFOC.h>
#include <WebSocketsServer.h>
#include "robot.h"

// 机器人控制头文件
#include "Servo_STS3032.h"
#include <ArduinoJson.h>
#include <WebServer.h>
#include <WiFi.h>
#include <FS.h>
#include "basic_web.h"
#include "wifi_config.h"
#include "esp_adc_cal.h"

void webSocketEventCallback(uint8_t num, WStype_t type, uint8_t *payload, size_t length);

// 声明全局变量
extern TwoWire I2Cone;
extern TwoWire I2Ctwo;
extern MPU6050 mpu6050;
extern RobotProtocol rp;

void basicWebCallback(void);
void adc_calibration_init();
void bat_check();
void web_loop();
void lqr_balance_loop();
void yaw_loop();
void leg_loop();
void jump_loop();
void yaw_angle_addup();
void sway_loop();

