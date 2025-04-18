// -----------------------------------------------------------------------------
// Copyright (c) 2024 Mu Shibo
// Modify by Tommy
// -----------------------------------------------------------------------------

// 转成PlatformIO工程创建
#include "main.h"

// 定义全局变量
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
MPU6050 mpu6050(I2Ctwo);
RobotProtocol rp(20);

/************实例定义*************/

// 电机实例
BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32, 33, 25, 22);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(26, 27, 14, 12);

// 编码器实例
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);

// PID控制器实例
PIDController pid_angle(1, 0, 0, 100000, 8);
PIDController pid_gyro(0.06, 0, 0, 100000, 8);
PIDController pid_distance(0.5, 0, 0, 100000, 8);
PIDController pid_speed(0.7, 0, 0, 100000, 8);
PIDController pid_yaw_angle(1.0, 0, 0, 100000, 8);
PIDController pid_yaw_gyro(0.04, 0, 0, 100000, 8);
PIDController pid_lqr_u(1, 15, 0, 100000, 8);
PIDController pid_zeropoint(0.002, 0, 0, 100000, 4);
PIDController pid_roll_angle(8, 0, 0, 100000, 450);

// 低通滤波器实例
LowPassFilter lpf_joyy(0.2);
LowPassFilter lpf_zeropoint(0.1);
LowPassFilter lpf_roll(0.3);

// commander通信实例
Commander command = Commander(Serial);

void StabAngle(char *cmd) { command.pid(&pid_angle, cmd); }
void StabGyro(char *cmd) { command.pid(&pid_gyro, cmd); }
void StabDistance(char *cmd) { command.pid(&pid_distance, cmd); }
void StabSpeed(char *cmd) { command.pid(&pid_speed, cmd); }
void StabYawAngle(char *cmd) { command.pid(&pid_yaw_angle, cmd); }
void StabYawGyro(char *cmd) { command.pid(&pid_yaw_gyro, cmd); }
void lpfJoyy(char *cmd) { command.lpf(&lpf_joyy, cmd); }
void StabLqrU(char *cmd) { command.pid(&pid_lqr_u, cmd); }
void StabZeropoint(char *cmd) { command.pid(&pid_zeropoint, cmd); }
void lpfZeropoint(char *cmd) { command.lpf(&lpf_zeropoint, cmd); }
void StabRollAngle(char *cmd) { command.pid(&pid_roll_angle, cmd); }
void lpfRoll(char *cmd) { command.lpf(&lpf_roll, cmd); }

// void Stabtest_zeropoint(char* cmd) { command.pid(&test_zeropoint, cmd); }

// WebServer实例
WebServer webserver;                               // server服务器
WebSocketsServer websocket = WebSocketsServer(81); // 定义一个webSocket服务器来处理客户发送的消息

// STS舵机实例
SMS_STS sms_sts;

// 参数定义
#define pi 3.1415927

// LQR自平衡控制器参数
float LQR_angle = 0;
float LQR_gyro = 0;
float LQR_speed = 0;
float LQR_distance = 0;
float angle_control = 0;
float gyro_control = 0;
float speed_control = 0;
float distance_control = 0;
float LQR_u = 0;
float angle_zeropoint = 1.79;  // 3.4 在理想平衡状态下俯仰角度，向前到正，向后倒负
float distance_zeropoint = -256.0; // 轮部位移零点偏置（-256为一个不可能的位移值，将其作为未刷新的标志）

// YAW轴控制数据
float YAW_gyro = 0;
float YAW_angle = 0;
float YAW_angle_last = 0;
float YAW_angle_total = 0;
float YAW_angle_zero_point = -10;
float YAW_output = 0;

// 腿部舵机控制数据
byte ID[2];
s16 Position[2];
u16 Speed[2];
byte ACC[2];

// 逻辑处理标志位
float robot_speed = 0;         // 记录当前轮部转速
float robot_speed_last = 0;    // 记录上一时刻的轮部转速
int wrobot_move_stop_flag = 0; // 记录摇杆停止的标识
int jump_flag = 0;             // 跳跃时段标识
float leg_position_add = 0;    // roll轴平衡控制量
int uncontrolable = 0;         // 机身倾角过大导致失控

// 定义摇摆相关变量
int sway_flag = 0;
int sway_count = 0;
int sway_direction = 1;
int sway_cycle = 0;
unsigned long sway_timer = 0;

// 电压检测
uint16_t bat_check_num = 0;
int BAT_PIN = 35; // select the input pin for the ADC
static esp_adc_cal_characteristics_t adc_chars;
static const adc1_channel_t channel = ADC1_CHANNEL_7;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

// 电量显示LED
#define LED_BAT 13

void setup()
{
  Serial.begin(115200);   // 通讯串口
  Serial2.begin(1000000); // 腿部sts舵机

  // Wifi初始化
  WiFi_SetAP();
  // set_sta_wifi();      // ESP-01S STA模式接入WiFi网络
  webserver.begin();
  webserver.on("/", HTTP_GET, basicWebCallback);
  websocket.begin();
  websocket.onEvent(webSocketEventCallback);

  // 舵机初始化
  // 舵机有效行程450
  // 左侧舵机[2048+12+50,2048+12+450]
  // 左侧舵机[2048-12-50,2048-12-450]
  sms_sts.pSerial = &Serial2;
  ID[0] = 1;
  ID[1] = 2;
  ACC[0] = 30;
  ACC[1] = 30;
  Speed[0] = 300;
  Speed[1] = 300;
  Position[0] = 2148;
  Position[1] = 1948;
  // 舵机(ID1/ID2)以最高速度V=2400步/秒，加速度A=50(50*100步/秒^2)，运行至各自的Position位置
  sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);

  // 电压检测
  adc_calibration_init();
  adc1_config_width(width);
  adc1_config_channel_atten(channel, atten);
  esp_adc_cal_characterize(unit, atten, width, 0, &adc_chars);

  // 电量显示LED
  pinMode(LED_BAT, OUTPUT);

  // 编码器设置
  I2Cone.begin(19, 18, 400000UL);
  I2Ctwo.begin(23, 5, 400000UL);
  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);

  // mpu6050设置
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // 连接motor对象与编码器对象
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  // 速度环PID参数
  motor1.PID_velocity.P = 0.05;
  motor1.PID_velocity.I = 1;
  motor1.PID_velocity.D = 0;

  motor2.PID_velocity.P = 0.05;
  motor2.PID_velocity.I = 1;
  motor2.PID_velocity.D = 0;

  // 驱动器设置
  motor1.voltage_sensor_align = 6;
  motor2.voltage_sensor_align = 6;
  driver1.voltage_power_supply = 8;
  driver2.voltage_power_supply = 8;
  driver1.init();
  driver2.init();

  // 连接motor对象与驱动器对象
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  motor1.torque_controller = TorqueControlType::voltage;
  motor2.torque_controller = TorqueControlType::voltage;
  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;

  // monitor相关设置
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);
  // 电机初始化
  motor1.init();
  motor1.initFOC();
  motor2.init();
  motor2.initFOC();

  // 映射电机到commander
  command.add('A', StabAngle, "pid angle");
  command.add('B', StabGyro, "pid gyro");
  command.add('C', StabDistance, "pid distance");
  command.add('D', StabSpeed, "pid speed");
  command.add('E', StabYawAngle, "pid yaw angle");
  command.add('F', StabYawGyro, "pid yaw gyro");
  command.add('G', lpfJoyy, "lpf joyy");
  command.add('H', StabLqrU, "pid lqr u");
  command.add('I', StabZeropoint, "pid zeropoint");
  command.add('J', lpfZeropoint, "lpf zeropoint");
  command.add('K', StabRollAngle, "pid roll angle");
  command.add('L', lpfRoll, "lpf roll");

  // command.add('M', Stabtest_zeropoint, "test_zeropoint");

  delay(500);
}

void loop()
{
  bat_check();        // 电压检测
  web_loop();         // Web数据更新
  mpu6050.update();   // IMU数据更新
  lqr_balance_loop(); // lqr自平衡控制，更新LQR_u
  yaw_loop();         // yaw轴转向控制，更新YAW_output
  sway_loop();        // 摇摆模式控制
  leg_loop();         // 腿部动作控制

  // 将自平衡计算输出转矩赋给电机
  motor1.target = (-0.5) * (LQR_u + YAW_output);
  motor2.target = (-0.5) * (LQR_u - YAW_output);

  // 倒地失控后关闭输出
  if (abs(LQR_angle) > 25.0f)
  {
    uncontrolable = 1;
  }
  if (uncontrolable != 0) // 倒地失控后，延时200次程序循环，避免失控后继续输出
  {
    if (abs(LQR_angle) < 10.0f) // 角度小于10度，认为小车被人扶起了
    {
      uncontrolable++;
    }
    if (uncontrolable > 200) // 被扶起后，等待200次，说明已经经过了足够的延时
    {
      uncontrolable = 0;
    }
  }

  // wrobot.go=0 未开启，1开启，如果没有开启 ，或没有被扶起，则关停输出
  // if (wrobot.go == 0 || uncontrolable != 0) {
  if (uncontrolable != 0) {
    motor1.target = 0;
    motor2.target = 0;
    leg_position_add = 0;
  }

  // 记录上一次的遥控数据数据
  wrobot.dir_last = wrobot.dir;
  wrobot.joyx_last = wrobot.joyx;
  wrobot.joyy_last = wrobot.joyy;

  // uncontrolable ==0 时，才能执行，否则target=0，电机没有输出

  // 迭代计算FOC相电压
  motor1.loopFOC();
  motor2.loopFOC();

  // 设置轮部电机输出
  motor1.move();
  motor2.move();

  command.run();
} // -----------------  void loop()

// lqr自平衡控制
void lqr_balance_loop()
{
  // LQR_distance,LQR_speed,LQR_angle,LQR_gyro 通过编码器或者mpu实际测得
  // angle_zeropoint，distance_zeropoint 角度零点和位置零点（在理想平衡状态下俯仰角度和位移目标值），用于和实际值做差，得到控制
  // angle_control , gyro_control , distance_control , speed_control : lqr 控制参数
  
  // LQR平衡算式，实际使用中为便于调参，讲算式分解为4个P控制，采用PIDController方法在commander中实时调试
  // QR_u = LQR_k1*(LQR_angle - angle_zeropoint) 
        // + LQR_k2*LQR_gyro 
        // + LQR_k3*(LQR_distance - distance_zeropoint) 
        // + LQR_k4*LQR_speed;

  // 给负值是因为按照当前的电机接线，正转矩会向后转
  // 编码器反馈shaft_angle，计算电机转轴角度平均数
  LQR_distance = (-0.5) * (motor1.shaft_angle + motor2.shaft_angle); 
  // 编码器反馈实时角速度，计算电机转速平均数
  LQR_speed = (-0.5) * (motor1.shaft_velocity + motor2.shaft_velocity); 
  LQR_angle = (float)mpu6050.getAngleY(); // pitch 角度，向前倒正数，向后倒负数
  LQR_gyro = (float)mpu6050.getGyroY(); // pitch 角速度
  // Serial.println(LQR_distance);

  // 计算pitch的角度和角速度的pid控制值
  // 实际pitch角度与零点的差值，目标值是零
  // 计算角度误差，并通过 PID 控制器得到角度控制量
  angle_control = pid_angle(LQR_angle - angle_zeropoint); 
  // 实际pitch 角速度与目标值的差值，目标值是零
  // 通过 PID 控制器对角速度进行处理，得到角速度控制量
  gyro_control = pid_gyro(LQR_gyro);

  // 零点找平时需要的日志输出
  // {
  //   static int time_temp = 0;
  //   if (time_temp > 35)
  //   {
  //     Serial.println(LQR_angle);
  //     time_temp = 0;
  //   }
  //   else
  //     time_temp++;
  // }

  // 运动细节优化处理
  if (wrobot.joyy != 0) // 有前后方向运动指令时的处理
  {
    distance_zeropoint = LQR_distance; // 位移零点重置
    pid_lqr_u.error_prev = 0;          // 输出积分清零
  }

  if ((wrobot.joyx_last != 0 && wrobot.joyx == 0) || (wrobot.joyy_last != 0 && wrobot.joyy == 0)) // 运动指令复零时的原地停车处理
  {
    wrobot_move_stop_flag = 1;
  }
  if ((wrobot_move_stop_flag == 1) && (abs(LQR_speed) < 0.5))
  {
    distance_zeropoint = LQR_distance; // 位移零点重置
    wrobot_move_stop_flag = 0;
  }

  if (abs(LQR_speed) > 15) // 被快速推动时的原地停车处理
  {
    distance_zeropoint = LQR_distance; // 位移零点重置
  }

  // 计算pitch的角度和角速度的pid控制值
  // 电机转轴角度平均数(上面计算)-位移零点=实际位移与零点的差值, 目标值是零
  // 计算位移误差，并通过位移 PID 控制器得到位移控制量
  distance_control = pid_distance(LQR_distance - distance_zeropoint); 
  // 电机转速平均数(上面计算)-速度目标值=实际转速与目标值的差值, 目标值是零
  // 计算速度误差（实际速度减去经过低通滤波和缩放后的遥控器期望速度），并通过速度 PID 控制器得到速度控制量
  speed_control = pid_speed(LQR_speed - 0.1 * lpf_joyy(wrobot.joyy));

  // 轮部离地检测
  robot_speed_last = robot_speed; // 记录连续两次的轮部转速
  robot_speed = LQR_speed;
  // 若轮部角速度、角加速度过大或处于跳跃后的恢复时期，认为出现轮部离地现象，需要特殊处理
  // 轮部离地情况下，对轮部分量不输出；反之，正常状态下完整输出平衡转矩
  if (abs(robot_speed - robot_speed_last) > 10 || abs(robot_speed) > 50 || (jump_flag != 0)) 
  {
    distance_zeropoint = LQR_distance;    // 位移零点重置
    LQR_u = angle_control + gyro_control; // 轮部离地情况下，对轮部分量不输出；反之，正常状态下完整输出平衡转矩
    pid_lqr_u.error_prev = 0;             // 输出积分清零
  }
  else
  {
    // 当轮部未离地时，LQR_u 包含:角度控制量、角速度控制量、位移控制量和速度控制量，以实现完整的自平衡控制
    LQR_u = angle_control + gyro_control + distance_control + speed_control;
    // 计算得到LQR_u在loop()函数中在几个子loop执行完，被motor1执行
    
  }

  // 触发条件：遥控器无信号输入、轮部位移控制正常介入、不处于跳跃后的恢复时期
  if (abs(LQR_u) < 5 && wrobot.joyy == 0 && abs(distance_control) < 4 && (jump_flag == 0))
  {

    LQR_u = pid_lqr_u(LQR_u); // 小转矩非线性补偿
    // Serial.println(LQR_u);

    // lpf_zeropoint 是一个低通滤波器，对 distance_control 进行滤波处理，减少高频噪声的影响。
    // 然后，将滤波后的 distance_control 作为输入，通过 pid_zeropoint 控制器进行进一步处理。
    // angle_zeropoint -= ...：将计算得到的调整量从 angle_zeropoint 中减去，实现重心自适应调整。
    // angle_zeropoint 是预设的角度零点，调整它可以让小车更好地适应不同的重心位置。
    angle_zeropoint -= pid_zeropoint(lpf_zeropoint(distance_control)); // 重心自适应
  }
  else
  {
    pid_lqr_u.error_prev = 0; // 输出积分清零
  }

  // 平衡控制参数自适应
  if (wrobot.height < 50)
  {
    pid_speed.P = 0.7;
  }
  else if (wrobot.height < 64)
  {
    pid_speed.P = 0.6;
  }
  else
  {
    pid_speed.P = 0.5;
  }
  
}

// 腿部动作控制（舵机）
void leg_loop() {
    jump_loop();  // 首先调用jump_loop
    
    // 不处于跳跃状态时才执行
    if (jump_flag == 0) {
        // 基础变量声明
        int left_height, right_height;
        float RollOffset = 25;  // 增加基础高度偏移（原来是18）
        
        // 舵机基础参数设置
        ACC[0] = 8;
        ACC[1] = 8;
        Speed[0] = 200;
        Speed[1] = 200;

        // 获取roll角度和角速度
        float roll_angle = (float)mpu6050.getAngleX() + 2.0;
        float roll_velocity = (float)mpu6050.getGyroX();
        leg_position_add = pid_roll_angle(lpf_roll(roll_angle));

        // 获取roll偏移量
        int roll_offset = wrobot.roll;
        
        // 计算高度调整
        float angleRad = abs(roll_offset) * (pi / 180.0);
        float height_factor = 1.0;  // 默认高度因子
        
        // 根据roll_offset大小调整高度因子
        if (abs(roll_offset) > 20) {
            height_factor = 1.2;  // 大角度时增加效果
        }
        
        // 计算高度调整量
        float heightAdjust = 45 * height_factor * sin(angleRad);  // 基础45mm高度差
        
        // 添加动态补偿
        float dynamic_compensation = 0;
        if (abs(roll_velocity) > 5.0) {
            dynamic_compensation = roll_velocity * 0.5;
        }

        // 根据roll_offset计算左右腿高度
        if (roll_offset > 10) {
            // 向右倾斜
            left_height = wrobot.height + RollOffset + heightAdjust + dynamic_compensation;
            right_height = wrobot.height + RollOffset - heightAdjust - dynamic_compensation;
            leg_position_add = 0;
        } else if (roll_offset < -10) {
            // 向左倾斜
            left_height = wrobot.height + RollOffset - heightAdjust + dynamic_compensation;
            right_height = wrobot.height + RollOffset + heightAdjust - dynamic_compensation;
            leg_position_add = 0;
        } else {
            // 保持平衡
            left_height = wrobot.height;
            right_height = wrobot.height;
        }

        // 安全限制：最大高度差
        float max_height_diff = 60;
        if (abs(left_height - right_height) > max_height_diff) {
            float scale = max_height_diff / abs(left_height - right_height);
            float height_diff = (left_height - right_height) * scale;
            left_height = wrobot.height + height_diff/2;
            right_height = wrobot.height - height_diff/2;
        }

        // 计算舵机位置
        Position[0] = 2048 + 12 + 9.5 * (left_height - 32) - leg_position_add;
        Position[1] = 2048 - 12 - 9.5 * (right_height - 32) - leg_position_add;

        // 舵机位置限制
        if (Position[0] < 2090) Position[0] = 2090;
        else if (Position[0] > 2530) Position[0] = 2530;
        if (Position[1] < 1566) Position[1] = 1566;
        else if (Position[1] > 2006) Position[1] = 2006;

        // 执行舵机控制
        sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);
    }
}

// 跳跃控制
void jump_loop()
{
  if ((wrobot.dir_last == 5) && (wrobot.dir == 4) && (jump_flag == 0))
  {
    ACC[0] = 0;
    ACC[1] = 0;
    Speed[0] = 0;
    Speed[1] = 0;
    Position[0] = 2048 + 12 + 8.4 * (60 - 32); // 原来80 降低腿部高度，即车重心
    Position[1] = 2048 - 12 - 8.4 * (60 - 32);
    sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);

    jump_flag = 1;
  }
  if (jump_flag > 0) // 对应上面设置 jump_flag = 1;
  {
    jump_flag++;
    if ((jump_flag > 30) && (jump_flag < 35))
    {
      ACC[0] = 0;
      ACC[1] = 0;
      Speed[0] = 0;
      Speed[1] = 0;
      Position[0] = 2048 + 12 + 8.4 * (35 - 32); // 降低腿部高度，即车重心
      Position[1] = 2048 - 12 - 8.4 * (35 - 32);
      sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);

      jump_flag = 40;
    }
    if (jump_flag > 200)
    {
      jump_flag = 0; // 跳跃过程结束
    }
  }
}

// yaw轴转向控制 （转向控制）
void yaw_loop()
{
  // YAW_output = 0.03*(YAW_Kp*YAW_angle_total + YAW_Kd*YAW_gyro);
  yaw_angle_addup();

  YAW_angle_total += wrobot.joyx * 0.002;
  float yaw_angle_control = pid_yaw_angle(YAW_angle_total);
  float yaw_gyro_control = pid_yaw_gyro(YAW_gyro);
  //  yaw 轴的角度和角速度数据进行了 PID 控制，得到 yaw_angle_control 和 yaw_gyro_control。
  // 然后，将这两个控制量相加，得到最终的 YAW_output。
  YAW_output = yaw_angle_control + yaw_gyro_control;
}

// Web数据更新
void web_loop()
{
  webserver.handleClient();
  websocket.loop();
  rp.spinOnce(); // 更新web端回传的控制信息
}

// yaw轴角度累加函数
void yaw_angle_addup()
{
  YAW_angle = (float)mpu6050.getAngleZ();
  YAW_gyro = (float)mpu6050.getGyroZ();

  if (YAW_angle_zero_point == (-10))
  {
    YAW_angle_zero_point = YAW_angle;
  }

  float yaw_angle_1, yaw_angle_2, yaw_addup_angle;
  if (YAW_angle > YAW_angle_last)
  {
    yaw_angle_1 = YAW_angle - YAW_angle_last;
    yaw_angle_2 = YAW_angle - YAW_angle_last - 2 * PI;
  }
  else
  {
    yaw_angle_1 = YAW_angle - YAW_angle_last;
    yaw_angle_2 = YAW_angle - YAW_angle_last + 2 * PI;
  }

  if (abs(yaw_angle_1) > abs(yaw_angle_2))
  {
    yaw_addup_angle = yaw_angle_2;
  }
  else
  {
    yaw_addup_angle = yaw_angle_1;
  }

  YAW_angle_total = YAW_angle_total + yaw_addup_angle;
  YAW_angle_last = YAW_angle;
}

void basicWebCallback(void)
{
  webserver.send(300, "text/html", basic_web);
}

void webSocketEventCallback(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  if (type == WStype_TEXT)
  {
    String payload_str = String((char *)payload);
    StaticJsonDocument<300> doc;
    DeserializationError error = deserializeJson(doc, payload_str);

    String mode_str = doc["mode"];
    if (mode_str == "basic")
    {
      rp.parseBasic(doc);
    }
  }
}

// 电压检测初始化
void adc_calibration_init()
{
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
  {
    printf("eFuse Two Point: Supported\n");
  }
  else
  {
    printf("eFuse Two Point: NOT supported\n");
  }
  // Check Vref is burned into eFuse
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
  {
    printf("eFuse Vref: Supported\n");
  }
  else
  {
    printf("eFuse Vref: NOT supported\n");
  }
}

// 电压检测
void bat_check()
{
  if (bat_check_num > 1000)
  {
    // 电压读取
    uint32_t sum = 0;
    sum = analogRead(BAT_PIN);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(sum, &adc_chars);
    double battery = (voltage * 3.97) / 1000.0;

    // Serial.println(battery);
    // 电量显示
    if (battery > 7.8)
      digitalWrite(LED_BAT, HIGH);
    else
      digitalWrite(LED_BAT, LOW);

    bat_check_num = 0;
  }
  else
    bat_check_num++;
}

// 摇摆模式控制函数
void sway_loop()
{
    if ((wrobot.dir_last != 6) && (wrobot.dir == 6) && (sway_flag == 0))
    {
        // 初始化摇摆参数
        sway_flag = 1;
        sway_count = 0;
        sway_cycle = 0;
        sway_direction = 1;
        sway_timer = millis();
        
        // 设置舵机参数 - 调整速度和加速度以获得更好的效果
        ACC[0] = 12;      // 增加加速度（原来是8）
        ACC[1] = 12;
        Speed[0] = 300;   // 增加速度（原来是200）
        Speed[1] = 300;
    }

    if (sway_flag > 0)
    {
        unsigned long current_time = millis();
        
        // 增加摇摆周期到800ms（原来是500ms），使动作更流畅
        if (current_time - sway_timer >= 800)
        {
            sway_direction = -sway_direction;
            sway_count++;
            sway_timer = current_time;
            
            if (sway_count % 4 == 0)
            {
                sway_cycle++;
                
                if (sway_cycle >= 5)
                {
                    // 重置所有标志和状态
                    sway_flag = 0;
                    sway_count = 0;
                    sway_cycle = 0;
                    wrobot.roll = 0;    // 恢复平衡位置
                    wrobot.dir = 4;     // 切换到STOP状态
                    return;
                }
            }
        }
        
        // 增加摇摆幅度并添加渐变效果
        float target_angle = sway_direction * 45;  // 增加到45度（原来是35度）
        
        // 计算当前周期内的时间比例（0到1之间）
        float time_ratio = (float)(current_time - sway_timer) / 800.0f;
        
        // 使用正弦函数使动作更平滑
        float smooth_angle = target_angle * sin(time_ratio * PI);
        
        // 设置摇摆角度
        wrobot.roll = (int)smooth_angle;
    }
}