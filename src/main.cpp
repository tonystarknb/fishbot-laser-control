#include <Arduino.h>
#include <micro_ros_platformio.h>    // 包含用于 ESP32 的 micro-ROS PlatformIO 库
#include <WiFi.h>                    // 包含 ESP32 的 WiFi 库
#include <rcl/rcl.h>                 // 包含 ROS 客户端库 (RCL)
#include <rclc/rclc.h>               // 包含用于 C 的 ROS 客户端库 (RCLC)
#include <rclc/executor.h>           // 包含 RCLC 执行程序库，用于执行订阅和发布
#include <geometry_msgs/msg/twist.h> // 包含 ROS2 geometry_msgs/Twist 消息类型
#include <Esp32PcntEncoder.h>        // 包含用于计数电机编码器脉冲的 ESP32 PCNT 编码器库
#include <Esp32McpwmMotor.h>         // 包含使用 ESP32 的 MCPWM 硬件模块控制 DC 电机的 ESP32 MCPWM 电机库
#include <PidController.h>           // 包含 PID 控制器库，用于实现 PID 控制
#include <Kinematics.h>

Esp32PcntEncoder encoders[4];      // 创建一个长度为 4 的 ESP32 PCNT 编码器数组
rclc_executor_t executor;          // 创建一个 RCLC 执行程序对象，用于处理订阅和发布
rclc_support_t support;            // 创建一个 RCLC 支持对象，用于管理 ROS2 上下文和节点
rcl_allocator_t allocator;         // 创建一个 RCL 分配器对象，用于分配内存
rcl_node_t node;                   // 创建一个 RCL 节点对象，用于此基于 ESP32 的机器人小车
rcl_subscription_t subscriber;     // 创建一个 RCL 订阅对象，用于订阅 ROS2 消息
geometry_msgs__msg__Twist sub_msg; // 创建一个 ROS2 geometry_msgs/Twist 消息对象
Esp32McpwmMotor motor;             // 创建一个 ESP32 MCPWM 电机对象，用于控制 DC 电机
float out_motor_speed[4];          // 创建一个长度为 2 的浮点数数组，用于保存输出电机速度
float current_speeds[4];           // 创建一个长度为 2 的浮点数数组，用于保存当前电机速度
PidController pid_controller[4];   // 创建PidController的两个对象
Kinematics kinematics;

void twist_callback(const void *msg_in)
{
  const geometry_msgs__msg__Twist *twist_msg = (const geometry_msgs__msg__Twist *)msg_in;
  static float target_motor_speed1, target_motor_speed2,target_motor_speed3,target_motor_speed4;
  float linear_x = twist_msg->linear.x;   // 获取 Twist 消息的线性 x 分量
  float linear_y = twist_msg->linear.y;
  float angular_z = twist_msg->angular.z; // 获取 Twist 消息的角度 z 分量
  kinematics.kinematic_inverse(linear_x * 1000,linear_x * 1000, angular_z, target_motor_speed1,target_motor_speed2,target_motor_speed3,target_motor_speed4);
  pid_controller[0].update_target(target_motor_speed1);
  pid_controller[1].update_target(target_motor_speed2);
  pid_controller[2].update_target(target_motor_speed3);
  pid_controller[3].update_target(target_motor_speed4);
  // if (linear_x == 0 && angular_z == 0)    // 如果 Twist 消息没有速度命令
  // {
  //   pid_controller[0].update_target(0); // 更新控制器的目标值
  //   pid_controller[1].update_target(0);
  //   pid_controller[2].update_target(0);
  //   pid_controller[3].update_target(0);
  //   motor.updateMotorSpeed(0, 0); // 停止第一个电机
  //   motor.updateMotorSpeed(1, 0); // 停止第二个电机
  //   motor.updateMotorSpeed(2, 0); 
  //   motor.updateMotorSpeed(3, 0); 
  //   return;                       // 退出函数
  //}
  // for(int index=0;index<4;index++)
  // {
  //   if(pid_controller[index].target_==0)
  //   {
  //     // motor.updateMotorSpeed(index,0);
  //     out_motor_speed[index]=0;
  //   }
  //   else{
  //     pid_controller[index].update_target(linear_x*1000);
  //   }
  // }
  // if(pid_controller)
  // // 根据线速度和角速度控制两个电机的转速
  // if (linear_x != 0)
  // {
  //   pid_controller[0].update_target(linear_x * 1000);  // 使用mm/s作为target
  //   pid_controller[1].update_target(linear_x * 1000);
  //   pid_controller[2].update_target(linear_x * 1000);
  //   pid_controller[3].update_target(linear_x * 1000);

  // }
}

// 这个函数是一个后台任务，负责设置和处理与 micro-ROS 代理的通信。
void microros_task(void *param)
{
  // 设置 micro-ROS 代理的 IP 地址。
  IPAddress agent_ip;
  agent_ip.fromString("192.168.31.121");//192.168.110.115

  // 使用 WiFi 网络和代理 IP 设置 micro-ROS 传输层。
  set_microros_wifi_transports("Xiaomi_E2EF", "opencvrobot", agent_ip, 8888);

  // 等待 2 秒，以便网络连接得到建立。
  delay(2000);

  // 设置 micro-ROS 支持结构、节点和订阅。
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_car", "", &support);
  rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel");

  // 设置 micro-ROS 执行器，并将订阅添加到其中。
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, &twist_callback, ON_NEW_DATA);

  // 循环运行 micro-ROS 执行器以处理传入的消息。
  while (true)
  {
    delay(100);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }
}

// 这个函数根据编码器读数更新两个轮子速度。
// void update_speed()
// {
//   // 初始化静态变量以存储上一次更新时间和编码器读数。
//   static uint64_t last_update_time = millis();
//   static int64_t last_ticks[4];

//   // 获取自上次更新以来的经过时间。
//   uint64_t dt = millis() - last_update_time;
//   if (dt == 0)
//     return;

//   // 获取当前的编码器读数并计算当前的速度。
//   int32_t pt[4];
//   pt[0] = encoders[0].getTicks() - last_ticks[0];
//   pt[1] = encoders[1].getTicks() - last_ticks[1];
//   pt[2] = encoders[2].getTicks() - last_ticks[2];
//   pt[3] = encoders[3].getTicks() - last_ticks[3];

//   current_speeds[0] = float(pt[0] * 0.1618396) / dt * 1000;//m/s
//   current_speeds[1] = float(pt[1] * 0.1618396) / dt * 1000;
//   current_speeds[2] = float(pt[2] * 0.1618396) / dt * 1000;
//   current_speeds[3] = float(pt[3] * 0.1618396) / dt * 1000;

//   // 更新上一次更新时间和编码器读数。
//   last_update_time = millis();
//   last_ticks[0] = encoders[0].getTicks();
//   last_ticks[1] = encoders[1].getTicks();
//   last_ticks[2] = encoders[2].getTicks();
//   last_ticks[3] = encoders[3].getTicks();
// }

void setup()
{
  // 初始化串口通信，波特率为115200
  Serial.begin(115200);
  // 将两个电机分别连接到引脚22、23和12、13上
  motor.attachMotor(0, 12, 22);
  motor.attachMotor(1, 17, 16);
  motor.attachMotor(2, 33, 25);
  motor.attachMotor(3, 26, 27);
  // motor.attachMotor(0, 33, 25); // 将电机0连接到引脚33和引脚25
  // motor.attachMotor(1, 26, 27); // 将电机1连接到引脚26和引脚27
  // motor.attachMotor(2, 12, 22); // 将电机2连接到引脚12和引脚22
  // motor.attachMotor(3, 16, 17); // 将电机3连接到引脚16和引脚17
  // 在引脚32、33和26、25上初始化两个编码器
  // encoders[0].init(0, 36, 39);
  // encoders[1].init(1, 35, 32);
  // encoders[2].init(2, 14, 23);
  // encoders[3].init(3, 13, 15);
  encoders[0].init(0, 14, 23);
  encoders[1].init(1, 13, 15);
  encoders[2].init(2, 39, 36);
  encoders[3].init(3, 32, 35);
  // 初始化PID控制器的kp、ki和kd
  pid_controller[0].update_pid(0.625, 0.125, 0.0);
  pid_controller[1].update_pid(0.625, 0.125, 0.0);
  pid_controller[2].update_pid(0.625, 0.125, 0.0);
  pid_controller[3].update_pid(0.625, 0.125, 0.0);
  // 初始化PID控制器的最大输入输出，MPCNT大小范围在正负100之间
  pid_controller[0].out_limit(-100, 100);
  pid_controller[1].out_limit(-100, 100);
  pid_controller[2].out_limit(-100, 100);
  pid_controller[3].out_limit(-100, 100);
  //// 设置运动学参数
  kinematics.set_motor_param(0,45,44,65);
  kinematics.set_motor_param(1,45,44,65);
  kinematics.set_motor_param(2,45,44,65);
  kinematics.set_motor_param(3,45,44,65);
  kinematics.set_kinematic_param(8,6);

  // 在核心0上创建一个名为"microros_task"的任务，栈大小为10240
  xTaskCreatePinnedToCore(microros_task, "microros_task", 10240, NULL, 1, NULL, 0);
}
void loop()
{
  static uint64_t last_update_info_time = millis();
  kinematics.update_motor_ticks(micros(),encoders[0].getTicks(),encoders[1].getTicks(),encoders[2].getTicks(),encoders[3].getTicks());
  for(int i=0;i<4;i++)
  {
  out_motor_speed[i]=pid_controller[i].update(kinematics.motor_speed(i));
  motor.updateMotorSpeed(i,out_motor_speed[i]);
  }
  delay(10);
}
/*
void loop()
{
  // 更新电机速度
  //update_speed();
  for (int index = 0; index < 4; index++)
    {
        // 目标速度为0时停止控制，解决 #https://fishros.org.cn/forum/topic/1372 问题
        if (pid_controller[index].target_ == 0)
        {
            out_motor_speed[index] = 0;
        }
        else
        {
            // 使用 pid_controller 控制器对电机速度进行 PID 控制
            Serial.printf("Motor%dSpeed %f\n",index,current_speeds[index]);
            out_motor_speed[index] = pid_controller[index].update(current_speeds[index]);
            //Serial.printf("Motor%dSpeed has update to %f\n",index,out_motor_speed[index]);
        }
        // 将 PID 控制器的输出值作为电机的目标速度进行控制0
        motor.updateMotorSpeed(index, out_motor_speed[index]);
        // fishlog_debug("pid", "index:%d target:%f current:%f out=%f", index, pid_controller[index].target_, kinematics.motor_speed(index), out_motor_speed[index]);
    }
  // // 计算最新的电机输出值
  // out_motor_speed[0] = pid_controller[0].update(current_speeds[0]);
  // out_motor_speed[1] = pid_controller[1].update(current_speeds[1]);
  // out_motor_speed[2] = pid_controller[2].update(current_speeds[2]);
  // out_motor_speed[3] = pid_controller[3].update(current_speeds[3]);
  // // 更新电机0和电机1的速度值
  // motor.updateMotorSpeed(0, out_motor_speed[0]);
  // motor.updateMotorSpeed(1, out_motor_speed[1]);
  // motor.updateMotorSpeed(2, out_motor_speed[2]);
  // motor.updateMotorSpeed(3, out_motor_speed[3]);
  // 延迟10毫秒
  delay(10);
}
*/
// #include <Arduino.h>
// #include <Esp32McpwmMotor.h>

// Esp32McpwmMotor motor;

// void setup()
// {
//     Serial.begin(115200);

//     motor.attachMotor(0, 33, 25);
//     motor.attachMotor(1, 26, 27);
//     motor.attachMotor(2, 12, 22);
//     motor.attachMotor(3, 16, 17);
// }

// void loop()
// {
//     motor.updateMotorSpeed(0, -70);  // 70%占空比
//     motor.updateMotorSpeed(1, 70);
//     motor.updateMotorSpeed(2, -70);
//     motor.updateMotorSpeed(3, 70);
//     delay(2000);
//     motor.updateMotorSpeed(0, 70);
//     motor.updateMotorSpeed(1, -70);
//     motor.updateMotorSpeed(2, 70);
//     motor.updateMotorSpeed(3, -70);
//     delay(2000);
// }