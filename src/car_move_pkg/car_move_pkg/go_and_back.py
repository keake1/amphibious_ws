#!/usr/bin/env python3
import smbus
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import struct

# I2C配置
I2C_BUS = 0
MOTOR_ADDR = 0x34

# 寄存器定义
class Registers:
    ADC_BAT = 0x00
    MOTOR_TYPE = 0x14
    ENCODER_POLARITY = 0x15
    FIXED_PWM = 0x1F
    FIXED_SPEED = 0x33
    ENCODER_TOTAL = 0x3C

# 电机类型定义
class MotorTypes:
    WITHOUT_ENCODER = 0
    TT = 1
    N20 = 2
    JGB37_520_12V_110RPM = 3  # 磁环每转44脉冲，减速比:90

# PWM配置
class PwmValues:
    FORWARD = [41, -48, -54, 62]   # 前进
    BACKWARD = [-48, 44, 50, -48]  # 后退
    STOP = [0, 0, 0, 0]            # 停止

class CarMoveNode(Node):
    def __init__(self):
        super().__init__('car_move_node')
        
        # 状态定义
        self.STATE_INIT = 'init'
        self.STATE_WAIT_FLY_OFF = 'wait_fly_off'
        self.STATE_FLY_OFF = 'fly_off'
        
        # 初始化属性
        self.bus = smbus.SMBus(I2C_BUS)
        self.state = self.STATE_INIT
        self.start_time = time.time()
        self.pwm2_start = None
        self.running = True
        
        # 创建发布者和订阅者
        self.mode_pub = self.create_publisher(String, '/mode_switch', 10)
        self.is_off_sub = self.create_subscription(
            String, '/is_off', self.is_off_callback, 10)
        
        # 初始化电机
        self.initialize_motors()
        self.get_logger().info('小车启动，持续发送前进PWM（三秒）')
        
        # 启动线程
        self.start_threads()
    
    def initialize_motors(self):
        """初始化电机设置并重置编码器"""
        try:
            # 设置电机类型
            self.bus.write_byte_data(
                MOTOR_ADDR, 
                Registers.MOTOR_TYPE, 
                MotorTypes.JGB37_520_12V_110RPM
            )
            time.sleep(0.5)
            
            # 设置编码器极性
            self.bus.write_byte_data(
                MOTOR_ADDR, 
                Registers.ENCODER_POLARITY, 
                0
            )
            
            # 重置编码器计数
            reset_data = [0] * 16
            self.bus.write_i2c_block_data(
                MOTOR_ADDR, 
                Registers.ENCODER_TOTAL, 
                reset_data
            )
        except Exception as e:
            self.get_logger().error(f'电机初始化失败: {e}')
    
    def start_threads(self):
        """启动PWM发送和状态监控线程"""
        # PWM发送线程 - 无延迟持续发送
        self.pwm_thread = threading.Thread(target=self.pwm_loop)
        self.pwm_thread.daemon = True
        self.pwm_thread.start()
        
        # 状态监控线程
        self.state_thread = threading.Thread(target=self.state_monitor)
        self.state_thread.daemon = True
        self.state_thread.start()
    
    def pwm_loop(self):
        """持续发送PWM信号的线程，无延迟"""
        while self.running:
            try:
                if self.state == self.STATE_INIT:
                    # 读取编码器数据记录行驶状态
                    # try:
                    #     encoder_data = self.bus.read_i2c_block_data(
                    #         MOTOR_ADDR, Registers.ENCODER_TOTAL, 16)
                    #     encoders = struct.unpack('iiii', bytes(encoder_data))
                    #     self.get_logger().info(
                    #         f"编码器: E1={encoders[0]} E2={encoders[1]} "
                    #         f"E3={encoders[2]} E4={encoders[3]}"
                    #     )
                    # except Exception as e:
                    #     self.get_logger().warn(f"读取编码器失败: {e}")
                    
                    # 发送前进PWM
                    self.bus.write_i2c_block_data(
                        MOTOR_ADDR, Registers.FIXED_PWM, PwmValues.STOP)
                        
                elif self.state == self.STATE_WAIT_FLY_OFF:
                    # 飞行过程中停止电机
                    self.bus.write_i2c_block_data(
                        MOTOR_ADDR, Registers.FIXED_PWM, PwmValues.STOP)
                        
                elif self.state == self.STATE_FLY_OFF:
                    # 发送后退PWM
                    self.bus.write_i2c_block_data(
                        MOTOR_ADDR, Registers.FIXED_PWM, PwmValues.BACKWARD)
            except Exception as e:
                self.get_logger().error(f"PWM发送失败: {e}")
                time.sleep(0.1)  # 错误时短暂延迟避免CPU占用过高
    
    def state_monitor(self):
        """监控状态和计时的线程"""
        while self.running:
            try:
                now = time.time()
                
                # 初始状态持续3秒后切换
                if self.state == self.STATE_INIT and now - self.start_time >= 3.8:
                    self.publish_fly_mode_on()
                # 飞行结束后返回，持续3秒后退出
                elif self.state == self.STATE_FLY_OFF:
                    if self.pwm2_start is None:
                        self.get_logger().info('收到fly_off，持续发送后退PWM（三秒）')
                        self.pwm2_start = now
                    elif now - self.pwm2_start >= 3.0:
                        self.get_logger().info('后退PWM已发送三秒，程序退出')
                        self.running = False
                        rclpy.shutdown()
                        break
                        
                time.sleep(0.01)  # 状态监控适当休眠
            except Exception as e:
                self.get_logger().error(f"状态监控错误: {e}")
                time.sleep(0.1)
    
    def publish_fly_mode_on(self):
        """发布飞行模式开启消息"""
        msg = String()
        msg.data = 'fly_mode_on'
        self.mode_pub.publish(msg)
        self.get_logger().info('已发布fly_mode_on，等待fly_off')
        self.state = self.STATE_WAIT_FLY_OFF
    
    def is_off_callback(self, msg):
        """处理fly_off消息的回调函数"""
        if msg.data == 'fly_off' and self.state == self.STATE_WAIT_FLY_OFF:
            self.state = self.STATE_FLY_OFF
            self.pwm2_start = None

def main():
    rclpy.init()
    node = CarMoveNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
