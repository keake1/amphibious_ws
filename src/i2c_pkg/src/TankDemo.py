import smbus
import time
import struct
import signal
import sys
import atexit

# 设置I2C总线号，通常为1
I2C_BUS = 0

# 设置四路电机驱动模块的I2C地址
MOTOR_ADDR = 0x34 

# 寄存器地址
ADC_BAT_ADDR = 0x00
MOTOR_TYPE_ADDR = 0x14
MOTOR_ENCODER_POLARITY_ADDR = 0x15
MOTOR_FIXED_PWM_ADDR = 0x1F
MOTOR_FIXED_SPEED_ADDR = 0x33
MOTOR_ENCODER_TOTAL_ADDR = 0x3C

# 电机类型具体值
MOTOR_TYPE_WITHOUT_ENCODER = 0
MOTOR_TYPE_TT = 1
MOTOR_TYPE_N20 = 2
MOTOR_TYPE_JGB37_520_12V_110RPM = 3 #磁环每转是44个脉冲   减速比:90  默认

# 电机类型及编码方向极性
MotorType = MOTOR_TYPE_JGB37_520_12V_110RPM
MotorEncoderPolarity = 0

bus = smbus.SMBus(I2C_BUS)
speed1 = [0,0,0,0]
speed2 = [30,30,-30,-30]
speed3 = [0,0,0,0]

pwm1 = [30,30,-30,-30]
pwm2 = [-100,-100,-100,-100]
pwm3 = [0,0,0,0]

def Motor_Init(): # 电机初始化
    bus.write_byte_data(MOTOR_ADDR, MOTOR_TYPE_ADDR, MotorType)  # 设置电机类型
    time.sleep(0.5)
    bus.write_byte_data(MOTOR_ADDR, MOTOR_ENCODER_POLARITY_ADDR, MotorEncoderPolarity)  # 设置编码极性

# 停止电机函数
def stop_motors():
    try:
        print("正在停止所有电机...")
        bus.write_i2c_block_data(MOTOR_ADDR, MOTOR_FIXED_SPEED_ADDR, speed3)
        time.sleep(0.1)  # 给一点时间让命令执行
        print("所有电机已停止")
    except Exception as e:
        print(f"停止电机时发生错误: {e}")

# 信号处理函数
def signal_handler(sig, frame):
    print('\n捕获到信号 Ctrl+C! 正在关闭...')
    stop_motors()
    sys.exit(0)

def main():
    # 注册信号处理函数（捕获Ctrl+C）
    signal.signal(signal.SIGINT, signal_handler)
    
    # 注册程序退出时的清理函数
    atexit.register(stop_motors)
    reset_data = [0] * 16
    bus.write_i2c_block_data(MOTOR_ADDR, MOTOR_ENCODER_TOTAL_ADDR, reset_data)
    print("编码器脉冲值已重置")
    try:
        while True:
            battery = bus.read_i2c_block_data(MOTOR_ADDR, ADC_BAT_ADDR)
            print("V = {0}mV".format(battery[0]+(battery[1]<<8)))
            
            Encode = struct.unpack('iiii',bytes(bus.read_i2c_block_data(MOTOR_ADDR, MOTOR_ENCODER_TOTAL_ADDR,16)))
            
            print("Encode1 = {0}  Encode2 = {1}  Encode3 = {2}  Encode4 = {3}".format(Encode[0],Encode[1],Encode[2],Encode[3]))
            
            
            # 固定转速控制
            # bus.write_i2c_block_data(MOTOR_ADDR, MOTOR_FIXED_PWM_ADDR, pwm1)
            # time.sleep(3)
            bus.write_i2c_block_data(MOTOR_ADDR, MOTOR_FIXED_SPEED_ADDR, speed2)
            # time.sleep(3)
    except Exception as e:
        print(f"发生错误: {e}")
        stop_motors()  # 确保在出错时也停止电机

if __name__ == "__main__":
    Motor_Init()
    try:
        main()
    finally:
        # 确保在任何情况下程序退出时都能停止电机
        stop_motors()
