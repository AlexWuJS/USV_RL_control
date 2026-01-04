#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import struct
import sys

# --- 配置参数 ---
PORT = '/dev/ttyUSB0'  # 根据你的实际情况修改，C++代码中是这个端口
BAUDRATE = 9600        # HWT101CT 默认波特率

# --- 标定指令集 ---
# 1. 解锁/配置模式 (FF AA 69 88 B5)
CMD_UNLOCK = b'\xFF\xAA\x69\x88\xB5'
# 2. Z轴角度归零 (FF AA 76 00 00)
CMD_ZERO_Z = b'\xFF\xAA\x76\x00\x00'
# 3. 保存并退出 (FF AA 00 00 00)
CMD_SAVE   = b'\xFF\xAA\x00\x00\x00'

def get_checksum(data_bytes):
    """计算校验和"""
    return sum(data_bytes) & 0xFF

def parse_imu_data(ser):
    """监听并解析IMU数据，验证标定结果"""
    print("\n正在验证标定结果 (按 Ctrl+C 退出)...")
    print("-" * 40)
    
    buffer = b''
    count = 0
    
    # 尝试读取 50 组数据用于验证
    while count < 50:
        try:
            # 读取数据
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)
            
            # 寻找包头 0x55
            while len(buffer) >= 11:
                # 检查帧头
                if buffer[0] != 0x55:
                    buffer = buffer[1:] # 移位
                    continue
                
                # 检查校验和 (前10字节之和等于第11字节)
                frame = buffer[:11]
                if get_checksum(frame[:10]) != frame[10]:
                    buffer = buffer[1:] # 校验失败，移位
                    continue

                # 校验通过，判断类型
                frame_type = frame[1]
                
                # 0x53 是角度包 (Angle)
                if frame_type == 0x53:
                    # 解析 Yaw (第6、7字节)
                    # HWT101CT 协议: 55 53 [RollL H] [PitchL H] [YawL H] [VL H] [SUM]
                    # 注意：C++代码中是 buffer[i+6] 开始，这对应 frame[6]
                    yaw_raw = struct.unpack('<h', frame[6:8])[0]
                    
                    # 转换公式: Angle = raw / 32768 * 180
                    yaw_angle = (yaw_raw / 32768.0) * 180.0
                    
                    print(f"验证数据 [{count+1}]: 当前航向角 (Yaw) = {yaw_angle:.4f}°")
                    count += 1
                    
                    # 如果角度非常接近 0 (例如绝对值小于 0.1)，说明成功
                    if abs(yaw_angle) < 0.5:
                        print(">> 标定成功！角度已归零。")
                        return True
                
                # 移除已处理的一帧
                buffer = buffer[11:]
                
            time.sleep(0.01)
            
        except KeyboardInterrupt:
            print("\n用户中断")
            return False
    return True

def main():
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        if ser.is_open:
            print(f"串口 {PORT} 已打开，波特率 {BAUDRATE}")
    except Exception as e:
        print(f"无法打开串口: {e}")
        sys.exit(1)

    try:
        # --- 发送标定指令 ---
        print("\n开始发送标零指令...")
        
        # 1. 发送解锁指令
        ser.write(CMD_UNLOCK)
        print(f"发送: {CMD_UNLOCK.hex().upper()} (解锁)")
        time.sleep(0.2) # 稍微延时，确保设备处理
        
        # 2. 发送归零指令
        ser.write(CMD_ZERO_Z)
        print(f"发送: {CMD_ZERO_Z.hex().upper()} (Z轴归零)")
        time.sleep(1.0) # 归零需要一点内部处理时间
        
        # 3. 发送保存指令
        ser.write(CMD_SAVE)
        print(f"发送: {CMD_SAVE.hex().upper()} (保存)")
        time.sleep(0.2)
        
        print("指令发送完毕，准备验证...")
        
        # 清空缓冲区，避免读取旧数据
        ser.reset_input_buffer()
        
        # --- 验证结果 ---
        parse_imu_data(ser)

    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        ser.close()
        print("\n串口已关闭")

if __name__ == "__main__":
    main()