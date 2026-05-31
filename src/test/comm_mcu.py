import serial
import struct
import time

class MCUComm:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        """
        初始化串口通信
        """
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        
        # 新增：记录最后一次收到 RECEIVE_OK 的时间（心跳机制）
        self.last_receive_time = 0 
        
        # 启动时尝试连接
        self.connect()

    def connect(self):
        """执行串口连接，并严格校验状态"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1, write_timeout=0.1)
            if self.ser.is_open:
                print(f"✅ MCU 通信连接成功: {self.port}")
            else:
                print(f"❌ MCU 串口 {self.port} 存在但未能打开")
                self.ser = None
        except serial.SerialException as e:
            print(f"❌ MCU 串口打开失败，请检查线缆是否断开。")
            self.ser = None

    def check_mcu_feedback(self):
        """非阻塞检查 MCU 是否回传了消息"""
        if self.ser and self.ser.in_waiting > 0:
            try:
                # 读取接收缓冲区里的所有内容
                raw_data = self.ser.read(self.ser.in_waiting)
                text_data = raw_data.decode('utf-8', errors='ignore')
                
                # 如果发现了指定的回传字符串，更新心跳时间
                if "RECEIVE_OK" in text_data:
                    self.last_receive_time = time.time()
                    
            except Exception as e:
                pass # 忽略解码报错

    def is_mcu_alive(self):
        """检查单片机是否正常响应 (如果在1秒内收到过回传，则认为在线)"""
        return (time.time() - self.last_receive_time) < 1.0

    def send_target_data(self, mode, dx, dy):
        """按照自定义协议发送数据帧，并检查回传"""
        if self.ser is None or not self.ser.is_open:
            self.connect()
            if self.ser is None:
                return

        # 处理模式字节
        if isinstance(mode, str):
            mode_int = int(mode, 16)
        else:
            mode_int = int(mode)
        mode_byte = mode_int & 0xFF 

        dx = float(dx)
        dy = float(dy)

        # 尝试打包并发送数据
        try:
            data = struct.pack('<BBBffBB', 0xAA, 0xFF, mode_byte, dx, dy, 0xFF, 0xAA)
            self.ser.write(data)
            
            # --- 发送完毕后，立刻检查有没有单片机的回传信息 ---
            self.check_mcu_feedback()
            
        except serial.SerialTimeoutException:
            print("⚠️ 串口发送超时，准备重连...")
            self.ser.close()
            self.ser = None
        except serial.SerialException as e:
            print(f"⚠️ 串口通信异常，设备已掉线！")
            if self.ser:
                self.ser.close()
            self.ser = None
        except Exception as e:
            pass