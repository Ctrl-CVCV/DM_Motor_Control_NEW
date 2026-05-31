import cv2
import numpy as np
import serial
import serial.tools.list_ports
import struct
import time
import sys
import math

# === 激光与摄像头物理安装偏差补偿 ===
OFFSET_X = -0.01
OFFSET_Y = 0.05

# ==========================================
# 0. 靶心坐标与半径平滑防跳变滤波器 (针对画圆专版)
# ==========================================
class TargetSmoother:
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.cx = None
        self.cy = None
        self.r = None
        self.reject_count = 0  # 连续跳变计数器

    def update(self, cx, cy, r):
        if self.cx is None:
            self.cx, self.cy, self.r = cx, cy, r
            self.reject_count = 0
        else:
            # 异常跳变屏蔽：如果靶心或半径单帧跳变超过 80 像素，判定为光照闪烁抽风
            if abs(cx - self.cx) > 80 or abs(cy - self.cy) > 80 or abs(r - self.r) > 80:
                self.reject_count += 1
                # 如果连续 10 帧(约 0.3秒)都跳变很大，说明真的是目标板被人移走了，接受新的位置
                if self.reject_count > 10:
                    self.cx, self.cy, self.r = cx, cy, r
                    self.reject_count = 0
                return int(self.cx), int(self.cy), int(self.r)
            
            # EMA平滑：消除小幅度的狂闪抖动，让锚点稳如泰山
            self.reject_count = 0
            self.cx = self.alpha * cx + (1.0 - self.alpha) * self.cx
            self.cy = self.alpha * cy + (1.0 - self.alpha) * self.cy
            self.r = self.alpha * r + (1.0 - self.alpha) * self.r
            
        return int(self.cx), int(self.cy), int(self.r)

    def reset(self):
        self.cx = None
        self.cy = None
        self.r = None
        self.reject_count = 0

# ==========================================
# 1. 运动控制通信模块 (ttyACM 自动重连版)
# ==========================================
class MCUComm:
    def __init__(self, baudrate=115200):
        self.port = "未连接"
        self.baudrate = baudrate
        self.ser = None
        self.last_receive_time = 0 
        self.connect()

    def connect(self):
        ports = serial.tools.list_ports.comports()
        target_ports = [p.device for p in ports if 'ttyACM' in p.device]
        for p in target_ports:
            try:
                self.ser = serial.Serial(p, self.baudrate, timeout=0.1, write_timeout=0.1)
                if self.ser.is_open:
                    self.port = p
                    print(f"\n✅ 运动控制连接成功: {self.port}")
                    return
            except serial.SerialException:
                pass
        self.ser = None
        self.port = "未连接"

    def check_mcu_feedback(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                if "RECEIVE_OK" in self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore'):
                    self.last_receive_time = time.time()
            except Exception: pass 

    def is_mcu_alive(self):
        return (time.time() - self.last_receive_time) < 1.0

    def send_target_data(self, mode, dx, dy):
        if self.ser is None or not self.ser.is_open:
            self.connect()
            if self.ser is None: return 

        # === 核心修改：强制无论何时，模式部分始终发送 0x02 ===
        mode_byte = 0x02 
        
        try:
            self.ser.write(struct.pack('<BBBffBB', 0xAA, 0xFF, mode_byte, float(dx), float(dy), 0xFF, 0xAA))
            self.check_mcu_feedback()
        except Exception:
            if self.ser: self.ser.close()
            self.ser = None
            self.port = "未连接"

# ==========================================
# 2. 视觉识别模块 (全点集计算最大半径)
# ==========================================
class BullseyeVision:
    def __init__(self):
        self.center_tolerance = 15  
        self.min_concentric_circles = 2 
        self.canny_lower = 40
        self.canny_upper = 120

    def find_bullseye(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        contours, _ = cv2.findContours(cv2.Canny(blurred, self.canny_lower, self.canny_upper), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        valid_centers = [] 
        for c in contours:
            area = cv2.contourArea(c)
            if 50 < area < 80000: 
                peri = cv2.arcLength(c, True)
                if peri == 0: continue
                if 4 * np.pi * (area / (peri * peri)) > 0.6: 
                    M = cv2.moments(c)
                    if M["m00"] > 0: valid_centers.append({'center': (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])), 'contour': c, 'area': area})
        
        best_target, max_circles_count, target_contours = None, 0, []
        for i, data1 in enumerate(valid_centers):
            cx1, cy1 = data1['center']
            current_cluster_contours, count = [data1['contour']], 1
            for j, data2 in enumerate(valid_centers):
                if i == j: continue
                if np.sqrt((cx1 - data2['center'][0])**2 + (cy1 - data2['center'][1])**2) < self.center_tolerance:
                    if abs(data1['area'] - data2['area']) > 50: 
                        count += 1
                        current_cluster_contours.append(data2['contour'])
            if count > max_circles_count and count >= self.min_concentric_circles:
                max_circles_count, best_target, target_contours = count, (cx1, cy1), current_cluster_contours

        radius = 0
        if best_target is not None and len(target_contours) > 0:
            # 聚合所有点以确保抗干扰能力，不受激光遮挡造成的轮廓断裂影响
            all_points = np.vstack(target_contours)
            _, radius = cv2.minEnclosingCircle(all_points)
            
        return best_target, target_contours, radius

# ==========================================
# 3. 主程序逻辑
# ==========================================
def main():
    vision, mcu = BullseyeVision(), MCUComm(baudrate=115200)
    smoother = TargetSmoother(alpha=0.2)  # 实例化靶心平滑器
    
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened(): sys.exit()

    total_points = 50
    current_point_idx = 0
    last_point_time = 0.0
    point_stay_duration = 0.4 # 每个点停留 0.4 秒，走完一圈约 20 秒

    try:
        while True:
            ret, frame = cap.read()
            if not ret: continue
            
            h, w = frame.shape[:2]
            center_x, center_y = w // 2, h // 2

            target_center, contours, radius_raw = vision.find_bullseye(frame)
            
            if target_center is not None:
                cx_raw, cy_raw = target_center
                
                # === 引入防跳变与平滑滤波：只平滑圆心和半径 ===
                cx, cy, radius = smoother.update(cx_raw, cy_raw, radius_raw)
                
                circle_points = []
                for i in range(total_points):
                    angle = -math.pi / 2 + (2 * math.pi * i / total_points)
                    circle_points.append((int(cx + radius * math.cos(angle)), int(cy + radius * math.sin(angle))))

                now = time.time()
                if now - last_point_time >= point_stay_duration:
                    current_point_idx = (current_point_idx + 1) % total_points
                    last_point_time = now

                target_px, target_py = circle_points[current_point_idx]
                # 基于当前目标点计算误差并补偿
                dx_raw = (target_px - center_x) / w
                dy_raw = (target_py - center_y) / h

                dx_comp = round(dx_raw - OFFSET_X, 2)
                dy_comp = round(dy_raw - OFFSET_Y, 2)

                # 参数里的 mode 是 2（虽然内部已强制设为0x02，但保持调用一致性）
                mcu.send_target_data(2, dx_comp, dy_comp)

                # 绘制与显示已移除：仅发送控制命令

            else:
                # 当视觉因为闪烁丢失目标时，发送 0.0 坐标但强制模式依然是 0x02！
                mcu.send_target_data(2, 0.0, 0.0)
                last_point_time = time.time()
            


    finally:
        if mcu.ser and mcu.ser.is_open: mcu.send_target_data(2, 0.0, 0.0); mcu.ser.close()
        cap.release()

if __name__ == "__main__":
    main()