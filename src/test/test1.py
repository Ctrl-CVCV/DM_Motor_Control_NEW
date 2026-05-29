import cv2
import numpy as np
import serial
import serial.tools.list_ports
import struct
import time
import sys

# === 激光与摄像头物理安装偏差补偿 ===
# 向左微调 0.02，将 -0.01 修改为 0.01
OFFSET_X = 0.01
OFFSET_Y = 0.05

# ==========================================
# 1. MCU 通信模块 (自动重连版)
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
                    print(f"\n✅ MCU 通信连接成功: {self.port}\n")
                    return
            except serial.SerialException:
                pass
        self.ser = None
        self.port = "未连接"

    def check_mcu_feedback(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                raw_data = self.ser.read(self.ser.in_waiting)
                text_data = raw_data.decode('utf-8', errors='ignore')
                if "RECEIVE_OK" in text_data:
                    self.last_receive_time = time.time()
            except Exception:
                pass 

    def is_mcu_alive(self):
        return (time.time() - self.last_receive_time) < 1.0

    def send_target_data(self, mode, dx, dy):
        if self.ser is None or not self.ser.is_open:
            self.connect()
            if self.ser is None: return

        mode_byte = int(mode, 16) if isinstance(mode, str) else int(mode)
        mode_byte &= 0xFF 
        dx, dy = float(dx), float(dy)

        try:
            data = struct.pack('<BBBffBB', 0xAA, 0xFF, mode_byte, dx, dy, 0xFF, 0xAA)
            self.ser.write(data)
            self.check_mcu_feedback()
        except (serial.SerialException, serial.SerialTimeoutException, OSError):
            if self.ser: self.ser.close()
            self.ser = None
            self.port = "未连接"

# ==========================================
# 2. 视觉识别模块
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
        edges = cv2.Canny(blurred, self.canny_lower, self.canny_upper)
        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        valid_centers = [] 
        for c in contours:
            area = cv2.contourArea(c)
            if 50 < area < 80000: 
                peri = cv2.arcLength(c, True)
                if peri == 0: continue
                circularity = 4 * np.pi * (area / (peri * peri))
                if circularity > 0.6: 
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                        valid_centers.append({'center': (cx, cy), 'contour': c, 'area': area})
        best_target, max_circles_count, target_contours = None, 0, []
        for i, data1 in enumerate(valid_centers):
            cx1, cy1 = data1['center']
            current_cluster_contours = [data1['contour']]
            count = 1
            for j, data2 in enumerate(valid_centers):
                if i == j: continue
                cx2, cy2 = data2['center']
                if np.sqrt((cx1 - cx2)**2 + (cy1 - cy2)**2) < self.center_tolerance:
                    if abs(data1['area'] - data2['area']) > 50: 
                        count += 1
                        current_cluster_contours.append(data2['contour'])
            if count > max_circles_count and count >= self.min_concentric_circles:
                max_circles_count = count
                best_target = (cx1, cy1)
                target_contours = current_cluster_contours
        return best_target, target_contours

# ==========================================
# 3. 主程序逻辑
# ==========================================
def main():
    vision = BullseyeVision()
    mcu = MCUComm(baudrate=115200)
    
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened(): sys.exit()

    while True:
        ret, frame = cap.read()
        if not ret: 
            time.sleep(0.1)
            continue
        
        display_frame = frame.copy()
        h, w = display_frame.shape[:2]
        center_x, center_y = w // 2, h // 2

        cv2.line(display_frame, (center_x - 15, center_y), (center_x + 15, center_y), (150, 150, 150), 2)
        cv2.line(display_frame, (center_x, center_y - 15), (center_x, center_y + 15), (150, 150, 150), 2)

        target_center, contours = vision.find_bullseye(frame)
        
        if target_center is not None:
            fx, fy = target_center
            dx_raw = (fx - center_x) / w
            dy_raw = (fy - center_y) / h
            
            dx_comp = round(dx_raw - OFFSET_X, 2)
            dy_comp = round(dy_raw - OFFSET_Y, 2)
            
            mcu.send_target_data(1, dx_comp, dy_comp)
            
            cv2.polylines(display_frame, contours, True, (0, 255, 0), 2)
            cv2.line(display_frame, (fx - 20, fy), (fx + 20, fy), (0, 0, 255), 2)
            cv2.line(display_frame, (fx, fy - 20), (fx, fy + 20), (0, 0, 255), 2)
            cv2.circle(display_frame, target_center, 4, (0, 0, 255), -1)
            cv2.line(display_frame, (center_x, center_y), (fx, fy), (0, 255, 255), 1)
            
            cv2.putText(display_frame, f"MODE: 1 (SCAN)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_frame, f"TX -> dx:{dx_comp:.2f}, dy:{dy_comp:.2f} (COMP)", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        else:
            mcu.send_target_data(0, 0.0, 0.0)
            cv2.putText(display_frame, "MODE: 0 (STOP) - Searching...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(display_frame, "TX -> dx:0.00, dy:0.00", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (150, 150, 150), 2)
        
        if mcu.ser is None:
            cv2.putText(display_frame, "MCU: DISCONNECTED (SEARCHING...)", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        elif mcu.is_mcu_alive():
            cv2.putText(display_frame, f"MCU [{mcu.port}]: ONLINE (RECEIVE_OK)", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display_frame, f"MCU [{mcu.port}]: WAITING REPLY...", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("Bullseye Targeting", display_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    if mcu.ser and mcu.ser.is_open: mcu.send_target_data(0, 0.0, 0.0)
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()