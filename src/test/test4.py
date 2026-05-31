import argparse
import os
import time
import struct
import cv2
import numpy as np
import serial
import serial.tools.list_ports
import sys
import math

try:
    from hobot_dnn import pyeasy_dnn as dnn
except ImportError:
    from hobot_dnn_rdkx5 import pyeasy_dnn as dnn

# === 激光与摄像头物理安装偏差补偿 ===
OFFSET_X = 0.01
OFFSET_Y = 0.06

# ==========================================
# 0. 坐标平滑与防跳变滤波器
# ==========================================
class CoordinateSmoother:
    def __init__(self, alpha=0.2):
        self.alpha = alpha  
        self.dx = None
        self.dy = None

    def update(self, new_dx, new_dy):
        if self.dx is None or self.dy is None:
            self.dx = new_dx
            self.dy = new_dy
        else:
            if abs(new_dx - self.dx) > 0.2 or abs(new_dy - self.dy) > 0.2:
                return round(self.dx, 2), round(self.dy, 2)
            
            self.dx = self.alpha * new_dx + (1.0 - self.alpha) * self.dx
            self.dy = self.alpha * new_dy + (1.0 - self.alpha) * self.dy
            
        return round(self.dx, 2), round(self.dy, 2)

    def reset(self):
        self.dx = None
        self.dy = None

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
            if self.ser is None:
                return

        mode_byte = int(mode, 16) if isinstance(mode, str) else int(mode)
        mode_byte &= 0xFF 
        try:
            self.ser.write(struct.pack('<BBBffBB', 0xAA, 0xFF, mode_byte, float(dx), float(dy), 0xFF, 0xAA))
            self.check_mcu_feedback()
        except (serial.SerialException, serial.SerialTimeoutException, OSError):
            if self.ser: self.ser.close()
            self.ser = None
            self.port = "未连接"

# ==========================================
# 1.5 状态信息通信模块 (ttyUSB0 单字节发送)
# ==========================================
class InfoComm:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        
        self.class_mapping = {
            "梯形": 0x01, "trapezoid": 0x01,
            "正方形": 0x02, "square": 0x02,
            "三角形": 0x03, "triangle": 0x03,
            "草莓": 0x04, "strawberry": 0x04,
            "苹果": 0x05, "apple": 0x05,
            "西瓜": 0x06, "watermelon": 0x06,
            "香蕉": 0x07, "banana": 0x07,
            "标靶": 0x08, "target": 0x08, "bullseye": 0x08
        }
        self.connect()

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1, write_timeout=0.1)
            if self.ser.is_open:
                print(f"✅ 状态信息连接成功: {self.port}")
        except Exception:
            self.ser = None

    def send_byte(self, byte_val):
        if self.ser is None or not self.ser.is_open:
            self.connect()
            if self.ser is None: return

        try:
            self.ser.write(struct.pack('B', byte_val))
        except Exception:
            if self.ser: self.ser.close()
            self.ser = None

    def send_class_info(self, class_name):
        byte_val = self.class_mapping.get(str(class_name).lower(), 0x00)
        if byte_val != 0x00:
            self.send_byte(byte_val)

    def send_finish(self):
        self.send_byte(0xFF)

# ==========================================
# 2. YOLOv8 BPU 模型推理相关函数与类
# ==========================================
def softmax(x: np.ndarray, axis: int = -1) -> np.ndarray:
    x = x - np.max(x, axis=axis, keepdims=True)
    exp_x = np.exp(x)
    return exp_x / np.sum(exp_x, axis=axis, keepdims=True)

def load_class_names(path: str) -> list[str]:
    with open(path, "r", encoding="utf-8") as f:
        return [line.strip() for line in f if line.strip()]

def bgr_to_nv12(frame_bgr: np.ndarray, input_w: int, input_h: int):
    resized = cv2.resize(frame_bgr, (input_w, input_h), interpolation=cv2.INTER_LINEAR)
    yuv_i420 = cv2.cvtColor(resized, cv2.COLOR_BGR2YUV_I420)
    yuv_i420 = yuv_i420.reshape((input_h * input_w * 3 // 2,))
    y = yuv_i420[: input_h * input_w]
    uv_planar = yuv_i420[input_h * input_w :].reshape((2, input_h * input_w // 4))
    uv_packed = uv_planar.transpose((1, 0)).reshape((input_h * input_w // 2,))
    nv12 = np.empty((input_h * input_w * 3 // 2,), dtype=np.uint8)
    nv12[: input_h * input_w] = y
    nv12[input_h * input_w :] = uv_packed
    return nv12

def letterbox(frame_bgr: np.ndarray, new_w: int, new_h: int, color=(114, 114, 114)):
    src_h, src_w = frame_bgr.shape[:2]
    scale = min(new_w / src_w, new_h / src_h)
    resize_w, resize_h = int(round(src_w * scale)), int(round(src_h * scale))
    pad_w, pad_h = new_w - resize_w, new_h - resize_h
    left, top = pad_w // 2, pad_h // 2
    resized = cv2.resize(frame_bgr, (resize_w, resize_h), interpolation=cv2.INTER_LINEAR)
    padded = cv2.copyMakeBorder(resized, top, pad_h - top, left, pad_w - left, cv2.BORDER_CONSTANT, value=color)
    return padded, scale, left, top

class RdkYoloV8Detector:
    def __init__(self, model_path: str, class_names: list[str], score_thres: float = 0.25, nms_thres: float = 0.45):
        self.models = dnn.load(model_path)
        self.model = self.models[0]
        self.class_names = class_names
        self.score_thres = score_thres
        self.nms_thres = nms_thres
        self.reg = 16
        self.class_num = len(class_names)
        self.input_h = int(self.model.inputs[0].properties.shape[2])
        self.input_w = int(self.model.inputs[0].properties.shape[3])
        self.conf_raw_thres = -np.log(1.0 / self.score_thres - 1.0)
        self.weights_static = np.arange(self.reg, dtype=np.float32)[np.newaxis, np.newaxis, :]
        self.strides = [8, 16, 32]
        self.grids = self._build_grids()

    def _build_grids(self):
        grids = []
        for stride in self.strides:
            grid_h, grid_w = self.input_h // stride, self.input_w // stride
            yy, xx = np.meshgrid(np.arange(grid_h, dtype=np.float32) + 0.5, np.arange(grid_w, dtype=np.float32) + 0.5, indexing="ij")
            grids.append(np.stack([xx, yy], axis=-1).reshape(-1, 2))
        return grids

    def _decode_outputs(self, outputs: list[np.ndarray], x_scale: float, y_scale: float, x_shift: int, y_shift: int, src_w: int, src_h: int):
        cls_maps = [outputs[0].reshape(-1, self.class_num), outputs[2].reshape(-1, self.class_num), outputs[4].reshape(-1, self.class_num)]
        box_maps = [outputs[1].reshape(-1, self.reg * 4), outputs[3].reshape(-1, self.reg * 4), outputs[5].reshape(-1, self.reg * 4)]

        all_boxes, all_scores, all_class_ids = [], [], []
        for cls_map, box_map, stride, grid in zip(cls_maps, box_maps, self.strides, self.grids):
            max_scores = np.max(cls_map, axis=1)
            selected = np.flatnonzero(max_scores >= self.conf_raw_thres)
            if selected.size == 0: continue

            selected_cls = cls_map[selected]
            selected_box = box_map[selected]
            selected_grid = grid[selected]

            class_ids = np.argmax(selected_cls, axis=1)
            scores = 1.0 / (1.0 + np.exp(-np.max(selected_cls, axis=1)))
            ltrb = np.sum(softmax(selected_box.reshape(-1, 4, self.reg), axis=2) * self.weights_static, axis=2)
            x1y1 = selected_grid - ltrb[:, 0:2]
            x2y2 = selected_grid + ltrb[:, 2:4]
            boxes = np.hstack([x1y1, x2y2]) * stride

            all_boxes.append(boxes)
            all_scores.append(scores)
            all_class_ids.append(class_ids)

        if not all_boxes: return []

        boxes = np.concatenate(all_boxes, axis=0)
        scores = np.concatenate(all_scores, axis=0)
        class_ids = np.concatenate(all_class_ids, axis=0)

        xywh = np.column_stack([boxes[:, 0], boxes[:, 1], boxes[:, 2] - boxes[:, 0], boxes[:, 3] - boxes[:, 1]])
        keep = cv2.dnn.NMSBoxes(xywh.tolist(), scores.tolist(), self.score_thres, self.nms_thres)
        
        if len(keep) == 0: return []
        keep = np.array(keep).reshape(-1)
        
        detections = []
        for idx in keep:
            x1, y1, x2, y2 = boxes[idx]
            x1, y1 = float(np.clip((x1 - x_shift) / x_scale, 0, src_w - 1)), float(np.clip((y1 - y_shift) / y_scale, 0, src_h - 1))
            x2, y2 = float(np.clip((x2 - x_shift) / x_scale, 0, src_w - 1)), float(np.clip((y2 - y_shift) / y_scale, 0, src_h - 1))
            if x2 <= x1 or y2 <= y1: continue

            detections.append({"class_id": int(class_ids[idx]), "score": float(scores[idx]), "bbox": (x1, y1, x2, y2)})
        return detections

    def infer(self, frame_bgr: np.ndarray):
        src_h, src_w = frame_bgr.shape[:2]
        padded, scale, x_shift, y_shift = letterbox(frame_bgr, self.input_w, self.input_h)
        nv12 = bgr_to_nv12(padded, self.input_w, self.input_h)
        outputs = self.model.forward(nv12)
        outputs = [np.array(item.buffer) for item in outputs]
        return self._decode_outputs(outputs, scale, scale, x_shift, y_shift, src_w, src_h)

def open_camera(camera_index: int, width: int, height: int):
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    if not cap.isOpened(): cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap

# ==========================================
# 3. 主程序逻辑：读取记忆并【空间锁定】重放
# ==========================================
def main():
    print("\n" + "="*50)
    print(f"🚀 启动【发挥部分2: 记忆读取与空间盲打】任务")
    
    sequence_file = "target_sequence.txt"
    if not os.path.exists(sequence_file):
        print(f"\n❌ 严重错误: 未找到记忆文件 '{sequence_file}'！")
        print("💡 请先运行 test2.py 从左到右扫描一次，生成记忆后再运行 test4.py。")
        print("="*50 + "\n")
        sys.exit()

    try:
        with open(sequence_file, "r", encoding="utf-8") as f:
            round_sequence = [line.strip() for line in f if line.strip()]
    except Exception as e:
        print(f"❌ 读取序列文件出错: {e}")
        sys.exit()

    if len(round_sequence) == 0:
        print("\n❌ 严重错误: 记忆序列为空！请重新运行 test2.py。\n")
        sys.exit()

    print(f"📄 成功加载记忆顺序: {' -> '.join(round_sequence)}")
    print("="*50 + "\n")

    parser = argparse.ArgumentParser(description="RDKX5 YOLOv8 Sequence Playback")
    parser.add_argument("--model", default="best_bayese_640x640_nv12.bin")
    parser.add_argument("--classes", default="classes.txt")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    class_names = load_class_names(args.classes)
    detector = RdkYoloV8Detector(args.model, class_names)
    
    mcu = MCUComm(baudrate=115200)
    info_mcu = InfoComm(port='/dev/ttyUSB0')
    smoother = CoordinateSmoother(alpha=0.2)
    
    cap = open_camera(args.camera, args.width, args.height)
    if not cap.isOpened():
        raise RuntimeError("Failed to open USB camera")

    # === 重放与空间锁定 状态机变量 ===
    target_index = 0
    round_target_count = len(round_sequence)
    stay_duration = 15.0 
    
    # 核心：空间锁定功能
    target_locked = False
    locked_cx = 0.0
    locked_cy = 0.0
    locked_bbox = (0, 0, 0, 0)
    TRACKING_THRESHOLD = 150.0 # 允许目标物理移动的最大像素偏移量
    
    accumulated_time = 0.0 
    last_frame_time = time.time()
    has_sent_usb_info = False

    try:
        while True:
            current_time = time.time()
            dt = current_time - last_frame_time
            last_frame_time = current_time

            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            frame_h, frame_w = frame.shape[:2]
            center_x, center_y = frame_w // 2, frame_h // 2

            detections = detector.infer(frame)
            
            # --- 验证是否所有任务完成 ---
            if target_index >= round_target_count:
                mcu.send_target_data(0, 0, 0)
                continue

            expected_target_name = round_sequence[target_index]
            
            # ====================================================
            # 阶段 A：寻找阶段 (按类别名寻找)
            # ====================================================
            if not target_locked:
                matched_det = None
                for det in detections:
                    cid = det["class_id"]
                    name = class_names[cid] if cid < len(class_names) else f"OBJ_{cid}"
                    if name == expected_target_name:
                        matched_det = det
                        break

                if matched_det is not None:
                    # 找到了！瞬间锁定其空间坐标
                    target_locked = True
                    x1, y1, x2, y2 = matched_det["bbox"]
                    locked_cx = (x1 + x2) / 2.0
                    locked_cy = (y1 + y2) / 2.0
                    locked_bbox = matched_det["bbox"]
                    
                    if not has_sent_usb_info:
                        print(f"👉 [空间锁定] 目标 {target_index + 1}/{round_target_count}: {expected_target_name}")
                        info_mcu.send_class_info(expected_target_name)
                        has_sent_usb_info = True
                        smoother.reset()
                else:
                    # 还没找到，云台停转，等待目标进入视野
                    mcu.send_target_data(0, 0, 0)

            # ====================================================
            # 阶段 B：盲眼跟踪阶段 (无视类别变异，死盯坐标)
            # ====================================================
            if target_locked:
                # 累加激光照射时间
                accumulated_time += dt

                # 寻找距离 locked_cx, locked_cy 最近的物体框 (完全无视它的种类名字！)
                closest_det = None
                min_dist = float('inf')

                for det in detections:
                    x1, y1, x2, y2 = det["bbox"]
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    dist = math.hypot(cx - locked_cx, cy - locked_cy)
                    
                    # 只要距离在阈值内(允许目标板缓慢移动)，就认为是同一个物体
                    if dist < min_dist and dist < TRACKING_THRESHOLD:
                        min_dist = dist
                        closest_det = det
                
                if closest_det is not None:
                    # 动态更新坐标 (使得激光能跟随目标板的移动)
                    x1, y1, x2, y2 = closest_det["bbox"]
                    locked_cx = (x1 + x2) / 2.0
                    locked_cy = (y1 + y2) / 2.0
                    locked_bbox = closest_det["bbox"]
                else:
                    # 激光强光导致 YOLO 连框都看不到了！
                    # 没关系，直接使用上一帧的 locked_cx 和 locked_cy
                    # 因为时间在累加，激光会保持打在原来的位置上
                    pass

                # 计算坐标与偏差补偿
                dx_raw = (locked_cx - center_x) / frame_w
                dy_raw = (locked_cy - center_y) / frame_h
                
                dx_comp_raw = dx_raw - OFFSET_X
                dy_comp_raw = dy_raw - OFFSET_Y
                
                # 经过 EMA 和跳变屏蔽滤波器
                dx_comp, dy_comp = smoother.update(dx_comp_raw, dy_comp_raw)
                
                mcu.send_target_data(1, dx_comp, dy_comp)

                time_left = max(0.0, stay_duration - accumulated_time)

                # 15秒照射时间到达，推进任务
                if accumulated_time >= stay_duration:
                    print(f"✅ {expected_target_name} 盲打完成！")
                    target_index += 1
                    target_locked = False
                    accumulated_time = 0.0
                    has_sent_usb_info = False
                    smoother.reset()
                    
                    if target_index >= round_target_count:
                        print("\n🏁 所有记忆目标已被全数击破！")
                        info_mcu.send_finish()

            # 已移除所有画面绘制，保留检测与控制逻辑



    finally:
        if mcu.ser and mcu.ser.is_open:
            mcu.send_target_data(0, 0, 0)
            mcu.ser.close()
        if info_mcu.ser and info_mcu.ser.is_open:
            info_mcu.send_finish()
            info_mcu.ser.close()
        cap.release()

if __name__ == "__main__":
    main()