import argparse
import os
import time
import struct
from collections import Counter, deque
import cv2
import numpy as np
import serial
import serial.tools.list_ports

try:
    from hobot_dnn import pyeasy_dnn as dnn
except ImportError:
    from hobot_dnn_rdkx5 import pyeasy_dnn as dnn

# === 激光与摄像头物理安装偏差补偿 ===
OFFSET_X = 0.00
OFFSET_Y = 0.06

# ==========================================
# 0. 坐标平滑与防跳变滤波器 (新增)
# ==========================================
class CoordinateSmoother:
    def __init__(self, alpha=0.2):
        self.alpha = alpha  # 平滑系数，越小越平稳，但跟随会有微小延迟
        self.dx = None
        self.dy = None

    def update(self, new_dx, new_dy):
        # 刚锁定新目标的第一帧，直接初始化
        if self.dx is None or self.dy is None:
            self.dx = new_dx
            self.dy = new_dy
        else:
            # 异常跳变屏蔽：如果瞬间跳动超过20%视场，判定为YOLO识别抽风，直接无视！
            if abs(new_dx - self.dx) > 0.2 or abs(new_dy - self.dy) > 0.2:
                return round(self.dx, 2), round(self.dy, 2)
            
            # EMA平滑：消除小幅度的狂闪抖动
            self.dx = self.alpha * new_dx + (1.0 - self.alpha) * self.dx
            self.dy = self.alpha * new_dy + (1.0 - self.alpha) * self.dy
            
        return round(self.dx, 2), round(self.dy, 2)

    def reset(self):
        """目标切换时清空历史状态"""
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
# 1.5 状态信息通信模块 (ttyUSB0 单字节发送)
# ==========================================
class InfoComm:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        
        # 中英文兼容字典
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
            print(f"📡 [USB0 TX] 发送种类: {class_name} -> 0x{byte_val:02X}")

    def send_finish(self):
        self.send_byte(0xFF)
        print("📡 [USB0 TX] 本轮任务结束 -> 0xFF")

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
            x1, y1 = (x1 - x_shift) / x_scale, (y1 - y_shift) / y_scale
            x2, y2 = (x2 - x_shift) / x_scale, (y2 - y_shift) / y_scale

            x1, y1 = float(np.clip(x1, 0, src_w - 1)), float(np.clip(y1, 0, src_h - 1))
            x2, y2 = float(np.clip(x2, 0, src_w - 1)), float(np.clip(y2, 0, src_h - 1))
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
# 3. 主程序逻辑：从左到右依次扫描打靶
# ==========================================
def main():
    print("\n" + "="*50)
    print("🚀 启动【多设备双串口+坐标补偿平滑防抖】扫描任务")
    
    sequence_file = "target_sequence.txt"
    if os.path.exists(sequence_file):
        os.remove(sequence_file)
        print(f"🧹 启动清理: 已成功清除历史遗留文件 '{sequence_file}'")
    else:
        print(f"✨ 启动清理: 环境干净，无遗留文件。")
    print("="*50 + "\n")

    parser = argparse.ArgumentParser(description="RDKX5 YOLOv8 Sequential Targeting")
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
    smoother = CoordinateSmoother(alpha=0.2)  # 实例化平滑滤波器
    class_vote_window = 7
    class_vote_threshold = 5
    current_class_votes = deque(maxlen=class_vote_window)
    current_target_class_sent = False
    
    cap = open_camera(args.camera, args.width, args.height)
    if not cap.isOpened():
        raise RuntimeError("Failed to open USB camera")

    in_scanning_round = False
    target_index = 0
    lock_start_time = 0.0
    stay_duration = 10.0
    scanned_history = []
    round_target_count = 0  

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            frame_h, frame_w = frame.shape[:2]
            center_x, center_y = frame_w // 2, frame_h // 2

            detections = detector.infer(frame)
            for det in detections:
                x1, y1, x2, y2 = det["bbox"]
                det["cx"], det["cy"] = (x1 + x2) / 2.0, (y1 + y2) / 2.0

            detections.sort(key=lambda d: d["cx"])

            if not in_scanning_round:
                if len(detections) > 0:
                    in_scanning_round = True
                    target_index = 0
                    lock_start_time = time.time()
                    scanned_history = []
                    smoother.reset() # 新回合开始，清空滤波器
                    current_class_votes.clear()
                    current_target_class_sent = False
                    round_target_count = len(detections) 
                    print(f"\n🎬 发现 {round_target_count} 个目标，开始新一轮从左到右扫描！")
                else:
                    mcu.send_target_data(0, 0, 0)

            if in_scanning_round:
                if target_index >= round_target_count:
                    print("\n🏆 【本轮扫描结束】汇总：", " -> ".join(scanned_history))
                    try:
                        with open(sequence_file, "w", encoding="utf-8") as f:
                            for item in scanned_history:
                                f.write(item + "\n")
                        print(f"💾 扫描顺序已成功保存至 {sequence_file}！")
                    except Exception as e:
                        print(f"❌ 保存序列失败: {e}")

                    mcu.send_target_data(0, 0, 0)
                    info_mcu.send_finish()
                    in_scanning_round = False
                    current_class_votes.clear()
                    current_target_class_sent = False
                    break  # 完成一轮扫描后退出
                
                else:
                    if len(detections) > 0:
                        current_idx = min(target_index, len(detections) - 1)
                        target = detections[current_idx]
                        
                        cx, cy = target["cx"], target["cy"]
                        class_id = target["class_id"]
                        obj_name = class_names[class_id] if class_id < len(class_names) else f"OBJ_{class_id}"

                        if not current_target_class_sent:
                            current_class_votes.append(obj_name)
                            vote_counter = Counter(current_class_votes)
                            vote_name, vote_count = vote_counter.most_common(1)[0]

                            if len(current_class_votes) >= class_vote_window and vote_count >= class_vote_threshold:
                                scanned_history.append(vote_name)
                                current_target_class_sent = True
                                print(f"👉 [多帧确认] 第 {target_index + 1}/{round_target_count} 个物体: {vote_name} (votes={vote_count}/{len(current_class_votes)})")
                                info_mcu.send_class_info(vote_name)

                        # =============================================
                        # 引入防跳变和阻尼器计算
                        # =============================================
                        dx_raw = (cx - center_x) / frame_w
                        dy_raw = (cy - center_y) / frame_h
                        
                        dx_comp_raw = dx_raw - OFFSET_X
                        dy_comp_raw = dy_raw - OFFSET_Y
                        
                        # 把原始含噪补偿坐标丢给 Smoother，输出稳定坐标
                        dx_comp, dy_comp = smoother.update(dx_comp_raw, dy_comp_raw)
                        
                        mcu.send_target_data(1, dx_comp, dy_comp)
                        # =============================================

                        time_elapsed = time.time() - lock_start_time
                        time_left = max(0.0, stay_duration - time_elapsed)

                        if time_elapsed >= stay_duration:
                            target_index += 1
                            lock_start_time = time.time()
                            smoother.reset() # 切换下一个目标时重置滤波器，避免拖泥带水
                            current_class_votes.clear()
                            current_target_class_sent = False
                    else:
                        mcu.send_target_data(0, 0, 0)

                # 绘制与显示已移除：保持检测逻辑不变，仅发送控制命令



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