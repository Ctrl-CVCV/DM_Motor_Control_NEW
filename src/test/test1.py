import argparse
import os
import struct
import sys
import time

import cv2
import numpy as np
import serial
import serial.tools.list_ports

try:
    from hobot_dnn import pyeasy_dnn as dnn
except ImportError:
    from hobot_dnn_rdkx5 import pyeasy_dnn as dnn

# === 激光与摄像头物理安装偏差补偿 ===
OFFSET_X = -0.01
OFFSET_Y = 0.06
TARGET_MODE = 0x01

TARGET_CLASS_ALIASES = {"标靶", "target", "bullseye"}


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
            if self.ser:
                self.ser.close()
            self.ser = None
            self.port = "未连接"


def load_class_names(path):
    with open(path, "r", encoding="utf-8") as f:
        return [line.strip() for line in f if line.strip()]


def bgr_to_nv12(frame_bgr, input_w, input_h):
    resized = cv2.resize(frame_bgr, (input_w, input_h), interpolation=cv2.INTER_LINEAR)
    yuv_i420 = cv2.cvtColor(resized, cv2.COLOR_BGR2YUV_I420)
    yuv_i420 = yuv_i420.reshape((input_h * input_w * 3 // 2,))
    y = yuv_i420[: input_h * input_w]
    uv_planar = yuv_i420[input_h * input_w:].reshape((2, input_h * input_w // 4))
    uv_packed = uv_planar.transpose((1, 0)).reshape((input_h * input_w // 2,))
    nv12 = np.empty((input_h * input_w * 3 // 2,), dtype=np.uint8)
    nv12[: input_h * input_w] = y
    nv12[input_h * input_w:] = uv_packed
    return nv12


def letterbox(frame_bgr, new_w, new_h, color=(114, 114, 114)):
    src_h, src_w = frame_bgr.shape[:2]
    scale = min(new_w / src_w, new_h / src_h)
    resize_w, resize_h = int(round(src_w * scale)), int(round(src_h * scale))
    pad_w, pad_h = new_w - resize_w, new_h - resize_h
    left, top = pad_w // 2, pad_h // 2
    resized = cv2.resize(frame_bgr, (resize_w, resize_h), interpolation=cv2.INTER_LINEAR)
    padded = cv2.copyMakeBorder(resized, top, pad_h - top, left, pad_w - left, cv2.BORDER_CONSTANT, value=color)
    return padded, scale, left, top


class RdkYoloV8Detector:
    def __init__(self, model_path, class_names, score_thres=0.25, nms_thres=0.45):
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

    def _decode_outputs(self, outputs, x_scale, y_scale, x_shift, y_shift, src_w, src_h):
        cls_maps = [outputs[0].reshape(-1, self.class_num), outputs[2].reshape(-1, self.class_num), outputs[4].reshape(-1, self.class_num)]
        box_maps = [outputs[1].reshape(-1, self.reg * 4), outputs[3].reshape(-1, self.reg * 4), outputs[5].reshape(-1, self.reg * 4)]

        all_boxes, all_scores, all_class_ids = [], [], []
        for cls_map, box_map, stride, grid in zip(cls_maps, box_maps, self.strides, self.grids):
            max_scores = np.max(cls_map, axis=1)
            selected = np.flatnonzero(max_scores >= self.conf_raw_thres)
            if selected.size == 0:
                continue

            selected_cls = cls_map[selected]
            selected_box = box_map[selected]
            selected_grid = grid[selected]

            class_ids = np.argmax(selected_cls, axis=1)
            scores = 1.0 / (1.0 + np.exp(-np.max(selected_cls, axis=1)))
            ltrb = np.sum(np.exp(selected_box.reshape(-1, 4, self.reg)) / np.sum(np.exp(selected_box.reshape(-1, 4, self.reg)), axis=2, keepdims=True) * self.weights_static, axis=2)
            x1y1 = selected_grid - ltrb[:, 0:2]
            x2y2 = selected_grid + ltrb[:, 2:4]
            boxes = np.hstack([x1y1, x2y2]) * stride

            all_boxes.append(boxes)
            all_scores.append(scores)
            all_class_ids.append(class_ids)

        if not all_boxes:
            return []

        boxes = np.concatenate(all_boxes, axis=0)
        scores = np.concatenate(all_scores, axis=0)
        class_ids = np.concatenate(all_class_ids, axis=0)

        xywh = np.column_stack([boxes[:, 0], boxes[:, 1], boxes[:, 2] - boxes[:, 0], boxes[:, 3] - boxes[:, 1]])
        keep = cv2.dnn.NMSBoxes(xywh.tolist(), scores.tolist(), self.score_thres, self.nms_thres)
        if len(keep) == 0:
            return []

        keep = np.array(keep).reshape(-1)
        detections = []
        for idx in keep:
            x1, y1, x2, y2 = boxes[idx]
            x1 = float(np.clip((x1 - x_shift) / x_scale, 0, src_w - 1))
            y1 = float(np.clip((y1 - y_shift) / y_scale, 0, src_h - 1))
            x2 = float(np.clip((x2 - x_shift) / x_scale, 0, src_w - 1))
            y2 = float(np.clip((y2 - y_shift) / y_scale, 0, src_h - 1))
            if x2 <= x1 or y2 <= y1:
                continue

            detections.append({"class_id": int(class_ids[idx]), "score": float(scores[idx]), "bbox": (x1, y1, x2, y2)})
        return detections

    def infer(self, frame_bgr):
        src_h, src_w = frame_bgr.shape[:2]
        padded, scale, x_shift, y_shift = letterbox(frame_bgr, self.input_w, self.input_h)
        nv12 = bgr_to_nv12(padded, self.input_w, self.input_h)
        outputs = self.model.forward(nv12)
        outputs = [np.array(item.buffer) for item in outputs]
        return self._decode_outputs(outputs, scale, scale, x_shift, y_shift, src_w, src_h)


def open_camera(camera_index, width, height):
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def find_bullseye_detection(detections, class_names):
    target_ids = set()
    for idx, name in enumerate(class_names):
        if str(name).strip().lower() in TARGET_CLASS_ALIASES:
            target_ids.add(idx)

    if not target_ids:
        return None

    target_detections = [det for det in detections if det["class_id"] in target_ids]
    if not target_detections:
        return None

    return max(target_detections, key=lambda det: det["score"])


def main():
    parser = argparse.ArgumentParser(description="YOLO-based bullseye targeting")
    parser.add_argument("--model", default="best_bayese_640x640_nv12.bin")
    parser.add_argument("--classes", default="classes.txt")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = args.model if os.path.isabs(args.model) else os.path.join(script_dir, args.model)
    classes_path = args.classes if os.path.isabs(args.classes) else os.path.join(script_dir, args.classes)

    if not os.path.exists(model_path):
        print(f"[ERROR] 找不到模型文件: {model_path}")
        sys.exit(1)

    if not os.path.exists(classes_path):
        print(f"[ERROR] 找不到类别文件: {classes_path}")
        sys.exit(1)

    class_names = load_class_names(classes_path)
    detector = RdkYoloV8Detector(model_path, class_names)
    mcu = MCUComm(baudrate=115200)

    cap = open_camera(args.camera, args.width, args.height)
    if not cap.isOpened():
        sys.exit(1)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue

            h, w = frame.shape[:2]
            center_x, center_y = w // 2, h // 2

            detections = detector.infer(frame)
            target_det = find_bullseye_detection(detections, class_names)

            if target_det is not None:
                x1, y1, x2, y2 = target_det["bbox"]
                fx = (x1 + x2) / 2.0
                fy = (y1 + y2) / 2.0
                dx_raw = (fx - center_x) / w
                dy_raw = (fy - center_y) / h

                dx_comp = round(dx_raw - OFFSET_X, 2)
                dy_comp = round(dy_raw - OFFSET_Y, 2)

                mcu.send_target_data(TARGET_MODE, dx_comp, dy_comp)
            else:
                mcu.send_target_data(0, 0.0, 0.0)

    except KeyboardInterrupt:
        pass
    finally:
        if mcu.ser and mcu.ser.is_open:
            mcu.send_target_data(0, 0.0, 0.0)
            mcu.ser.close()
        cap.release()


if __name__ == "__main__":
    main()