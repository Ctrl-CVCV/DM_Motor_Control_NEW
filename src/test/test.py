import os
import signal
import subprocess
import sys
import time

import serial


SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

SCRIPT_MAP = {
	0x01: "test1.py",
	0x02: "test2.py",
	0x03: "test1.py",
	0x04: "test3.py",
	0x05: "test4.py",
}


def script_path(script_name):
	return os.path.join(os.path.dirname(os.path.abspath(__file__)), script_name)


def start_script(script_name):
	path = script_path(script_name)
	if not os.path.exists(path):
		print(f"[WARN] 找不到脚本: {path}")
		return None

	print(f"[INFO] 启动脚本: {script_name}")
	return subprocess.Popen([sys.executable, path])


def stop_process(process):
	if process is None:
		return

	if process.poll() is not None:
		return

	try:
		process.terminate()
		process.wait(timeout=2)
	except subprocess.TimeoutExpired:
		process.kill()
	except Exception:
		pass


def open_serial():
	while True:
		try:
			ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
			print(f"[INFO] 已连接串口: {SERIAL_PORT}")
			return ser
		except serial.SerialException:
			print(f"[WARN] 无法打开 {SERIAL_PORT}，1 秒后重试...")
			time.sleep(1)


def main():
	current_process = None
	current_script = None
	ser = open_serial()

	try:
		while True:
			if not ser.is_open:
				ser.close()
				ser = open_serial()

			if ser.in_waiting > 0:
				data = ser.read(ser.in_waiting)
				for byte_value in data:
					script_name = SCRIPT_MAP.get(byte_value)
					if script_name is None:
						continue

					if current_script != script_name:
						stop_process(current_process)
						current_process = start_script(script_name)
						current_script = script_name

			time.sleep(0.02)

	except KeyboardInterrupt:
		print("\n[INFO] 收到退出信号，正在清理...")
	finally:
		stop_process(current_process)
		try:
			if ser and ser.is_open:
				ser.close()
		except Exception:
			pass


if __name__ == "__main__":
	main()
