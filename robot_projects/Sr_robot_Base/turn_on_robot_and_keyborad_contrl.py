import subprocess
import signal
import sys
import os
from PyQt5 import QtWidgets, QtCore
import rclpy
from geometry_msgs.msg import Twist



# 完整參考 wheeltec_keyboard.py 的邏輯
moveBindings = {
	'i': (1, 0),
	'o': (1, -1),
	'j': (0, 1),
	'l': (0, -1),
	'u': (1, 1),
	',': (-1, 0),
	'.': (-1, 1),
	'm': (-1, -1),
}
speedBindings = {
	'q': (1.1, 1.1),
	'z': (0.9, 0.9),
	'w': (1.1, 1),
	'x': (0.9, 1),
	'e': (1, 1.1),
	'c': (1, 0.9),
}


class KeyboardWindow(QtWidgets.QWidget):
	def __init__(self, publisher):
		super().__init__()
		self.setWindowTitle('Wheeltec Robot Keyboard Controller')
		self.setGeometry(100, 100, 400, 200)
		self.label = QtWidgets.QLabel('請直接在此視窗按鍵控制機器人 (i:前進, j/l:左/右, ,:後退, u/o/m/./.:斜向, q/z/w/x/e/c:速度, 空白/k:停, b:模式, ESC:關)', self)
		self.label.setAlignment(QtCore.Qt.AlignCenter)
		layout = QtWidgets.QVBoxLayout()
		layout.addWidget(self.label)
		self.setLayout(layout)
		self.publisher = publisher
		self.speed = 0.2
		self.turn = 1.0
		self.x = 0.0
		self.th = 0.0
		self.count = 0
		self.target_speed = 0.0
		self.target_turn = 0.0
		self.control_speed = 0.0
		self.control_turn = 0.0
		self.Omni = 0
		self.target_HorizonMove = 0.0
		self.control_HorizonMove = 0.0
		self.last_key = ''
		self.last_input_time = QtCore.QTime.currentTime()

		# 定時器：每 100ms 平滑減速
		self.timer = QtCore.QTimer(self)
		self.timer.timeout.connect(self.smooth_stop)
		self.timer.start(100)

	def keyPressEvent(self, event):
		key = event.text()
		qtkey = event.key()
		self.last_input_time = QtCore.QTime.currentTime()
		# 模式切換
		if key == 'b':
			self.Omni = ~self.Omni
			if self.Omni:
				self.label.setText("Switch to OmniMode")
				moveBindings['.'] = [-1, -1]
				moveBindings['m'] = [-1, 1]
			else:
				self.label.setText("Switch to CommonMode")
				moveBindings['.'] = [-1, 1]
				moveBindings['m'] = [-1, -1]
		# 方向鍵
		if key in moveBindings:
			self.x = moveBindings[key][0]
			self.th = moveBindings[key][1]
			self.count = 0
		# 速度調整
		elif key in speedBindings:
			self.speed *= speedBindings[key][0]
			self.turn  *= speedBindings[key][1]
			self.count = 0
			self.label.setText(f'速度: {self.speed:.2f}, 轉向: {self.turn:.2f}')
		# 空白鍵或 k 強制停止
		elif qtkey == QtCore.Qt.Key_Space or key == 'k':
			self.x = 0
			self.th = 0.0
			self.control_speed = 0.0
			self.control_turn = 0.0
			self.control_HorizonMove = 0.0
		# 其他鍵
		else:
			self.count += 1
			if self.count > 4:
				self.x = 0
				self.th = 0.0
			if qtkey == QtCore.Qt.Key_Escape:
				self.close()
				return

		self.update_twist()

	def smooth_stop(self):
		# 若 0.15 秒內沒新按鍵，x/th 歸零
		if self.last_input_time.msecsTo(QtCore.QTime.currentTime()) > 150:
			self.x = 0
			self.th = 0.0
		self.update_twist()

	def update_twist(self):
		# 平滑控制
		self.target_speed = self.speed * self.x
		self.target_turn = self.turn * self.th
		self.target_HorizonMove = self.speed * self.th

		if self.target_speed > self.control_speed:
			self.control_speed = min(self.target_speed, self.control_speed + 0.1)
		elif self.target_speed < self.control_speed:
			self.control_speed = max(self.target_speed, self.control_speed - 0.1)
		else:
			self.control_speed = self.target_speed

		if self.target_turn > self.control_turn:
			self.control_turn = min(self.target_turn, self.control_turn + 0.5)
		elif self.target_turn < self.control_turn:
			self.control_turn = max(self.target_turn, self.control_turn - 0.5)
		else:
			self.control_turn = self.target_turn

		if self.target_HorizonMove > self.control_HorizonMove:
			self.control_HorizonMove = min(self.target_HorizonMove, self.control_HorizonMove + 0.1)
		elif self.target_HorizonMove < self.control_HorizonMove:
			self.control_HorizonMove = max(self.target_HorizonMove, self.control_HorizonMove - 0.1)
		else:
			self.control_HorizonMove = self.target_HorizonMove

		twist = Twist()
		if self.Omni == 0:
			twist.linear.x = self.control_speed
			twist.linear.y = 0.0
			twist.linear.z = 0.0
			twist.angular.x = 0.0
			twist.angular.y = 0.0
			twist.angular.z = self.control_turn
		else:
			twist.linear.x = self.control_speed
			twist.linear.y = self.control_HorizonMove
			twist.linear.z = 0.0
			twist.angular.x = 0.0
			twist.angular.y = 0.0
			twist.angular.z = 0.0
		self.publisher.publish(twist)

def start_qt_keyboard_node():
	rclpy.init()
	from rclpy.node import Node
	node = Node('qt_keyboard_controller')
	pub = node.create_publisher(Twist, 'cmd_vel', 10)
	app = QtWidgets.QApplication(sys.argv)
	win = KeyboardWindow(pub)
	win.show()
	app.exec_()
	# 關閉時自動停止
	twist = Twist()
	pub.publish(twist)
	rclpy.shutdown()

def main():
	# 切換到腳本所在目錄
	script_dir = os.path.dirname(os.path.abspath(__file__))
	os.chdir(script_dir)

	# 啟動 robot node (背景)
	setup_cmd = "source install/setup.bash"
	robot_cmd = f"{setup_cmd} && ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py"
	robot_proc = subprocess.Popen(["bash", "-i", "-c", robot_cmd])

	def shutdown(signum, frame):
		print("\n[INFO] 偵測到中斷，正在結束子進程...")
		robot_proc.terminate()
		try:
			robot_proc.wait(timeout=5)
		except Exception:
			robot_proc.kill()
		sys.exit(0)

	signal.signal(signal.SIGINT, shutdown)
	signal.signal(signal.SIGTERM, shutdown)

	print("[INFO] 已啟動機器人主體，請用下方視窗鍵盤控制")

	# 啟動 Qt5 GUI + rclpy 控制
	start_qt_keyboard_node()

	print("[INFO] GUI 關閉，正在關閉機器人主體...")
	robot_proc.terminate()
	try:
		robot_proc.wait(timeout=5)
	except Exception:
		robot_proc.kill()

if __name__ == "__main__":
	main()
