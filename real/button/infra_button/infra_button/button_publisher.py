#--- ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy

#--- OpenCV
import cv2

#--- PyQt5
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import QThread, pyqtSignal

#--- system
import threading
import sys
import readchar

class ButtonPublisher(Node):
    def __init__(self):
        super().__init__('button_publisher')

        #--- button status
        print("Init")
        self.status_button_a_ = Bool()
        self.status_button_a_.data = False
        self.status_button_b_ = Bool()
        self.status_button_b_.data = False

        self.button_input = 0
        self.lock = threading.Lock()

        self.publisher_button_a_ = self.create_publisher(Bool, '/ButtonA_button_sensor', 10)
        self.publisher_button_b_ = self.create_publisher(Bool, '/ButtonB_button_sensor', 10)

        #--- callback
        timer_period = 1  # 秒
        self.timer = self.create_timer(timer_period, self.camera_callback)

    #--- camera callback
    def camera_callback(self, ):
        print("call buck")
        print(f"input key {self.button_input}")
        
        if self.button_input == 1:
            print("push button A")
            self.status_button_a_.data = True
            self.publisher_button_a_.publish(self.status_button_a_)
        elif self.button_input == 2:
            print("push button B")
            self.status_button_b_.data = True
            self.publisher_button_b_.publish(self.status_button_b_)
        elif self.button_input == 0:
            self.publisher_button_a_.publish(self.status_button_a_)
            self.publisher_button_b_.publish(self.status_button_b_)

        self.button_input = 0
        self.status_button_a_.data = False
        self.status_button_b_.data = False
        #print(f"reset data a:{self.status_button_a_.data}")
        #print(f"reset data b:{self.status_button_b_.data}")
        #print(f"reset data key:{self.button_input}")

    def set_push_button(self, button_input):
        with self.lock:
            self.button_input = button_input


def spin_node(node):
    rclpy.spin(node)

#--- Keybord Monitor ---#
class KeyboardMonitor(QThread):
    mode_changed = pyqtSignal(int)

    def run(self):
        while True:
            #user_input = input('Enter command (1: push button A, 2: bush button B): ')
            user_input = readchar.readchar()
            if user_input == '1':
                self.mode_changed.emit(1)
            elif user_input == '2':
                self.mode_changed.emit(2)                

#region Main
def main(args=None):
    app = QApplication(sys.argv)
    rclpy.init(args=args)
    button_publisher = ButtonPublisher()
    button_publisher.get_logger().info("InfraSensor UP")

    print("Now scanning environments..., please wait.")
    # スレッドを作成して spin を実行
    spin_thread = threading.Thread(target=spin_node, args=(button_publisher,))
    spin_thread.start()

    # キーボードモニターを設定
    keyboard_monitor = KeyboardMonitor()
    keyboard_monitor.mode_changed.connect(button_publisher.set_push_button)
    keyboard_monitor.start()

    # PyQt5のイベントループを実行
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        spin_thread.join()
        keyboard_monitor.terminate()
        button_publisher.destroy_node()


if __name__ == '__main__':
    main()

#endregion