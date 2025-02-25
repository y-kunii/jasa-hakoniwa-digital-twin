#--- ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
import math

import numpy as np
from pyquaternion import Quaternion

#--- OpenCV
import cv2

#--- PyQt5
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import QThread, pyqtSignal

#--- system
import threading
import sys

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        #--- setting para
        self.resize_scale = 0.25
        self.real_scale_w = 900
        self.real_scale_h = 900

        #--- base picture
        self.init_base_picture()
        self.set_scale()
        self.send_data = Pose()
        self.publisher_ = self.create_publisher(Pose, '/Camera_pose_data', 10)

        self.camera_cap = cv2.VideoCapture(0)
        self.camera_cap.set(cv2.CAP_PROP_EXPOSURE, -1)

        #--- callback
        timer_period = 1  # 秒
        self.timer = self.create_timer(timer_period, self.camera_callback)

    #--- camera callback
    def camera_callback(self, ):
        picture = self.take_capture()
        x, y, z, x_angle, y_angle, z_angle = self.search_angle_position(picture)
        ##self.convert_euler_to_quaternion(x, y, z, x_angle, y_angle, z_angle)
        self.convert_euler_to_euler(x, y, z, x_angle, y_angle, z_angle)
        self.send_pose_date()

    def set_scale(self, ):
        self.area = cv2.imread('/home/jasa/work/template/area_template.png')
        area_h, area_w, channels = self.area.shape
        self.area = cv2.resize(self.area, (int(area_w*self.resize_scale), int(area_h*self.resize_scale)))
        hsv = cv2.cvtColor(self.area, cv2.COLOR_BGR2HSV)
        
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 120, 70])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2
        '''
        lower_red = np.array([75, 100, 100])
        upper_red = np.array([90, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        '''

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        self.area_top_left = (x, y)
        self.area_bottom_right = (x + w, y + h)
        self.x_pixcel_mm = float(self.real_scale_w / w)
        self.y_pixcel_mm = float(self.real_scale_h / h)

        drew_circle = cv2.circle(self.area, (self.area_top_left[0], self.area_top_left[1]), 10, (255, 0, 0), thickness=1)
        drew_circle = cv2.circle(self.area, (self.area_bottom_right[0], self.area_bottom_right[1]), 10, (255, 0, 0), thickness=1)
        cv2.imwrite('/home/jasa/work/template/area_proc.png', drew_circle)

        print(f"template info top_left({self.area_top_left[0]}, {self.area_top_left[1]})") 
        print(f"bottom_right({self.area_bottom_right[0]}, {self.area_bottom_right[1]})")
        print(f"pixcel_size({self.x_pixcel_mm}, {self.y_pixcel_mm})")

    #--- take capture   
    def take_capture(self, ):

        ret, target_pic = self.camera_cap.read()
        #target_pic = cv2.imread('/home/jasa/work/template/test_1.png')
        #target_pic = cv2.cvtColor(target_pic, cv2.COLOR_BGR2GRAY)
        h, w, c = target_pic.shape
        target_pic = cv2.resize(target_pic, (int(w*self.resize_scale), int(h*self.resize_scale)))        
        cv2.imwrite('/home/jasa/work/template/cap.jpg', target_pic)
        #camera_cap.release()
        return target_pic
    
    #--- search angle and position
    def search_angle_position(self, frame):
        h, w, c = self.template_pic.shape
        h_t, w_t, c = frame.shape
        x_position = 0.0
        y_position = 0.0
        z_position = 0.0
        x_angle = 0.0 #固定
        y_angle = 0.0 #固定
        z_angle = 0.0 #固定
        result_temp = -1
        angle = -1
        top_left = None
        #small_template = self.template_pic
        #small_frame = cv2.resize(frame, (int(w_t*1), int(h_t*1)))

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([40, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 120, 70])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        top_contours = contours[:3]
        points = []
        for contour in top_contours:
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])
                point = (cx, cy)
                points.append(point)

        centroid = []
        if len(points) == 3:
            p1 = np.array(points[0])
            p2 = np.array(points[1])
            p3 = np.array(points[2])
            centroid = (p1 + p2 + p3) / 3.0
        elif len(points) == 2:
            p1 = np.array(points[0])
            p2 = np.array(points[1])
            centroid = (p1 + p2) / 2.0
        else:
            print("赤い点が取得できません")
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        drew_circle = cv2.circle(frame, (int(centroid[0]), int(centroid[1])), 10, (255, 0, 0), thickness=1)

        #重心から進行方向へ
        drew_arrow = cv2.arrowedLine(drew_circle, (int(centroid[0]), int(centroid[1])), (int(p1[0]), int(p1[1])), (255, 0, 0), 3)
        
        #角度計算
        vector = centroid - p1
        angle = np.arctan2(vector[1], vector[0])
        angle_degrees = np.degrees(angle) + 90

        cv2.imwrite('/home/jasa/work/template/result_2.jpg', drew_arrow)

        x_position = centroid[0]
        y_position = centroid[1]
        z_position = 0.0
        z_angle = angle_degrees
        '''
        for i in range(0,360,5):
            t3 = self.rotate_target_image(self.template_pic, i)
            result = cv2.matchTemplate(frame, t3, cv2.TM_CCOEFF_NORMED)ss
            min_value, max_value, min_pt, max_pt = cv2.minMaxLoc(result)
        
            print(f"回転{i}{max_value}")

            if max_value > result_temp:
                top_left = max_pt
                angle = i
                result_temp = max_value

        #result = cv2.matchTemplate(frame, self.template_pic, cv2.TM_CCOEFF_NORMED)
        #min_value, max_value, min_pt, max_pt = cv2.minMaxLoc(result)
        #top_left = max_pt
        bottom_right = (top_left[0] + w, top_left[1] + h)
        print(f"top left[{top_left[0]}, {top_left[1]}] bottom_rifht[{bottom_right[0]}, {bottom_right[1]}] angle:{angle}")
        x_position = int((bottom_right[0] - top_left[0]) / 2) + int(top_left[0])
        y_position = int((bottom_right[1] - top_left[1]) / 2) + int(top_left[1])
        print(f"{0},{1}", x_position, y_position)

        drew_circle = cv2.circle(frame, (x_position, y_position), 10, (0, 0, 0), thickness=1)
        drew_circle = cv2.circle(drew_circle, (top_left[0], top_left[1]), 10, (0, 0, 0), thickness=1)
        drew_circle = cv2.circle(drew_circle, (bottom_right[0], bottom_right[1]), 10, (0, 0, 0), thickness=1)
        angle_red = math.radians(angle)
        end_point_x = (int(x_position + 100 * math.cos(angle_red)))
        end_point_y = (int(y_position - 100 * math.sin(angle_red)))
        drew_arrow = cv2.arrowedLine(drew_circle, (x_position, y_position), (end_point_x, end_point_y), (0, 0, 0), 3)

        cv2.imwrite('/home/jasa/work/template/result.jpg', drew_circle)

        z_angle = float(angle)
        #z_angle = 0
        '''
        return x_position, y_position, z_position, 0, 0, z_angle

    #--- convert scalar to quaternion
    def convert_euler_to_quaternion(self, x_position, y_position, z_position, x_angle, y_angle, z_angle):
        qx = Quaternion(np.cos(0/2), np.sin(0/2), 0, 0)
        qy = Quaternion(np.cos(0/2), 0, np.sin(0/2), 0)
        qz = Quaternion(np.cos(z_angle/2), 0, 0, np.sin(z_angle/2))
        q = qz * qy * qx
        self.send_data.position.x = float((x_position - self.area_top_left[0]) * self.x_pixcel_mm / 1000)
        self.send_data.position.y = float((y_position - self.area_top_left[1]) * self.y_pixcel_mm / 1000)
        self.send_data.position.z = float(0.0)
        self.send_data.orientation.x = q.x
        self.send_data.orientation.y = q.y
        self.send_data.orientation.z = q.z
        self.send_data.orientation.w = q.w

    def convert_euler_to_euler(self, x_position, y_position, z_position, x_angle, y_angle, z_angle):
        self.send_data.position.x = float((x_position - self.area_top_left[0]) * self.x_pixcel_mm / 1000)
        self.send_data.position.y = float(0.0)
        self.send_data.position.z = float((y_position - self.area_top_left[1]) * self.y_pixcel_mm * -1 / 1000) 
        self.send_data.orientation.x = 0.0
        self.send_data.orientation.y = z_angle
        #self.send_data.orientation.y = 0.0
        self.send_data.orientation.z = 0.0
        self.send_data.orientation.w = 0.0

    def send_pose_date(self, ):
        print(f"position({self.send_data.position.x },{self.send_data.position.y}, {self.send_data.position.z}) quaternion({self.send_data.orientation.x }, {self.send_data.orientation.y }, {self.send_data.orientation.z }, {self.send_data.orientation.w })")
        self.publisher_.publish(self.send_data)

    #--- init
    def init_base_picture(self, ):
        # honban
        # self.template_pic = self.take_capture()
        self.template_pic = cv2.imread('/home/jasa/work/template/template_3_t4.png')
        #self.template_pic = cv2.cvtColor(self.template_pic, cv2.COLOR_BGR2GRAY)
        h, w, c = self.template_pic.shape
        self.template_pic = cv2.resize(self.template_pic, (int(w*self.resize_scale), int(h*self.resize_scale)))

        cv2.imwrite('/home/jasa/work/template/temp.jpg', self.template_pic)

    def rotate_target_image(self, image, angle):
        height, width = image.shape[:2]
        center = (width // 2, height //2)
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        rotated_image = cv2.warpAffine(image, rotation_matrix, (width, height))
        return rotated_image

def spin_node(node):
    rclpy.spin(node)

#region Main
def main(args=None):
    app = QApplication(sys.argv)
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    camera_publisher.get_logger().info("InfraSensor UP")

    print("Now scanning environments..., please wait.")
    # スレッドを作成して spin を実行
    spin_thread = threading.Thread(target=spin_node, args=(camera_publisher,))
    spin_thread.start()

    # キーボードモニターを設定
    # keyboard_monitor = KeyboardMonitor()
    # keyboard_monitor.mode_changed.connect(lidar_subscriber.set_scan_mode)
    # keyboard_monitor.start()

    # PyQt5のイベントループを実行
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()  
        # keyboard_monitor.terminate()
        camera_publisher.destroy_node()
        spin_thread.join()
      


if __name__ == '__main__':
    main()

#endregion