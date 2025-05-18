#!/usr/bin/env python3
import rospy
import socket
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String

class HexagonController:
    def __init__(self):
        rospy.init_node('hexagon_controller')
        
        # UDP Configuration
        self.udp_ip = "0.0.0.0"  # Listen to all interfaces
        self.udp_port = 4321  # Port yang digunakan untuk menerima data IMU
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(0.01)  # Timeout 10ms
        
        # Robot state initialization
        self.x = 0.0  # Posisi x robot
        self.y = 0.0  # Posisi y robot
        self.yaw = 0.0  # Arah robot (yaw)
        self.last_time = rospy.Time.now().to_sec()  # Waktu saat terakhir menerima data IMU
        self.speed = 0.1  # Kecepatan pergerakan robot
        self.side_length = 1.0  # Panjang sisi hexagon (bisa disesuaikan)

        # ROS Publishers
        self.pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=10, latch=True)
        self.command_pub = rospy.Publisher('/hexagon_commands', String, queue_size=10)
        
        # ROS Subscriber
        rospy.Subscriber('/hexagon_commands', String, self.command_callback)
        
        # Path message initialization
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"  # Frame yang digunakan di RViz
        rospy.loginfo("Hexagon Controller Ready")

    def send_command(self, command):
        try:
            self.sock.sendto(command.encode(), (self.esp32_ip, self.esp32_port))
            rospy.loginfo(f"Sent: {command}")
        except Exception as e:
            rospy.logerr(f"Command error: {e}")

    def command_callback(self, msg):
        if msg.data in ["start", "stop", "reset"]:
            self.send_command(msg.data.upper())

    def parse_udp(self, data):
        try:
            decoded = data.decode().strip()  # Decode data dari UDP
            if decoded.startswith("ANGLE:"):
                parts = decoded.split(',')
                yaw = float(parts[0].split(':')[1])  # Parsing yaw
                moving = parts[1].split(':')[1] == "1" if len(parts) > 1 else False  # Parsing status moving
                return yaw, moving
        except Exception as e:
            rospy.logwarn(f"UDP error: {e}")
        return None, None

    def update_pose(self, yaw_deg, is_moving):
        # Menghitung posisi berdasarkan yaw dan status gerakan
        now = rospy.Time.now().to_sec()
        dt = now - self.last_time  # Waktu delta antara pembaruan pose
        self.last_time = now
        
        if is_moving and dt > 0:
            yaw_rad = math.radians(yaw_deg)
            self.x += self.speed * math.cos(yaw_rad) * dt  # Update posisi x
            self.y += self.speed * math.sin(yaw_rad) * dt  # Update posisi y
        self.yaw = yaw_deg

    def create_pose_msg(self):
        # Membuat pesan PoseStamped untuk robot
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"  # Frame yang digunakan di RViz
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0  # Z=0, asumsikan robot bergerak di dataran datar
        q = quaternion_from_euler(0, 0, math.radians(self.yaw))  # Mengubah yaw menjadi quaternion
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def run(self):
        # Loop utama yang terus berjalan selama ROS node aktif
        rate = rospy.Rate(50)  # Set rate 50Hz
        while not rospy.is_shutdown():
            try:
                data, _ = self.sock.recvfrom(1024)  # Menerima data UDP
                yaw, moving = self.parse_udp(data)  # Parse data IMU
                if yaw is not None:
                    self.update_pose(yaw, moving)  # Update posisi berdasarkan yaw
                    pose = self.create_pose_msg()  # Membuat pesan pose terbaru
                    self.pose_pub.publish(pose)  # Publish pose ke ROS

                    # Update path jika posisi berubah lebih dari 0.05 meter
                    if not self.path_msg.poses or \
                       math.sqrt((pose.pose.position.x - self.path_msg.poses[-1].pose.position.x)**2 + 
                                 (pose.pose.position.y - self.path_msg.poses[-1].pose.position.y)**2) > 0.05:
                        self.path_msg.poses.append(pose)
                        self.path_pub.publish(self.path_msg)  # Publish path ke ROS
            except socket.timeout:
                pass  # Timeout jika tidak ada data UDP yang diterima
            rate.sleep()  # Delay untuk mencapai rate yang diinginkan

if __name__ == '__main__':
    try:
        controller = HexagonController()  # Inisialisasi HexagonController
        controller.run()  # Jalankan node
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.sock.close()  # Pastikan soket ditutup setelah selesai

