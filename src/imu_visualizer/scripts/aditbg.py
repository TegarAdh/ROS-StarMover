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
        self.udp_ip = "0.0.0.0"
        self.udp_port = 4321  # <- Port disamakan dengan ESP32
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(0.01)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = rospy.Time.now().to_sec()
        self.speed = 0.1
        self.side_length = 1.0

        # ROS Publishers
        self.pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=10, latch=True)
        self.command_pub = rospy.Publisher('/hexagon_commands', String, queue_size=10)

        # ROS Subscriber
        rospy.Subscriber('/hexagon_commands', String, self.command_callback)

        # Path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        rospy.loginfo("Hexagon Controller Ready")

    def send_command(self, command):
        try:
            self.sock.sendto(command.encode(), (self.esp32_ip, self.esp32_port))
            rospy.loginfo(f"Sent: {command}")
        except Exception as e:
            rospy.logwarn(f"send_command error: {e}")

    def command_callback(self, msg):
        if msg.data in ["start", "stop", "reset"]:
            self.send_command(msg.data.upper())

    def parse_udp(self, data):
        try:
            decoded = data.decode().strip()
            parts = decoded.split(',')
            yaw = float(parts[0].split(':')[1])
            step = float(parts[1].split(':')[1])
            moving = step < 5
            # Print yaw to terminal
            rospy.loginfo(f"Received Yaw: {yaw:.2f}°, Step: {step:.2f}")
            return yaw, moving
        except Exception as e:
            rospy.logwarn(f"UDP parse error: {e}")
            return None, None

    def update_pose(self, yaw_deg, is_moving):
        now = rospy.Time.now().to_sec()
        dt = now - self.last_time
        self.last_time = now

        if is_moving and dt > 0:
            yaw_rad = math.radians(yaw_deg)
            self.x += self.speed * math.cos(yaw_rad) * dt
            self.y += self.speed * math.sin(yaw_rad) * dt
        self.yaw = yaw_deg

    def create_pose_msg(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, math.radians(self.yaw))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                data, _ = self.sock.recvfrom(1024)
                yaw, moving = self.parse_udp(data)
                if yaw is not None:
                    self.update_pose(yaw, moving)
                    pose = self.create_pose_msg()
                    self.pose_pub.publish(pose)

                    if not self.path_msg.poses or \
                       math.hypot(pose.pose.position.x - self.path_msg.poses[-1].pose.position.x,
                                  pose.pose.position.y - self.path_msg.poses[-1].pose.position.y) > 0.05:
                        self.path_msg.poses.append(pose)
                        self.path_pub.publish(self.path_msg)

            except socket.timeout:
                pass
            except Exception as e:
                rospy.logerr(f"Main loop error: {e}")
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = HexagonController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.sock.close()

