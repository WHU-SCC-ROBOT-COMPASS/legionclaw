#!/usr/bin/env python3
import sys, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomRecorder(Node):
    def __init__(self, path):
        super().__init__("odom_recorder")
        self.f = open(path, "w")
        self.f.write("# timestamp sec x y z qx qy qz qw\n")
        self.sub = self.create_subscription(Odometry, "/odom", self.cb, 10)
    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position; o = msg.pose.pose.orientation
        self.f.write(f"{t:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {o.x:.6f} {o.y:.6f} {o.z:.6f} {o.w:.6f}\n")
        self.f.flush()
    def __del__(self):
        self.f.close()

if __name__ == "__main__":
    rclpy.init()
    r = OdomRecorder(sys.argv[1])
    rclpy.spin(r)
    r.destroy_node(); rclpy.shutdown()
