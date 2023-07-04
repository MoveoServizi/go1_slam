#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

class EulerAngles:
    def __init__(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw





class sensor_cheking:
    def ToEulerAngles(self,q):
        angles = EulerAngles(0, 0, 0)

        # roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        angles.roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = math.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
        cosp = math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
        angles.pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

        # yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        angles.yaw = math.atan2(siny_cosp, cosy_cosp)

        return angles

    def __init__(self):
        sub_topic_name ="/scan"
        self.lidar_subscriber = rospy.Subscriber(sub_topic_name, LaserScan, self.lidar_cb)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.position_cb)

    def lidar_cb(self, data):
        region_a = min( min(data.ranges[0:120]) ,  10) #infinity
        region_b = min( min(data.ranges[121:240]), 10)
        region_c = min( min(data.ranges[241:360]), 10)
        print("A :" , round(region_a,3) , " B :", round(region_b,3) , " C ", round(region_c,3) )
    
    def position_cb(self,data):
        q = Quaternion(data.pose.pose.orientation.w,data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z)
        angles = self.ToEulerAngles(q)
        print("posizione \n x ", round(data.pose.pose.position.x,3),"\n y ", round(data.pose.pose.position.y,3),"\n z ", round(data.pose.pose.position.z,3),"\norientazione\n roll ",round(angles.roll,3), "\n pich ",round(angles.pitch,3), "\n yaw ", round(angles.yaw,3), "\n\n" )


if __name__ == '__main__':
    node_name ="lidar_check"
    rospy.init_node(node_name)
    sensor_cheking()
    rospy.spin()