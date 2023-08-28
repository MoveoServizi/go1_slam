#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np
#from map_creation import map_creation
from threading import Thread
import matplotlib.pyplot as plt

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
    def __init__(self):
        sub_topic_name ="/scan"
        self.lidar_subscriber = rospy.Subscriber(sub_topic_name, LaserScan, self.lidar_cb)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.position_cb)
        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

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



    def lidar_cb(self, data):
        x = self.x
        y = self.y
        z = self.z
        pos = [x,y,z]
        roll = self.roll
        pitch = self.pitch
        yaw = self.yaw
        region_a = min( min(data.ranges[0:120]) ,  10) #infinity
        region_b = min( min(data.ranges[121:240]), 10)
        region_c = min( min(data.ranges[241:360]), 10)
        print("A :" , round(region_a,3) , " B :", round(region_b,3) , " C ", round(region_c,3) )

        list_points = []
        for i in range(360):                    # Index in this array corresponds to the angle of the laser
            list_points.append( min(data.ranges[i],16))


        obs = self.Obstacles(list_points,roll,pitch,yaw,pos)
        self.plot_map(obs)
    
    def position_cb(self,data):
        q = Quaternion(data.pose.pose.orientation.w,data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z)
        angles = self.ToEulerAngles(q)
        self.x = round(data.pose.pose.position.x,8)
        self.y = round(data.pose.pose.position.y,8),
        self.z = round(data.pose.pose.position.z,8)
        self.roll = round(angles.roll,8)
        self.pitch = round(angles.pitch,8)
        self.yaw = round(angles.yaw,8)
        print("posizione \n x ", round(data.pose.pose.position.x,3),"\n y ", round(data.pose.pose.position.y,3),"\n z ", round(data.pose.pose.position.z,3),"\norientazione\n roll ",round(angles.roll,3), "\n pich ",round(angles.pitch,3), "\n yaw ", round(angles.yaw,3), "\n\n" )

    def rot_RPYbody2NED(self,roll,pitch,yaw,vett):
        #matrice di rotazione


        R_x_roll  =  np.matrix([[1,        0,             0 ,       0],\
                            [0,    np.cos(roll),  np.sin(roll),     0],\
                            [0,    -np.sin(roll), np.cos(roll),     0],\
                            [0 ,    0,              0 ,             1]])
        
        R_y_pitch = np.matrix([[np.cos(pitch),  0,   -np.sin(pitch),   0],\
                            [       0,          1,          0,         0],\
                            [np.sin(pitch),     0,    np.cos(pitch),   0],\
                            [       0,          0,          0,         1]])
        
        R_z_yaw   = np.matrix([[np.cos(yaw),  np.sin(yaw),    0,    0],\
                            [-np.sin(yaw),  np.cos(yaw),      0,    0],\
                            [      0,           0,            1,    0],\
                            [      0,           0,            0,    1]])
        
        R_transl  =  np.matrix([[1,        0,             0,          0.3],\
                            [0,            1,             0,          0],\
                            [0,            0,             1,          0.435],\
                            [0 ,           0,             0,          1]])

          
        ans = np.matmul(R_x_roll,R_y_pitch)
        ans = np.matmul(ans,R_z_yaw)
        ans = np.matmul(ans.T,R_transl)
        ans = np.matmul(ans,vett.T)
        return ans
        #return (R_z_yaw*R_y_pitch*R_x_roll*R_transl).T*vett.T
        #return (R_x_roll*R_y_pitch*R_z_yaw*R_transl).T*vett.T

    def Obstacles(self,lidar_data,roll,pitch,yaw,pos_NED):
        LiDar = 15
        self.lidar_data = lidar_data
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.pos_NED = pos_NED
        obs = np.array([None,None,None])
        

        for i in range(len(lidar_data)-1):
            if lidar_data[i]<LiDar:
                x_l =  lidar_data[i]*(np.cos( i*(360/len(lidar_data))*np.pi/180))
                y_l =  lidar_data[i]*(np.sin( i*(360/len(lidar_data))*np.pi/180))
                raggioPos = np.array([x_l,y_l,0,1])[np.newaxis]
                [x_dr, y_dr, z_dr, w_dr] = self.rot_RPYbody2NED(roll,pitch,yaw,raggioPos)
                #calcolo posizione ostacolo
                vect_l = np.array([float("{0:.3f}".format(float(pos_NED[0] + x_dr))), float("{0:.3f}".format(float(pos_NED[1] + y_dr))), float("{0:.3f}".format(float(z_dr)))])
                if vect_l[2] > 0:
                    obs = np.vstack([vect_l,obs])
        return obs
    
    def plot_map(self,PF):
        #plt.clf()
        x_values = []
        y_values = []
        #if PF != [None,None,None]:
        if PF.any() != None :
            for coord in PF:
            
                if coord[0] != 0:
                    x_values.append(coord[0])
                    y_values.append(coord[1])

        plt.scatter(x_values, y_values, color='red', s =1)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Obstacle Map')
        plt.xlim(-15, 15)  
        plt.ylim(-15, 15)  
        plt.grid(True)
        plt.show()
        plt.pause(0.001)
            

if __name__ == '__main__':
    node_name ="lidar_check"
    rospy.init_node(node_name)
    sensor_cheking()

    plt.ion()


    rospy.spin()
