#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np
#from map_creation import map_creation

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import threading
import time
import tf
from nav_msgs.msg import OccupancyGrid

class occupancy_grid:
    def __init__(self, sensor_checker):
        print("map creation...")
        self.sensor_checker = sensor_checker
        # Initialize occupancy grid message
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        self.resolution = 0.1
        self.width = 1000
        self.height = 1000
        # Map update rate (defaulted to 5 Hz)
        self.rate = 5.0

        # fill map_msg with the parameters from launchfile
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        map_msg.data = np.zeros(self.width*self.height, dtype=np.int)

        # initialize grid with -1 (unknown)
        self.grid = np.ndarray((self.width, self.height), buffer=np.zeros((self.width, self.height), dtype=np.int),
                dtype=np.int)
        self.grid.fill(int(-1))

        # set map origin [meters]
        map_msg.info.origin.position.x = - self.width // 2 * self.resolution
        map_msg.info.origin.position.y = - self.height // 2 * self.resolution

        # Publishers
        occ_pub = rospy.Publisher("/map", OccupancyGrid, queue_size = 10)
        map_msg.header.stamp = rospy.Time.now()
        loop_rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.set_obstacle()
            # stamp current ros time to the message
            map_msg.header.stamp = rospy.Time.now()

            # build ros map message and publish
            for i in range(self.width*self.height):
                map_msg.data[i] = self.grid.flat[i]
            occ_pub.publish(map_msg)
            #print("loop")
            loop_rate.sleep()

    def bresenham(self,start, end):
        """
        Implementation of Bresenham's line drawing algorithm
        See en.wikipedia.org/wiki/Bresenham's_line_algorithm
        Bresenham's Line Algorithm
        Produces a np.array from start and end (original from roguebasin.com)
        >>> points1 = bresenham((4, 4), (6, 10))
        >>> print(points1)
        np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
        """
        # setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
        is_steep = abs(dy) > abs(dx)  # determine how steep the line is
        if is_steep:  # rotate line
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        # swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        dx = x2 - x1  # recalculate differentials
        dy = y2 - y1  # recalculate differentials
        error = int(dx / 2.0)  # calculate error
        y_step = 1 if y1 < y2 else -1
        # iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = [y, x] if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += y_step
                error += dx
        if swapped:  # reverse the list if the coordinates were swapped
            points.reverse()
        points = np.array(points)
        return points

    def set_obstacle(self):
        # set the occupied cells when detecting an obstacle
        obstacles_lidar = sensor_checker.get_PF()
        pose = sensor_checker.get_pose()

        pose_grid_x = int(pose[1] // self.resolution + self.width  // 2)
        pose_grid_y = int(pose[0] // self.resolution + self.height  // 2)
        #if obstacles_lidar:
        for obs in obstacles_lidar:
            if obs.any() != None :
                
                oy = obs[0] // self.resolution + self.width  // 2
                ox = obs[1] // self.resolution + self.height // 2
                if obs[3] == 1:
                    # set probability of occupancy to 100 and neighbour cells to 50
                    self.grid[int(ox), int(oy)] = int(100)
                    if  self.grid[int(ox+1), int(oy)]   < int(1):
                        self.grid[int(ox+1), int(oy)]   = int(50)
                    if  self.grid[int(ox), 	 int(oy+1)] < int(1):
                        self.grid[int(ox),   int(oy+1)] = int(50)
                    if  self.grid[int(ox-1), int(oy)]   < int(1):
                        self.grid[int(ox-1), int(oy)]   = int(50)
                    if  self.grid[int(ox),   int(oy-1)] < int(1):
                        self.grid[int(ox),   int(oy-1)] = int(50)
                    points = self.bresenham((pose_grid_x ,pose_grid_y),(int(ox),int(oy)))
                    for p in points[:-3]:
                        if self.grid[int(p[0]), int(p[1])]   >= int(30):
                            self.grid[int(p[0]), int(p[1])]   = self.grid[int(p[0]), int(p[1])]-5
                        else:
                            self.grid[int(p[0]), int(p[1])]   = int(0)
                elif obs[3] == 0:
                    points = self.bresenham((pose_grid_x ,pose_grid_y),(int(ox),int(oy)))
                    for p in points:
                        if self.grid[int(p[0]), int(p[1])]   >= int(30):
                            self.grid[int(p[0]), int(p[1])]   = self.grid[int(p[0]), int(p[1])]-5
                        else:
                            self.grid[int(p[0]), int(p[1])]   = int(0)


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
        self.grid_map = []
        self.PF = []
        self.lidar_range = 12
        


    def rot_RPYbody2NED(self,roll,pitch,yaw,vett):
        #matrice di rotazione
        R_x_roll  =  np.matrix([[1,        0,             0 ,       0],\
                            [0,    np.cos(roll),  -np.sin(roll),     0],\
                            [0,    np.sin(roll), np.cos(roll),     0],\
                            [0 ,    0,              0 ,             1]])
        
        R_y_pitch = np.matrix([[np.cos(pitch),  0,   np.sin(pitch),   0],\
                            [       0,          1,          0,         0],\
                            [-np.sin(pitch),     0,    np.cos(pitch),   0],\
                            [       0,          0,          0,         1]])
        
        R_z_yaw   = np.matrix([[np.cos(yaw),  -np.sin(yaw),    0,    0],\
                            [np.sin(yaw),  np.cos(yaw),      0,    0],\
                            [      0,           0,            1,    0],\
                            [      0,           0,            0,    1]])
        
        R_transl  =  np.matrix([[1,        0,             0,          0.3],\
                            [0,            1,             0,          0],\
                            [0,            0,             1,          0.335],\
                            [0 ,           0,             0,          1]])

          
        ans = np.matmul(R_x_roll,R_y_pitch)
        ans = np.matmul(ans,R_z_yaw)
        ans = np.matmul(ans.T,R_transl)
        ans = np.matmul(ans,vett.T)
        return ans


    def position_cb(self,data):
        [self.roll,self.pitch,self.yaw] = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,\
                                                                                    data.pose.pose.orientation.z,data.pose.pose.orientation.w])
        self.yaw = -self.yaw
        self.x = round(data.pose.pose.position.x,8)
        self.y = round(data.pose.pose.position.y,8)
        self.z = round(data.pose.pose.position.z,8)
         #print("posizione: ", round(self.x,2), " ", round(self.y,2), " ", self.yaw, "\n")


    def lidar_cb(self, data):
        pos = [self.x,self.y,self.z]
        roll = self.roll
        pitch = self.pitch
        yaw = self.yaw

        list_points = []
        for i in range(360):                    # Index in this array corresponds to the angle of the laser
            list_points.append( min(data.ranges[i],self.lidar_range))

        obs = self.Obstacles(list_points,roll,pitch,yaw,pos)
        #self.add_to_grid_map(obs)
        self.PF = obs
    

    def get_grid_map(self):
        return self.grid_map
    

    def get_PF(self):
        return self.PF
    
    def get_pose(self):
        return [self.x,self.y,self.yaw]
    
    


    def Obstacles(self,lidar_data,roll,pitch,yaw,pos_NED):
        LiDar = self.lidar_range - 0.2
        self.lidar_data = lidar_data
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.pos_NED = pos_NED
        points = np.array([None,None,None,None])
        
        for i in range(len(lidar_data)-1):
            
                x_l =  lidar_data[i]*(np.cos( i*(360/len(lidar_data))*np.pi/180))
                y_l =  lidar_data[i]*(np.sin( i*(360/len(lidar_data))*np.pi/180))
                raggioPos = np.array([x_l,y_l,0,1])[np.newaxis]
                [x_dr, y_dr, z_dr, w_dr] = self.rot_RPYbody2NED(roll,pitch,yaw,raggioPos)
                if lidar_data[i]<LiDar:
                    obs = 1
                else:
                    obs = 0
                print(z_dr)
                if z_dr > 0.15:
                    
                    vect_l = np.array([float("{0:.3f}".format(float(pos_NED[0] + x_dr))), float("{0:.3f}".format(float(pos_NED[1] + y_dr))), float("{0:.3f}".format(float(z_dr))), obs])
                    if vect_l[2] > 0:
                        points = np.vstack([vect_l,points])
                
        return points 
    
    def add_to_grid_map(self,obstacles_lidar):
        for obs in obstacles_lidar:
             if obs.any() != None :
                if obs[3] == 1:
                    ox = round(obs[0],1)
                    oy = round(obs[1],1)
                    pixel = (ox, oy)
                    if pixel not in self.grid_map:
                        self.grid_map.append(pixel)
                    
 

class MapPlotter:
    def __init__(self, sensor_checker):
        self.sensor_checker = sensor_checker

    def update_and_plot(self):
        while True:
            PF = self.sensor_checker.get_PF()
            grid_map = self.sensor_checker.get_grid_map()
            pose = self.sensor_checker.get_pose()
            self.plot_combined_map(PF, grid_map,pose)
            time.sleep(0.5)

    def start_thread(self):
        # Start the thread for updating and plotting
        update_thread = threading.Thread(target=self.update_and_plot)
        update_thread.daemon = True  # Set the thread as daemon so it will exit when the main program ends
        update_thread.start()
        
    def plot_combined_map(self, PF,grid_map,pose):
        plt.figure(num=1, figsize=(12, 12))
        plt.cla()
        plt.title('Combined Map')
        
        # Plot grid map squares (blue)
        if grid_map:
            for x, y in grid_map:
                square = patches.Rectangle((x, y), 0.1, 0.1, angle=0, color='blue')
                plt.gca().add_patch(square)
            plt.xlim([min(x for x, y in grid_map) - 5, max(x for x, y in grid_map) + 5])
            plt.ylim([min(y for x, y in grid_map) - 5, max(y for x, y in grid_map) + 5])
        else:
            plt.xlim([-7.5, 7.5])
            plt.ylim([-7.5, 7.5])
        
        triangle_points = np.array([[1, 0], [-0.5, -0.5], [-0.5, 0.5], [1, 0]])

        # Rotazione del triangolo
        cos_yaw = np.cos(pose[2])
        sin_yaw = np.sin(pose[2])
        rotation_matrix = np.array([[cos_yaw, -sin_yaw],
                                    [sin_yaw, cos_yaw]])
        rotated_triangle_points = np.dot(triangle_points, rotation_matrix)
        translated_triangle_points = rotated_triangle_points + np.array([pose[0], pose[1]])
        robot = patches.Polygon(translated_triangle_points, closed=True, edgecolor='green')
        plt.gca().add_patch(robot)

        # Plot scatter plot of Lidar Map (red)
        x_values = [coord[0] for coord in PF if coord[0] != 0 and coord[3] != 0]
        y_values = [coord[1] for coord in PF if coord[0] != 0 and coord[3] != 0]
        plt.scatter(x_values, y_values, color='red', s=3)
        
        # Add labels and grids
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid(True)
        plt.gca().set_aspect('equal', adjustable='box')

        # Show the plot
        plt.tight_layout()
        plt.show()
        plt.pause(0.001)

if __name__ == '__main__':
    node_name = "create_map"
    rospy.init_node(node_name)
    sensor_checker = sensor_cheking()
    #map_plotter = MapPlotter(sensor_checker)  # Pass the sensor_checker instance to the MapPlotter
    #map_plotter.start_thread()
    plt.ion()

    grid = occupancy_grid(sensor_checker)

    

    rospy.spin()