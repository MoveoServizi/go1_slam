#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from threading import Thread
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry

import math
import numpy as np
import matplotlib.lines as mlines
import matplotlib.pyplot as plt

from tf_conversions import posemath as pm
from visualization_msgs.msg import Marker
#from scipy.ndimage import gaussian_filter1d
import statistics

maximum = 115
minimum = 350

#global side
#-----------------USEFULL FUNCTIONS----------------------#
def distance_origine(point):                            ## Compare points coordinates
    # Calculer la distance euclidienne entre le point et l'origine (0, 0)
    return (point[0]**2 + point[1]**2) ** 0.5

def points_plus_loin_origin(point1, point2):            ## Sort points from farthest to closer (to create hough line)
    # Calculer les distances de chaque point à l'origine
    distance_point1 = distance_origine(point1)
    distance_point2 = distance_origine(point2)

    # Comparer les distances et renvoyer les points en conséquence
    if distance_point1 > distance_point2:
        return point1, point2
    else:
        return point2, point1

def tri_listes_points(Hough_points, Mean_points):       ## Sort lists of hough and mean points
    P1 = []
    P2 = []

    for i in range(min(len(Hough_points), len(Mean_points))):
        hough_point, mean_point = Hough_points[i], Mean_points[i]
        point_plus_loin, point_plus_proche = points_plus_loin_origin(hough_point, mean_point)

        P1.append(point_plus_loin)
        P2.append(point_plus_proche)

    return P1, P2

def calculate_angle(point1, point2):                    ## Calculate angle of the hough line
    # Calculer la différence entre les coordonnées en y
    delta_y = round(point2[1] - point1[1],3)
    # Calculer la différence entre les coordonnées en x
    delta_x = round(point2[0] - point1[0],3)
    # Calculer l'angle en radians en utilisant la fonction atan2
    angle_rad = round(math.atan2(delta_y, delta_x),3)
    # Convertir l'angle en radians en degrés si nécessaire
    #angle_deg = round(math.degrees(angle_rad),1)
    return angle_rad

def cmd_publisher(x,z):                                 ## Publish on cmd_vel topic
    pub = rospy.Publisher("/cmd_vel" ,Twist, queue_size=10)
    rate = rospy.Rate(10)
    msg= Twist()
    msg.linear.x = x
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.z = z
    #while not rospy.is_shutdown():
    pub.publish(msg)
        #rate.sleep()

#----------------------CLASS------------------------------#
class Quaternion:                                       ## Quaternion
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

class EulerAngles:                                      ## Roll Pitch Yaw
    def __init__(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

class GetObs:                                           ## Get and plot obstacles seen by the Lidar
    def __init__(self, initial_side):
        #subscribe to obstacles topic  (seen by lidar)
        obs_subscriber = rospy.Subscriber("/scan", LaserScan,self.local_map )
        self.P=[]
        self.PF = []
        self.Pfull=[]
        self.wantedP = []
        self.list_points = []
        self.side=initial_side
        
    def round_int(self,x):
        if x in [float("-inf"),float("inf")]: return float("10")
        return float(round(x,5))  

    def local_map(self,data):
        self.list_points = [0]*360                   # Array containing all distance of obs from Lidar                                       
        for i in range(360):                    # Index in this array corresponds to the angle of the laser
            self.list_points[i]=( self.round_int((data.ranges[i])))

        self.P = [[0] * 3 for i in range(360)]  # Obstacle coordinates in Lidar Base
        self.wantedP = [[0] * 3 for i in range(360)]  # Obstacle coordinates in Lidar Base

        wantedP2 = np.zeros((360, 3))  # Obstacle coordinates in Robot base
        wantedP2_new = np.zeros((360, 3))
        P2 = np.zeros((360, 3))  # Obstacle coordinates in Robot base
        P2_new = np.zeros((360, 3))

        T = np.array([[1, 0, 0.3],  # Translation matrix: Lidar to robot base
                      [0, 1, 0],
                      [0, 0, 1]])

        for j in range(len(self.P)):
            if self.list_points[j]!=10:

                angle = math.radians(j)
                self.P[j][0] = round(self.list_points[j] * math.cos(angle), 4) # from polar to cartesian coordinates
                self.P[j][1] = round(self.list_points[j] * math.sin(angle), 4)
                self.P[j][2] = 0  # Homogeneous coordinate
                
                if self.side == 'left':
                    if (j <= 130): #or j > 350) : 
                        self.wantedP[j][0] = round(self.list_points[j] * math.cos(angle), 4) # from polar to cartesian coordinates
                        self.wantedP[j][1] = round(self.list_points[j] * math.sin(angle), 4)
                        self.wantedP[j][2] = 0  # Homogeneous coordinate
                
                elif self.side == 'right':
                    if j > 231:
                    #if (j < 10 or j > 231) : 
                        self.wantedP[j][0] = round(self.list_points[j] * math.cos(angle), 4) # from polar to cartesian coordinates
                        self.wantedP[j][1] = round(self.list_points[j] * math.sin(angle), 4)
                        self.wantedP[j][2] = 0  # Homogeneous coordinate
                
                elif self.side == 'back':
                    if (j < 285 and j > 75) : 
                        self.wantedP[j][0] = round(self.list_points[j] * math.cos(angle), 4) # from polar to cartesian coordinates
                        self.wantedP[j][1] = round(self.list_points[j] * math.sin(angle), 4)
                        self.wantedP[j][2] = 0  # Homogeneous coordinate
                
                elif self.side == 'front':
                    if (j < 30 or j > 330) : 
                        self.wantedP[j][0] = round(self.list_points[j] * math.cos(angle), 4) # from polar to cartesian coordinates
                        self.wantedP[j][1] = round(self.list_points[j] * math.sin(angle), 4)
                        self.wantedP[j][2] = 0  # Homogeneous coordinate
                
                elif self.side == 'left_maze':
                    if (j < 135 and j > 45 ) : 
                        self.wantedP[j][0] = round(self.list_points[j] * math.cos(angle), 4) # from polar to cartesian coordinates
                        self.wantedP[j][1] = round(self.list_points[j] * math.sin(angle), 4)
                        self.wantedP[j][2] = 0  # Homogeneous coordinate
                
                elif self.side == 'right_maze':
                    if (j < 315 and j > 225) : 
                        self.wantedP[j][0] = round(self.list_points[j] * math.cos(angle), 4) # from polar to cartesian coordinates
                        self.wantedP[j][1] = round(self.list_points[j] * math.sin(angle), 4)
                        self.wantedP[j][2] = 0  # Homogeneous coordinate
                
                # elif self.side == 'None':  ## wantedP=PF
                #     self.wantedP[j][0] = round(self.list_points[j] * math.cos(angle), 4) # from polar to cartesian coordinates
                #     self.wantedP[j][1] = round(self.list_points[j] * math.sin(angle), 4)
                #     self.wantedP[j][2] = 0  # Homogeneous coordinate
                
                else :
                    self.wantedP = None

        ## Translate coordinates in robot base : P2 = tous les obstacles vus par Lidar 
        for i in range(len(self.P)):        
                P2[i] = np.matmul(T, np.array(self.P[i]))  # Translate coordinate in robot base
        P2_new = P2[P2[:, 0] != 0]  # Filter out zero entries to see only the current obstacle on the map
        self.Pfull = P2_new[:, :2]

        ## Translate coordinates in robot base : WantedP = obstacles sur le coté observé
        if self.wantedP is not None : 
            for i in range(len(self.wantedP)):        
                    wantedP2[i] = np.matmul(T, np.array(self.wantedP[i]))  # Translate coordinate in robot base        
            wantedP2_new = wantedP2[wantedP2[:, 0] != 0]  # Filter out zero entries to see only the current obstacle on the map
            self.PF = wantedP2_new[:, :2]
            # print('pf from getobs',self.PF)
        #self.plot_map(self.PF)

    def plot_map(self,PF, color):
        #plt.clf()
        x_values = []
        y_values = []
        for coord in PF:
            if coord[0] != 0:
                x_values.append(coord[0])
                y_values.append(coord[1])

        plt.scatter(x_values, y_values, color=color, s =1)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Obstacle Map')
        plt.xlim(-7, 7)  
        plt.ylim(-7, 7)  
        plt.grid(True)
        plt.pause(0.001)   #pause to allow plot to update

    def show_plot(self):
        plt.ioff()
        plt.show()

class Odom:                                             ## Get odometry infos (position/angles)
    def __init__(self):
        #subscribe to odometrys topic
        odom_subscriber = rospy.Subscriber("/odom", Odometry, self.position_cb)
        #self.z_angle = 0
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

    def position_cb(self, data):
        q = Quaternion( data.pose.pose.orientation.w,
                        data.pose.pose.orientation.x,
                        data.pose.pose.orientation.y,
                        data.pose.pose.orientation.z,)

        angles = self.ToEulerAngles(q)

        self.alpha  =round(angles.yaw   , 6)  #rotation around z    #created to make the transform matrix in create_map
        self.beta   =round(angles.pitch , 6)  #rotation around y
        self.gama   =round(angles.roll  , 6)  #rotation around x

        self.rob_pos =np.array([ round(data.pose.pose.position.x,5) , round(data.pose.pose.position.y, 5) , round(data.pose.pose.position.z, 5) ])

class Wall_follower:                                    ## Follow a wall, left or right side
    
    #----------------------------------------------------------------------------------------------------------------------------------------------------------------#
    # The objective of this class is to follow a wall (left, or right). This class allows the robot to : chose a wall to follow. Manage the cases in which obstacles
    # are too close or too far from the robot, updating the robot's trajectory. 
    ## to do : 
    ##  -if obstacle in front but space between wall followed and front obs -> pass between
    ##  -if no more obstacle seen :  back in itial "go straight" state
    def __init__(self):

        self.new_side=''
        self.previous_side=''
        self.new_mode=''

    def cmd_angle(self,reach_alpha):                    ## Command in order to follow the direction set by reach_alpha, in the robot base. See schema

            if (reach_alpha >1.57 and reach_alpha < 3.09) or (reach_alpha < -0.05 and reach_alpha > -1.57):
                    z_angle_cmd = 1.3# + additional #rad/s
                    x_cmd = 0.8
            elif (reach_alpha <=1.57 and reach_alpha > 0.05) or (reach_alpha <= -1.57 and reach_alpha > -3.09):
                    z_angle_cmd = -1.3 #+ additional
                    x_cmd = 0.8

            else :
                    z_angle_cmd =  0
                    x_cmd = 2

            return z_angle_cmd, x_cmd

    def indices_valeurs_inf_1(self,tableau):            ## Returns indices in array (ex : lidar_points) of obs closer than 1 meter (useless for the moment)
        indices_inferieurs_a_1 = [i for i, valeur in enumerate(tableau) if valeur < 1]
        return indices_inferieurs_a_1

    def indices_valeurs_sup_4(self,tableau):            ## Returns indices in array (ex : lidar_points) of obs farther than 1 meter (useless for the moment)
        indices_superieurs_a_4 = [i for i, valeur in enumerate(tableau) if valeur > 4 and valeur <10]
        return indices_superieurs_a_4
    
    def indices_5last_sup_a_4(self,tableau):            ## Returns indices in array (ex : lidar_points) of the 5 farthest obstacles (useless for the moment)
        # Créer une liste de tuples (indice, valeur) pour garder la correspondance avant le tri
        indices_valeurs = list(enumerate(tableau))

        # Tri du tableau en utilisant la valeur pour comparer les éléments
        tableau_trie = sorted(indices_valeurs, key=lambda x: x[1])

        # Vérification des cinq premières valeurs triées pour s'assurer qu'elles sont supérieures à 4
        for i in range(5):
            if tableau_trie[i][1] <= 4:
                return None

        # Récupération des indices des cinq premières valeurs triées dans le tableau initial
        indices_cinq_plus_petites = [t[0] for t in tableau_trie[:5]]
        return indices_cinq_plus_petites

    def choose_side(self,previous_side, lidar_points):  ## Update the current side or mode we want to follow
            #print('Check side')
            self.previous_side=previous_side
            mode=0
            section = { #'front': min(lidar_points[0:30] + lidar_points[331:360]),  ## Lidar sections importants for navigation
                        'left'          : min(lidar_points[0  :110]),               ## side which will give PF in getobs class
                        'left_to_back'  : min(lidar_points[115:130]),               ## watch to know when go from left observation to back observation
                        'left_to_right' : min(lidar_points[351:356]),               ## same thing for left to right
                        'left_no_more1'  : min(lidar_points[ 90:100]),              ## watch to confirm that no more obs on left and then go to back obs
                        
                        'back'  : min(lidar_points[131:230]),
                        'back_to_left' : min(lidar_points[72:77]),
                        'back_to_right' : min(lidar_points[270:275]),

                        'right'         : min(lidar_points[249:359]),
                        'right_to_left' : min(lidar_points[6  :11 ]),
                        'right_to_back' : min(lidar_points[230:245]),
                        'right_no_more1': min(lidar_points[260:270])
                        }
            
            #--------- MAZE MODE : the main will use the "mode" value to change the range of points seen to be more precise on a side-----------#
            #--------- used in case of close obstacles around the robot, allow to approach closer to obs (see use in manage_close)   -----------#
            
            ## Enter in maze mode
            if section['left']<1.5 and section['right']<1.5:    
                if previous_side=='left':
                    self.new_side='left_maze'
                    mode=1
                elif previous_side=='right':
                    self.new_side='right_maze'
                    mode=1
            
            ## exit from maze mode (to see in main)
            elif section['left']>2 and section['right']>=2:     
                mode=0
                if previous_side=='left_maze':
                    self.new_side='left'
                    mode=2
                elif previous_side=='right_maze':
                    self.new_side='right'
                    mode=2
            
            #--------- NORMAL MODE-----------#  

            if previous_side == 'None' :                                            ## from NONE to detect one side
                mode=0
                if section['left']<10 and section['right']<10:                      ## both left and right sides on vision 
                    if section['left']<section['right']:
                        self.new_side='left'
                    elif section['right']<section['left']:                          ## chose the closest       
                        self.new_side='right'
                
                elif section['left']<10:                                            ## if only left : follow left
                    self.new_side='left'
                    
                elif section['right']<10:                                           ## if only right : follow right
                    self.new_side='right'
                
                # elif section['back']<10:                                          ## if only back : follow back   /!\ TO ADD /!\
                #     self.new_side='back'
            
            ## from left to other sides 
            elif previous_side == 'left' :                                          
                if section['left_to_back']<10 and section['left_no_more1']>=10 :    ## check if still see the wall on back_left side, and if still no more wall on front_left
                    self.new_side='back'
            
            ## from right to other sides
            elif previous_side == 'right' :                                         
                if section['right_to_back']<10 and section['right_no_more1']>=10 :  ## check if still see the wall on back_right side, and if still no more wall on front_right
                    self.new_side='back'
            
            ## from back to other sides 
            elif previous_side=='back':
                if section['back_to_left']<10:
                    self.new_side='left'
                elif section['back_to_right']<10:
                    self.new_side='right'

            ## nothin on lidar : NONE
            elif lidar_points == []:
                self.new_side='None'
                mode=0

            return self.new_side, mode

    def manage_close(self,mode, lidar_points) :         ## Detect when obstacles are too close of robot, return the danger side (in main we will apply a react command)
        if mode == 1 :                                                               ## the mode changes the minimum distance for closest obstacles
            lim = 0.4
        else: 
            lim = 1.5
        if not lidar_points:
            # Handle the case when lidar_points is empty (e.g., raise an exception, return default values, etc.)
            print("LIDAR scan points list is empty")
            x_cmd=0
            angle_cmd=0
            danger='No'
        else:
            section = { 'front' : min(lidar_points[0:30] + lidar_points[331:359]),
                        'left'  : min(lidar_points[31:105]),
                        'back'  : min(lidar_points[131:230]),
                        'right' : min(lidar_points[255:330]),
                        'left_close': min(lidar_points[70:105]),
                        'right_close':min(lidar_points[255:290])}

            angle_cmd=0
            x_cmd=0

            ## Manage for front side                   /!\ TO CHANGE : add a way to recognize when space between front obs and side followed (like door)           
            if section['front'] < lim :
                danger='front'
                if self.new_side=='left' or self.previous_side=='left' or self.new_side=='left_maze' or self.previous_side=='left_maze':
                    if (section['left_close']>= section['right_close']) :
                        x_cmd=0
                        angle_cmd= -1.5
                    elif (section['left_close']< section['right_close']) :
                        x_cmd=0
                        angle_cmd= 1.5
                elif self.new_side=='right' or self.previous_side=='right' or self.new_side=='right_maze'or self.previous_side=='right_maze':
                    x_cmd=0
                    angle_cmd= -1.5

            #### manage for left and right sides       /!\ maybe check that sections are well defined and cause no problems when a side is close to obs
            elif section['left_close']< lim and section['right_close']< lim :
                danger='left & right'
                if section['left_close']<section['right_close']:
                    x_cmd=0.3
                    angle_cmd=-1.5
                elif section['left_close']>section['right_close']:
                    x_cmd=0.3
                    angle_cmd= 1.5
            elif section['left_close']< lim  :
                danger='left'
                x_cmd=0.3
                angle_cmd=1.5 

            elif section['right_close']< lim :
                danger='right'
                x_cmd=0.3
                angle_cmd=-1.5
            else:
                danger='No'

        return danger,angle_cmd, x_cmd

    def manage_far(self, mean_point):                   ## Create the react command to reach too far obstacles 
        reach_aplha= calculate_angle([0,0],mean_point)              # Angle to follow to reach the wall
        angle_far, x_far = self.cmd_angle(reach_aplha)
        
        if mean_point[0]>0 and mean_point[1]>=0:  # haut droit
            angle_far=angle_far
            x_far=x_far

        elif mean_point[0]<0 and mean_point[1]>=0: # haut gauche
            x_far = -x_far
            angle_far= -angle_far

        elif mean_point[0] < 0 and mean_point[1] <0: # bas gauche      
            angle_far = -angle_far
            x_far =-x_far
        
        elif mean_point[0]>0 and mean_point[1]<0:   #bas droit
            x_far= x_far
            angle_far=angle_far

        return angle_far, x_far

    def far_from_mean(self,mean_point):                 ## Detect if the obstacle mean point created by Hough on the watched side is farthest than 4 meters (then obs is too far)                 
        robot_point=[0, 0]
        x=mean_point[0]
        y=mean_point[1]
        angle_cmd=0
        too_far=0
        dist=(x**2 + y**2)**0.5
        if dist >= 4:
            angle_cmd=calculate_angle(robot_point, mean_point)
            too_far=1
        return too_far, angle_cmd

class HoughTransform:                                   ## Trace line trajectory from obstacle points (usefull for wall_follower)

    #----------------------------------------------------------------------------------------------------------------------------------------------------------------#
    # The objective is to trace a trajectory unsing a Hough Transform on points seen localy by the Lidar, in order to follow a wall.For each point, we want to test  #
    # 18 (for 180 degree) fake polar points (Hough Points) that will allow to imagine line between these fake points and every point that sees the Lidar. These fake # 
    # polar points will lead to arrays of datas : each point will have a value of radius for each theta. We will be able to find some similar polar points for some  # 
    # values of theta and for some point : it means that these points are almost on the same line.Then, we will create a mean_point between all these initial points #
    # and create a line between this mean_point and the Hough Point.                                                                                                 #
    #----------------------------------------------------------------------------------------------------------------------------------------------------------------#
    
    def hough_point(self, x, y):                        ## Calculate radius array for each theta of a fake polar point linked to a Lidar Point
        tab_t = [0, 10, 20, 30, 40, 50,60,70,80,90,100,110,120,130,140,150,160,170] # angles we want to check for all points
        rays = [0] * len(tab_t)                                                      # array that will have the radius values for each angles
        h = math.sqrt(x*x + y*y)

        if y>=0:
            alpha = math.acos(x / h)
            for i in range(len(tab_t)):
                beta = alpha - math.radians(tab_t[i])
                rays[i] = round(h * math.cos(beta),3)
        else:
            alpha = -(math.acos(x / h))
            for i in range(len(tab_t)):
                beta = alpha + math.radians(tab_t[i])
                rays[i] = round(h * math.cos(beta),3)
        
        return rays  # contains radius values for each theta value (polar points created) for 1 inital point
    
    def hough_data_array(self, x_array, y_array):       ## Create the recap of radius and theta for each Lidar Point
        DATA = []
        for i in range(len(x_array)):
            rays = self.hough_point(x_array[i], y_array[i])
            DATA.append({'rays': rays})
        return DATA

    def matrice_rays(self, x_array, y_array):           ## Create a matrix from the recap
        data = self.hough_data_array(x_array, y_array)
        num_points = len(data)
        # if len(data[0]) is not None:
        max_rays = len(data[0]['rays'])  
        
        matrix = np.zeros((num_points, max_rays))
        for i in range(num_points):
            matrix[i] = data[i]['rays']

        return matrix         #### Contains values of radius for each points(rows) and each theta(columns)

    def transpose(self,M):                              ## Transpose this Matrix to study similar radius values for each point for each theta
        A = np.transpose(M)
        # print(' ')
        # print ('MARICE A : radius values for theta(rows) and each points(columns) ')
        # print(A)
        return A

    def valeurs_similaires(self, tableau):              ## Return the longest set of similar values in a tab, and their indices in this tab
        valeurs_similaires = []
        longest_similaires = []  # Initialize the variable for the longest list of similar values
        longest_indices = []  # Initialize the variable for the indices of the longest list

        for i in range(len(tableau)):
            valeur_reference = tableau[i]
            similaires = [valeur_reference]
            indices = [i]

            for j in range(len(tableau)):
                if i != j:
                    valeur_courante = tableau[j]
                    similarity = valeur_reference / valeur_courante

                    if similarity >= 0.95 and similarity <= 1.05:
                        similaires.append(valeur_courante)
                        indices.append(j)

            if len(similaires) > 1:
                valeurs_similaires.append((similaires, indices))

                # Update longest_similaires and longest_indices if the current list is longer
                if len(similaires) > len(longest_similaires):
                    longest_similaires = similaires.copy()
                    longest_indices = indices.copy()

        if longest_similaires:
            merged_results = [longest_similaires, longest_indices]
            # print(merged_results[0], "-----> indices:", merged_results[1])
            return merged_results
        else:
            # print(' ---- ')
            return []

    def find_best_rows(self, x_array, y_array, num_rows=1): ## Find the best lines on these parameters : bigger number of values, different points, minimal gap between values
        M = self.matrice_rays(x_array, y_array)
        A = self.transpose(M)

        best_rows = []
        best_gaps = []
        best_similar_values = []
        best_similar_indices = []

        size_max = 0
        for i in range(len(A)):
            similar_values = self.valeurs_similaires(A[i])

            if similar_values and len(similar_values[0]) > size_max:
                size_max = len(similar_values[0])

        for _ in range(num_rows):
            best_row_index = None
            best_gap = np.inf
            best_similar_values_single = None
            best_similar_indices_single = None

            for i in range(len(A)):
                similar_values = self.valeurs_similaires(A[i])

                if similar_values and len(similar_values[0]) == size_max:
                    current_gap = np.max(similar_values[0]) - np.min(similar_values[0][0])

                    if current_gap < best_gap:
                        best_gap = round(current_gap, 3)
                        best_row_index = i
                        best_similar_values_single = similar_values[0]
                        best_similar_indices_single = similar_values[1]

            if best_row_index is not None:
                best_rows.append(best_row_index)
                best_gaps.append(best_gap)
                best_similar_values.append(best_similar_values_single)
                best_similar_indices.append(best_similar_indices_single)

                A = np.delete(A, best_row_index, axis=0)

        results = []
        for i in range(len(best_rows)):
            result = {
                'best_row_index': best_rows[i],
                'best_gap': best_gaps[i],
                'best_similar_values': best_similar_values[i],
                'best_similar_indices': best_similar_indices[i]
            }
            results.append(result)

        # print(results)
        return results
    
    def majorite_negatif(self,tableau):                 ## Returns 1 if the majority of points have negative y coordinate (usefull for theta in best_hough_point)
        total_negatifs = sum(1 for element in tableau if element < 0)
        majorite = len(tableau) // 2 + 1
        if total_negatifs >= majorite:
            return 1
        else:
            return 0

    def best_hough_point(self, x_array, y_array):       ## For every row selected, return hough point and mean point
        results = self.find_best_rows(x_array, y_array)
        hough_points = []
        mean_points = []
        
        for result in results:
            best_row_index = result['best_row_index']
            best_similar_values = result['best_similar_values']
            best_similar_indices = result['best_similar_indices']
            
            # Calculate the Mean point
            x_points = []
            y_points = []
            for i in range(len(best_similar_indices)):
                x_points.append(x_array[best_similar_indices[i]])
                y_points.append(y_array[best_similar_indices[i]])

            x_mean=round(np.mean(x_points),3)
            y_mean=round(np.mean(y_points),3)
            mean_point=(x_mean,y_mean)

            # Calculate the Hough point
            radius=round(np.mean(best_similar_values),3)
            
            if self.majorite_negatif(y_points)==1:
                theta=-(best_row_index*10)
            else:
                theta=best_row_index*10

            x_hough = round(radius * math.cos(math.radians(theta)),3)
            y_hough = round(radius * math.sin(math.radians(theta)),3)
            hough_point = (x_hough, y_hough)

            # ajouter mean et hough
            hough_points.append(hough_point)
            mean_points.append(mean_point)
        # print ('Hough Points : ', hough_points)
        # print ('Mean Points  : ', mean_points)

        return hough_points, mean_points
 
    ####------     PLOT    -----####

    def plot_hough_points(self,hough_points, ax):       ## Plot hough points
        colors = ['green','red', 'orange']  # Define colors for each point
        
        # Plot each Hough point with a different color
        for i in range(len(hough_points)):
            ax.scatter(hough_points[i][0], hough_points[i][1], color=colors[i], label=f'Point {i+1}')
        
        ax.set_xlabel('X Hough')
        ax.set_ylabel('Y Hough')
        ax.set_title('Hough Points')
        ax.set_xlim(-5, 10)
        ax.set_ylim(-5, 10)

        ax.legend()
        
    def plot_mean_points(self,mean_points, ax):         ## Plot mean points 
        colors = ['red', 'orange']  # Define colors for each point
        
        # Plot each Hough point with a different color
        for i in range(len(mean_points)):
            ax.scatter(mean_points[i][0], mean_points[i][1], color=colors[i], label=f'Point {i+1}')
        
        ax.set_xlabel('X Mean')
        ax.set_ylabel('Y Mean')
        ax.set_title('Mean Points')
        ax.set_xlim(-5, 10)
        ax.set_ylim(-5, 10)

        ax.legend()

    def plot_init_points(self, x_array, y_array, ax):   ## Plot initial points (seen by lidar)
        if ax is None:
            ax = plt.gca()

        # Plot points
        ax.scatter(x_array, y_array, color='black', label='Points', s=10)

        # Set plot labels and legend
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.legend()
   
    def plot_line_between_points(self, array_hough, array_mean, ax):    ## Plot the hough line
        colors = ['red', 'orange']  # Define colors for lines

        # Plot lines between corresponding Hough and Mean points
        for i in range(len(array_hough)):
            dx = array_mean[i][0] - array_hough[i][0]
            dy = array_mean[i][1] - array_hough[i][1]

            slope = dy / (dx+0.001) 
            line_length = 10
            x_start = array_hough[i][0] - line_length * dx
            x_end = array_mean[i][0] + line_length * dx
            y_start = array_hough[i][1] - line_length * slope * dx
            y_end = array_mean[i][1] + line_length * slope * dx
            # else:
            #     slope = float('inf')
            #     line_length = 10
            #     x_start = array_hough[i][0] - line_length
            #     x_end = array_mean[i][0] + line_length
            #     y_start = array_hough[i][1]
            #     y_end = array_mean[i][1]
            #     print('ERROR LINE')

            ax.plot([x_start, x_end], [y_start, y_end], color=colors[i], label='Line')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Line Between Hough and Mean Points')

#-----------------------MAIN------------------------------#
def main():
    rospy.init_node('obstacle_detection')       # Initialize ROS node
    getobs = GetObs(initial_side='None')        # Create instances 
    hough = HoughTransform()
    odom = Odom()
    wall = Wall_follower()
    #side_before=''

    def plot_data():
        fig, ax = plt.subplots()
        while not rospy.is_shutdown():
            ax.clear()
           
            getobs.plot_map(getobs.Pfull,'black')                                                                       # Plot in black all the points seen by the Lidar
            new_side, new_mode=wall.choose_side(getobs.side,getobs.list_points)
            danger, react_angle, react_x = wall.manage_close(new_mode,getobs.list_points)                               # Call the distance function (used when an obstacle is found)                                                                        
            print('-------------------------------')
            print(' new side : ',new_side)
            print(' new mode : ',new_mode)
            print(' Danger   : ',danger)

            ######  No side followed
            if getobs.side=='None':
                print('None side : ', getobs.side)
                print('No wall detected')
                angle_cmd=0
                x_cmd= 1.3
                print('Searching.... : x =', x_cmd, ', angle =' , angle_cmd)
                
                ######  check if obstacle seen --> change side
                if len(getobs.Pfull)!=0:
                    previous_side=getobs.side              
                    getobs.side=new_side                    # <--------------------                                     # Change side on GetObs to watch only the good section of Pfull
                    print('New side found : ',getobs.side)
            
            
            ######  A side is followed
            elif getobs.side != 'None' and getobs.side != 'back':                                                
                
                ###### Check if no obs seen --> back to None
                if len(getobs.Pfull)==0:
                    previous_side=getobs.side               
                    getobs.side='None'                      # <--------------------
                
                ###### Obstacle seen on the side followed
                else : 
                    getobs.plot_map(getobs.PF,'Blue')                                                                   # Plot in blue obstacle followed
                    print(' nb of observed points : ', len(getobs.PF))
                    
                    ###### Check if still something on side watched
                    if len(getobs.PF) != 0 :
                        ###### If almost no more obs on side ---> change side (usefull for GO BACK)                                                      
                        if len(getobs.PF) < 5:                                                                          # If almost no more obs on the side watched
                                                                                                
                            previous_side=getobs.side                                                                   # Change the side watched
                            getobs.side=new_side            # <--------------------                                     # Change side on GetObs to watch only the good section of Pfull
                            print('SIDE CHANGED : ', getobs.side)
                        
                        ################################################
                        ##### if asked by 'mode', change side in MAZE MODE
                        if new_mode==1:
                            if new_side=='left_maze':
                                previous_side=getobs.side
                                getobs.side=new_side        # <--------------------
                                print('current_side : ', getobs.side)
                                print('( MAZE MODE LEFT )')
                            elif new_side =='right_maze':
                                previous_side=getobs.side
                                getobs.side=new_side
                                print('current_side : ', getobs.side)
                                print('( MAZE MODE RIGHT )')
                        #################################################
                            else:
                                getobs.side=new_side
                                new_mode=0
                                print('EXIT MAZE MODE')
                        ##### exit maze mode
                        elif new_mode==2:
                            previous_side=getobs.side
                            getobs.side=new_side
                            print('current_side : ', getobs.side)
                            new_mode=0


                        ###### Check if obs too close
                        if danger != 'No' :#and danger !='maze_mode':                                                       # If obstacle seen < 1m in the side watched
                            print('/!\ Danger on',danger,' /!\ ')
                            angle_cmd = react_angle
                            x_cmd     = react_x
                            print('react : x =', x_cmd, ', angle = ' , angle_cmd)
                        
                        ###### No danger, mode choosen, side choosen --> follow a wall 
                        else:
                            ###### If still something on side watched 
                            if getobs.PF !=[]:
                                print('current_side : ', getobs.side)                                                                              # We can follow a wall
                                hough_points, mean_points = hough.best_hough_point(getobs.PF[:, 0], getobs.PF[:, 1])        # Create hough and mean points on side watched
                                if hough_points is not None and mean_points is not None:
                                    too_far, far_cmd = wall.far_from_mean(mean_points[0])                                                      
                                    
                                    ###### Check if side watched is too far --> get closer
                                    if too_far==1:                                                                          # Dist wall watched-robot > 4m
                                        robot=[[0,0]]                                                     
                                        hough.plot_line_between_points(robot,mean_points, ax)                               # Line between robot and mean of side watched
                                        angle_cmd, x_cmd = wall.manage_far(mean_points[0])
                                        print('Try get closer to ',new_side,' wall : x =', x_cmd, ', angle = ' , angle_cmd)
                                    
                                    ###### if no other constraints --> follow with Hough Line
                                    else:
                                        if getobs.side!='back':
                                            if mean_points[0][1]<0:
                                                getobs.side='right'
                                                new_side='right'
                                            elif mean_points[0][1]>=0:
                                                getobs.side='left'
                                                new_side='left'                                                                 # Normal Hough follow
                                        hough.plot_hough_points(hough_points,ax)                                            # Hough point in green
                                        hough.plot_mean_points(mean_points,ax)                                              # Mean point in red
                                        P1,P2 = tri_listes_points(hough_points,mean_points)                                 # Filter in order to have a regular angle (plot_line takes the smallest first)
                                        hough.plot_line_between_points(P1,P2,ax)
                                        #acutal_alpha = odom.alpha                                                          # Robot orientation (useless for the moment)
                                        reach_alpha = round(calculate_angle(P2[0],P1[0]),2)                                          # Orientation given to follow the wall
                                        angle_HOUGH, x_HOUGH = wall.cmd_angle(reach_alpha)                                  # Command to apply to be parallel to this wall
                                        angle_cmd = angle_HOUGH 
                                        x_cmd = x_HOUGH
                                        print('HOUGH : x =', x_cmd, ', angle = ' , angle_cmd)
                            
                            ###### If side watched empty :
                            else:
                                print('No more obstacle, back to research...')
                                previous_side=getobs.side
                                getobs.side='None'
            
            ###### Back Side
            #elif getobs.side=='back':
            elif new_side=='back' or getobs.side=='back':
                if new_side== 'left':
                    getobs.side='left'
                    print('found my wall')
                elif new_side=='right':
                    getobs.side='right'
                    print('Found my wall')
                elif previous_side == 'left':
                    print(' GO BACK LEFT')
                    angle_cmd = -2.3
                    x_cmd = 2.5
                elif previous_side == 'right':
                    print(' GO BACK RIGHT')
                    angle_cmd = 2.3
                    x_cmd = 2.5
                # else :
                #     getobs.side=previous_side

            cmd_publisher(x_cmd,angle_cmd)
            #print("---------------------------------------")
            ax.set_xlabel('X')  # Set plot labels and limits
            ax.set_ylabel('Y')
            ax.set_xlim(-7, 7)
            ax.set_ylim(-7, 7)
            ax.grid(True)
            plt.pause(0.001)    # Update the plot
    plot_data()                 # Call the plot_data function
    rospy.sleep(0.01)
    rospy.spin()                # Spin ROS

###################### HAND TEST  ###################
#### the main i used for testing hough transform (change the parameter on find_best_row to keep more than one line)
#### works with hand test but there is an issue when using on gazebo simulation
# def main():
    

#     #SET1
#     x_array = [4.5 , 5 , 5.5, 6, 7  , 8 ,9    ]
#     y_array = [9   , 8 , 7  , 6, 5.5, 5 , 4.5 ]
    
#     #SET2
#     x_array = [4.25 , 4.5 , 4.75, 5, 6 ,7 , 8   ]
#     y_array = [ 8 , 7  , 6 , 5, 4.75, 4.5 , 4.25 ]

#     #SET3
#     x_array = [4.25 , 4.5 , 4.75, 5, 6 ,7 , 8 , 9  ]
#     y_array = [ 8 , 7  , 6 , 5, 4.75, 4.5 , 4.25, 4 ]

#     #SET4
#     x_array = [4, 4.25 , 4.5 , 4.75, 5, 6 ,7 , 8 , 9  ]
#     y_array = [9, 8 , 7  , 6 , 5, 4.75, 4.5 , 4.25, 4 ]

#     x_array = [4, 4.25 , 4.5 , 4.75, 5, 6 ,7 , 8 , 9  ]
#     y_array = [9, 8 , 7  , 6 , 5, 4.75, 4.5 , 4.25, 4 ]
#     #SET5
#     # x_array = [ 5, 6 ,7 , 8 , 9  ]
#     # y_array = [ 5, 4.75, 4.5 , 4.25, 4 ]

#     #SET6
#     # x_array = [5, 5 , 5 , 5, 5, 6 ,7 , 8 , 9  ]
#     # y_array = [9, 8 , 7  , 6 , 5, 5, 5 , 5, 5 ]

#     # # #SET6
#     # x_array = [5, 5 , 5 , 5, 5]
#     # y_array = [9, 8 , 7  , 6 , 5 ]
#     ######################################
#     H = HoughTransform()
#     fig, ax = plt.subplots()

#     hough_points, mean_points=H.best_hough_point(x_array,y_array)

#     print('--------- main results ----------')
#     print('HOUGH_POINTS : ' , hough_points)
#     print('MEAN_POINTS : ' , mean_points)
#     print('---------------------------------')

#     H.plot_init_points(x_array,y_array,ax)
#     H.plot_hough_points(hough_points,ax)
#     H.plot_mean_points(mean_points,ax)

#     H.plot_line_between_points(hough_points,mean_points,ax)

#     plt.grid(color = 'black', linewidth = 0.2)
    
#     ################   TEST   ##############

#     # test = [ 9.558,  8.616,  7.675,  6.734 , 5.792,  5.72,   5.647 , 5.575 , 5.502]
#     # print('in test : ')
#     # print(H.valeurs_similaires(test))
#     #######################
#     plt.show()       ######
#     #######################
if __name__ == '__main__':

    main()


###################################################################################

 