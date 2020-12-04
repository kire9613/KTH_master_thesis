"""
Adapted from Atsushi Sakai's PythonRobotics pure pursuit example
"""
import math, rospy
import numpy as np
from pprint import pprint
from sensor_msgs.msg import LaserScan
from svea.path_planners.mpc_map.ros_interface import ROSInterface as MapROSInterface
class PurePursuitController(object):

    k = 0.6  # look forward gain
    Lfc = 0.2#0.4  # look-ahead distance
    K_p = 1 #speed control propotional gain
    K_i = 2  #speed control integral gain
    K_d = 0.0  #speed control derivitive gain
    P = 0 # initilize P value (PID)
    I = 0 # intialize I value (PID)
    L = 0.324  # [m] wheel base of vehicle
    max_velocity = 1
    emergency_distance = 0.5#1 # [m] minimum distance until emergency_stop activated
    mapping_distance = 3 # distance of obstacles to map
    width = 0.2485
    height = 0.586
    mapping_angle = 1.57#0.785

    def __init__(self, vehicle_name=''):
        self.traj_x = []
        self.traj_y = []
        self.target = None
        # initialize with 0 velocity
        self.target_velocity = 0.0
        self.last_index = 0
        self.is_finished = False
        self.emg_stop = False
        self.emg_traj_running = False
        self.last_time = 0.0
        self.print_time = 0.0
        self.lidar_to_base = 0.3  #svea position is measured at rear axis, but lasar at front axis 
        self.backed_up = False
        self.emg_angle_range = 0
        self.compute_angle()
        self.laser_scan = None
        self.steering = 0
        self.velocity = 0
        rospy.set_param('/team_5_floor2/lidar_obstacles',[])

    def compute_angle(self):
        self.emg_angle_range = np.arctan2((self.width + 0.1),(2*self.emergency_distance))
        print("emergency angle in rad", self.emg_angle_range)

    def set_emg_traj_running(self,running):
        self.emg_traj_running = running 

    def compute_control(self, state, target=None):
        if self.backed_up:
            return self.steering,-0.6
        elif self.emg_stop and not self.emg_traj_running:
            return 0,0
        
        else:
            self.steering = self.compute_steering(state, target)
            self.velocity = self.compute_velocity(state)
            return self.steering, self.velocity

    def compute_steering(self, state, target=None):
        if target is None:
            dist = self.find_target(state)
        else:
            # allow manual setting of target
            self.target = target

        tx, ty = self.target
        alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
        if state.v < 0:  # back
            alpha = math.pi - alpha
        Lf = self.k * state.v + self.Lfc
        delta = math.atan2(2.0 * self.L * math.sin(alpha) / Lf, 1.0)
        return delta

    def compute_velocity(self, state):
        if self.is_finished:
            # stop moving if trajectory done and reset controller.
            self.I = 0.0
            self.P = 0.0
            return 0.0
        else:
            # speed control
            e = self.target_velocity - state.v
            if state.v < 0.1: # reset integral gain if current velocity is low or if we are close to target velocity
                self.I = 0 
            dt =  state.time_stamp.to_sec() - self.last_time
            self.last_time = state.time_stamp.to_sec()
            self.P = self.K_p*e
            self.I = self.I + self.K_i*e*dt
            velocity_output = self.P + self.I
            if velocity_output > self.max_velocity:
                velocity_output = self.max_velocity
        return  velocity_output


    def find_target(self, state):
        ind, dist = self._calc_target_index(state)
        tx = self.traj_x[ind]
        ty = self.traj_y[ind]
        self.target = (tx, ty)
        return dist

    def _calc_target_index(self, state):
        # search nearest point index
        dx = [state.x - icx for icx in self.traj_x]
        dy = [state.y - icy for icy in self.traj_y]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        dist = 0.0
        Lf = self.k * state.v + self.Lfc

        # search look ahead target point index
        while Lf > dist and (ind + 1) < len(self.traj_x):
            dx = self.traj_x[ind + 1] - self.traj_x[ind]
            dy = self.traj_y[ind + 1] - self.traj_y[ind]
            dist += math.sqrt(dx ** 2 + dy ** 2)
            ind += 1

        # terminating condition
        if dist < 0.1 and not self.backed_up:
            self.is_finished = True

        return ind, dist

    def emergency_stop(self, laserScan):
        
        # Compute index ranges for emergency stop scan
        min_index =  int(round((-self.emg_angle_range - laserScan.angle_min)/laserScan.angle_increment))
        max_index =  int(round((self.emg_angle_range - laserScan.angle_min)/laserScan.angle_increment))

        points = laserScan.ranges[min_index:max_index]
        self.laser_scan = laserScan
        if min(points) < self.emergency_distance:
            self.emg_stop = True
    def laser_mapping(self,state):
        
        laserScan = self.laser_scan

        min_index =  int(round((-self.mapping_angle - laserScan.angle_min)/laserScan.angle_increment))
        max_index =  int(round((self.mapping_angle - laserScan.angle_min)/laserScan.angle_increment))

        # Find indices of laserscans that make up our obstacle
        #indices = np.where(np.array(laserScan.ranges) < self.mapping_distance)
        if min == max:
            indices = np.where(np.array(laserScan.ranges[min_index]) < self.mapping_distance)
        else: 
            indices = np.where(np.array(laserScan.ranges[min_index:max_index]) < self.mapping_distance)
        
        #Get svea's pose
        svea_x = state[0]
        svea_y = state[1]
        yaw = state[2]

        #Calculate lidar pose in map frame             
        length = self.lidar_to_base 
        lidar_coord = [svea_x+length*math.cos(yaw), svea_y+length*math.sin(yaw)]

        #Mapping dynamic obstacles
        idx_list = []
        iobstacles_list = []
        for idx in indices[0]:  
            idx = idx + min_index
            idx_list.append(idx)
            #calculate angle to laser point
            angle = laserScan.angle_min + idx*laserScan.angle_increment
            #coordinate of the laser point/obstacle
            lidar_range = laserScan.ranges[idx]
            obs_coord = [lidar_coord[0]+lidar_range*math.cos(yaw+angle), lidar_coord[1]+lidar_range*math.sin(yaw+angle)]

            # inflate points
            dy = self.width/2
            dx = self.height/2
            
            inflated_obstacle =  [[dx,dy ],
                                 [ + dx, - dy ], 
                                 [ - dx, - dy ],
                                 [ - dx, + dy ]]   
            
            rot = [[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw), math.cos(yaw)]]                  
            obs = np.matmul(rot,np.transpose(inflated_obstacle))
            inflated_obstacle = [[float(obs[0][0]),float(obs[1][0])],[float(obs[0][1]),float(obs[1][1])],[float(obs[0][2]),float(obs[1][2])],[float(obs[0][3]),float(obs[1][3])]]                 
            idx = 0
            for coord in inflated_obstacle:
                inflated_obstacle[idx][0] = coord[0] + obs_coord[0]
                inflated_obstacle[idx][1] = coord[1] + obs_coord[1]
                idx = idx + 1
            # Update list of obstacles with the new obstacle 
      
            iobstacles_list.append(inflated_obstacle)
        
        try:
            rospy.delete_param('/team_5_floor2/lidar_obstacles')
        except KeyError:
            print("value not set")
        rospy.set_param('/team_5_floor2/lidar_obstacles',iobstacles_list)
        self.publish_obstacles(iobstacles_list)

    def laser_mapping_old(self,state):
        
        laserScan = self.laser_scan

        min_index =  int(round((-self.mapping_angle - laserScan.angle_min)/laserScan.angle_increment))
        max_index =  int(round((self.mapping_angle - laserScan.angle_min)/laserScan.angle_increment))

        # Find indices of laserscans that make up our obstacle
        #indices = np.where(np.array(laserScan.ranges) < self.mapping_distance)
        if min == max:
            indices = np.where(np.array(laserScan.ranges[min_index]) < self.mapping_distance)
        else: 
            indices = np.where(np.array(laserScan.ranges[min_index:max_index]) < self.mapping_distance)
        
        #Get svea's pose
        svea_x = state[0]
        svea_y = state[1]
        yaw = state[2]

        #Calculate lidar pose in map frame             
        length = self.lidar_to_base 
        lidar_coord = [svea_x+length*math.cos(yaw), svea_y+length*math.sin(yaw)]

        #Mapping dynamic obstacles
        obs_points = []
        idx_list = []
        for idx in indices[0]:  
            idx = idx + min_index
            idx_list.append(idx)
            #calculate angle to laser point
            angle = laserScan.angle_min + idx*laserScan.angle_increment
            #coordinate of the laser point/obstacle
            lidar_range = laserScan.ranges[idx]
            obs_coord = [lidar_coord[0]+lidar_range*math.cos(yaw+angle), lidar_coord[1]+lidar_range*math.sin(yaw+angle)]
            obs_points.append(obs_coord)

        # Update list of obstacles with the new obstacle 
        
        obstacles_list = []# rospy.get_param('/team_5_floor2/lidar_obstacles')
        #self.publish_obstacles(obstacles_list)
        #rospy.sleep(4)
        obstacles_list.append(obs_points)
        try:
            rospy.delete_param('/team_5_floor2/lidar_obstacles')
        except KeyError:
            print("value not set")
        rospy.set_param('/team_5_floor2/lidar_obstacles',obstacles_list)
        self.publish_obstacles(obstacles_list)

    def publish_obstacles(self, obstacles_list):
        Interface = MapROSInterface()
        Interface.publish(obstacles_list)

    def print_every_second(self, message, data):
                # print once per second
        if (self.last_time - self.print_time)  > 1:
            print(message, data)
            self.print_time = self.last_time

    def reset_isfinished(self):
            self.is_finished = False
