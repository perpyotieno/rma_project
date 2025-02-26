#!/usr/bin/env python

import rospy
import tf
import numpy as np
import psutil
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from scipy.spatial import KDTree
from mavros_msgs.msg import PositionTarget, State
from gazebo_msgs.srv import SpawnModel, DeleteModel
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Pose
from math import sqrt
import os


class UAVExplorer:
    def __init__(self):
        rospy.init_node('uav_explorer', anonymous=True)
        
        # Subscribers
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, self.map_callback)
        rospy.Subscriber("/mavros/state", State, self.state_callback)

        # Publishers
        self.next_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        self.path_pub = rospy.Publisher("/uav_exploration_path", Path, queue_size=10)
        self.cpu_mem_pub = rospy.Publisher("/uav/performance", Float32, queue_size=10)
        self.uav_cmd_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        
        # UAV State
        self.current_position = None
        self.map_data = None
        self.current_state = None
        self.model_spawned = False
        self.model_name = "uav"
        self.flight_started = False
        
        
        
        # Performance Metrics
        self.start_time = None
        self.total_distance = 0.0
        self.last_position = None
        self.coverage = 0.0
        self.visited_cells = set()
        self.grid_resolution = 1.0  # For area coverage calculation
        self.x_min, self.x_max = -10, 10  # Define area bounds
        self.y_min, self.y_max = -10, 10
        self.total_cells = ((self.x_max - self.x_min) / self.grid_resolution) * ((self.y_max - self.y_min) / self.grid_resolution)
    
        
        
        rospy.loginfo("UAV Explorer Initialized!")
        
        # Procedures
        self.spawn_uav_in_gazebo()
        self.wait_for_connection()
        self.set_offboard_and_arm()
        self.run_exploration()
        
    def state_callback(self, msg):
        self.current_state = msg
    
    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        
        if not self.flight_started:
            self.start_time = rospy.Time.now()
            self.flight_started = True
            
        #Update performance metrics
        self.update_metrics()
        self.track_performance()
        self.visualize_path()
    
    def map_callback(self, msg):
        self.map_data = msg
      #  self.identify_frontiers()
    
    def update_metrics(self):
        # Flight Time
        flight_time = (rospy.Time.now() - self.start_time).to_sec()
        
        # Path Length
        if self.last_position:
            dx = self.current_position.x - self.last_position.x
            dy = self.current_position.y - self.last_position.y
            dz = self.current_position.z - self.last_position.z
            distance = sqrt(dx**2 + dy**2 + dz**2)
            self.total_distance += distance
        self.last_position = self.current_position
        
        # Area Coverage
        cell_x = int((self.current_position.x - self.x_min) / self.grid_resolution)
        cell_y = int((self.current_position.y - self.y_min) / self.grid_resolution)
        self.visited_cells.add((cell_x, cell_y))
        self.coverage = (len(self.visited_cells) / self.total_cells) * 100
        
        # Log metrics
        rospy.loginfo_throttle(5, f"Flight Time: {flight_time:.2f}s, Total Distance: {self.total_distance:.2f}m, Coverage: {self.coverage:.2f}%")
    
    
    def track_performance(self):
        cpu_usage = psutil.cpu_percent(interval=None)
        memory_usage = psutil.virtual_memory().percent
        rospy.loginfo_throttle(5, f"CPU Usage: {cpu_usage}%, Memory Usage: {memory_usage}%")
        
        # Publish CPU usage
        self.cpu_mem_pub.publish(Float32(cpu_usage))
    
    def visualize_path(self):
        if self.current_position:
            path = Path()
            path.header.frame_id = "map"
            pose = PoseStamped()
            pose.pose.position = self.current_position
            path.poses.append(pose)
            self.path_pub.publish(path)
    
    
    def visualize_waypoint(self, goal):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = goal.pose.position
        marker.scale.x = marker.scale.y = marker.scale.z = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)
    

    
        
    def spawn_uav_in_gazebo(self):
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        try:
            spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
            uav_sdf_path = "/home/perpetua/tigris_ws/src/smb_common/model.sdf"  # Change this path
            with open(uav_sdf_path, "r") as f:
                uav_sdf = f.read()
                
              
            initial_pose = Pose()
            initial_pose.position.x = 0.0
            initial_pose.position.y = 0.0
            initial_pose.position.z = 0.5  # Slightly above ground to avoid collision  
              
            spawn_model(model_name=self.model_name, model_xml=uav_sdf, robot_namespace="", initial_pose=initial_pose, reference_frame="world")
            rospy.loginfo("UAV spawned in Gazebo.")
            self.model_spawned = True
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to spawn UAV: {e}")  
             
    
    def wait_for_connection(self):
        rospy.loginfo("Waiting for MAVROS connection...")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and not self.current_state:
            rospy.loginfo_throttle(5, "Connecting to MAVROS...")
            rate.sleep()
        rospy.loginfo("Connected to MAVROS.")
    
    
    
    def set_offboard_and_arm(self):
        rospy.loginfo("Setting OFFBOARD mode and arming UAV...")
        rate = rospy.Rate(20)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 2  # Take-off altitude
        
        
        # Send a few setpoints before starting
        for _ in range(100):
            pose.header.stamp = rospy.Time.now()
            self.uav_cmd_pub.publish(pose)
            rate.sleep()
            
            
         # Service proxies for arming and mode change
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        arm_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        last_req = rospy.Time.now()
        while not rospy.is_shutdown():
                # Continuously publish setpoints
            pose.header.stamp = rospy.Time.now()
            self.uav_cmd_pub.publish(pose)
            
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req > rospy.Duration(1.0)):
                set_mode_client(base_mode=0,custom_mode="OFFBOARD")
                last_req = rospy.Time.now()
                rospy.loginfo("Requested OFFBOARD mode.")
            else:
                if not self.current_state.armed and (rospy.Time.now() - last_req > rospy.Duration(1.0)):
                    arm_client(True)
                    last_req = rospy.Time.now()
                    rospy.loginfo("Requested arming.")
                    
            if self.current_state.armed and self.current_state.mode == "OFFBOARD":
                rospy.loginfo("UAV is armed and in OFFBOARD mode.")
                break
            
            # self.uav_cmd_pub.publish(pose)
            rate.sleep()
    
    
    def run_exploration(self):
        rospy.loginfo("Starting exploration...")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.identify_frontiers()
            rate.sleep()
    
    
    def identify_frontiers(self):
        if not self.current_position:
            rospy.logwarn("Current UAV position unknown.")
            return
        
        rospy.loginfo("Identifying next frontier...")
        # Simulate frontier points; in practice, replace with actual frontier detection
        points = np.random.uniform(-10, 10, (50, 3))
        tree = KDTree(points)
        _, idx = tree.query([self.current_position.x, self.current_position.y, self.current_position.z])
        next_goal_point = points[idx]
        
        next_goal = PoseStamped()
        next_goal.header.frame_id = "map"
        next_goal.pose.position.x = next_goal_point[0]
        next_goal.pose.position.y = next_goal_point[1]
        next_goal.pose.position.z = max(2.0, next_goal_point[2])  # Ensure altitude is safe
        
        self.visualize_waypoint(next_goal)
        self.move_uav(next_goal)
    

    def move_uav(self, goal):
        rospy.loginfo(f"Moving UAV to waypoint: x={goal.pose.position.x}, y={goal.pose.position.y}, z={goal.pose.position.z}")
        rate = rospy.Rate(20)
        tolerance = 0.5  # Meters
        while not rospy.is_shutdown():
            if not self.current_state.armed or self.current_state.mode != "OFFBOARD":
                rospy.logwarn("UAV not armed or not in OFFBOARD mode. Cannot move.")
                break

            self.uav_cmd_pub.publish(goal)
            
            # Check if the UAV reached the goal
            distance = sqrt(
                (self.current_position.x - goal.pose.position.x) ** 2 +
                (self.current_position.y - goal.pose.position.y) ** 2 +
                (self.current_position.z - goal.pose.position.z) ** 2
            )
            if distance < tolerance:
                rospy.loginfo("Reached waypoint.")
                break
            rate.sleep()
    
    
    # def track_performance(self):
    #     cpu_usage = psutil.cpu_percent(interval=1)
    #     memory_usage = psutil.virtual_memory().percent
    #     rospy.loginfo(f"CPU: {cpu_usage}%, Memory: {memory_usage}%")
    #     self.cpu_mem_pub.publish(Float32(cpu_usage))

            
   
    def stop(self):
        rospy.signal_shutdown("Exploration complete.")

    # def run(self):
    #     rospy.spin()

if __name__ == '__main__':
    try:
        explorer = UAVExplorer()
    except rospy.ROSInterruptException:
        pass