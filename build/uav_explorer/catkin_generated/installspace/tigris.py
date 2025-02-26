#!/usr/bin/env python3

import rospy
import numpy as np
import networkx as nx  # To handle tree structures
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import random

class TigrisPlanner:
    def __init__(self):
        # Initialize the planner
        rospy.init_node("tigris_planner", anonymous=True)

        # Define the publisher for the planned path
        self.path_pub = rospy.Publisher("/tigris_path", Path, queue_size=10)

         # Maximum cost budget
        self.budget = 1000 
        
         # Search area dimensions 
        self.grid_size = (1000, 1000)  
         # Number of iterations for the tree expansion
        self.max_iterations = 500 
        # Distance to extend the tree
        self.extend_distance = 50  
       

        # Initialize the directed graph for path planning
        self.graph = nx.DiGraph()

        # Include the start position
        self.start_pose = (50, 50)  

        # Add start node
        self.graph.add_node(self.start_pose, cost=0, reward=self.information_gain(self.start_pose))

        rospy.loginfo("Tigris Planner Initialized!")

    def information_gain(self, position):
        """Estimates information gain at a given position"""
        x, y = position
        # Higher reward near center
        return np.exp(-((x - 500)**2 + (y - 500)**2) / 50000)  

# Function to find the nearest node 
    def nearest_neighbor(self, sample):
        """Find nearest node in the graph"""
        return min(self.graph.nodes, key=lambda node: np.linalg.norm(np.array(node) - np.array(sample)))


# This function will calculate how to move from a node to another
    def steer(self, nearest, sample):
        """Move from nearest node towards sample within the maximum distance"""
        direction = np.array(sample) - np.array(nearest)
        distance = np.linalg.norm(direction)
        if distance > self.extend_distance:
            direction = (direction / distance) * self.extend_distance
        new_position = tuple(np.array(nearest) + direction)
        return new_position
    
# Function for the planner to find new points
    def expand_tree(self):
        """Expand the search tree"""
        for _ in range(self.max_iterations):
            # Sample a random point in the grid
            sample = (random.uniform(0, self.grid_size[0]), random.uniform(0, self.grid_size[1]))

            # Find nearest node 
            nearest = self.nearest_neighbor(sample)

            # Move towards sample
            new_position = self.steer(nearest, sample)

            # Calculate new cost and reward
            new_cost = self.graph.nodes[nearest]['cost'] + np.linalg.norm(np.array(new_position) - np.array(nearest))
            new_reward = self.information_gain(new_position)

            # Check budget constraint
            if new_cost > self.budget:
                continue  # Skip this node if over budget

            # Add new node to the tree 
            self.graph.add_node(new_position, cost=new_cost, reward=new_reward)
            self.graph.add_edge(nearest, new_position)

        rospy.loginfo("Tree Expansion Complete!")
        
# finds the best path with the most information gain
    def best_path(self):
        """Find the best path based on maximum information gain"""
        best_node = max(self.graph.nodes, key=lambda n: self.graph.nodes[n]['reward'])
        path = nx.shortest_path(self.graph, source=self.start_pose, target=best_node, weight="cost")
        return path

# Function to pblish the planner
    def publish_path(self, path):
        """Publish the planned path as a ROS Path message"""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for node in path:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = node[0]
            pose.pose.position.y = node[1]
            pose.pose.position.z = 0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        rospy.loginfo("Published Path!")
        
# The run method
    def run(self):
        """Main function to execute Tigris algorithm"""
        rate = rospy.Rate(1)  # 1 Hz loop

        while not rospy.is_shutdown():
            self.expand_tree()
            best_path = self.best_path()
            self.publish_path(best_path)
            rate.sleep()

if __name__ == "__main__":
    planner = TigrisPlanner()
    planner.run()
