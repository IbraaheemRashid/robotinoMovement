#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

from stable_baselines3 import SAC

class SocialNavigationNode(Node):
    def __init__(self):
        super().__init__('social_navigation')
        
        rl_model = "SAC"
        
        self.odom_sub = self.create_subscription(
            Odometry, 'robotino/odom', self.odom_callback, 10)

        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10)
            
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.odom_data = None
        self.goal_pose = None
        
        self.humans = [
            {"id": 1, "x": 2.0, "y": 1.0, "angle": 0.0},
            {"id": 2, "x": -1.0, "y": 2.0, "angle": 1.57},
            {"id": 3, "x": 3.0, "y": -2.0, "angle": 0.7},
            {"id": 4, "x": -2.0, "y": -3.0, "angle": 2.1},
            {"id": 5, "x": 4.0, "y": 4.0, "angle": 3.14}
        ]
        
        self.tables = [
            {"id": 1, "x": 3.5, "y": -3.5, "angle": 0.0, "width": 0.8, "length": 1.2},
            {"id": 2, "x": -3.5, "y": 3.5, "angle": 1.57, "width": 0.8, "length": 1.2}
        ]
        
        self.plants = [
            {"id": 1, "x": 4.0, "y": 0.0, "angle": 0.0, "radius": 0.3},
            {"id": 2, "x": -4.0, "y": 0.0, "angle": 0.0, "radius": 0.3}
        ]
        
        self.laptops = [
            {"id": 1, "x": 3.0, "y": 3.0, "angle": 0.0, "width": 0.3, "length": 0.2},
            {"id": 2, "x": -3.0, "y": -3.0, "angle": 0.0, "width": 0.3, "length": 0.2}
        ]
        
        self.walls = [
            [-5.0, -5.0, 5.0, -5.0], 
            [5.0, -5.0, 5.0, 5.0],   
            [5.0, 5.0, -5.0, 5.0],   
            [-5.0, 5.0, -5.0, -5.0],
            [-2.0, -2.0, 2.0, -2.0],
            [2.0, -2.0, 2.0, 2.0],
            [2.0, 2.0, -2.0, 2.0],
            [-2.0, 2.0, -2.0, -2.0]
        ]
        
        import gym
        SB3_POLICY = "MlpPolicy"
        env = gym.make('Pendulum-v1')  
        
        if rl_model == "SAC":
            self.model = SAC(SB3_POLICY, env, verbose=1)
            model_path = "/home/mir/SocNavGym/tests/models/dev.yaml_SAC_1743335927.1838963/model.zip"
            try:
                self.model.load(model_path, print_system_info=True)
                self.using_trained_model = True
                print(f"Successfully loaded model from {model_path}")
            except Exception as e:
                print(f"Failed to load model: {e}")
                print("Using fallback approach with simplified observations")
                self.using_trained_model = False
        
        # Timer for updates
        self.create_timer(0.1, self.update_loop)
        
        print("Social navigation node started")

    def odom_callback(self, msg):
        self.odom_data = msg
        
    def goal_callback(self, msg):
        self.goal_pose = msg
        print(f"New goal: {msg.pose.position.x}, {msg.pose.position.y}")

    def update_loop(self):
        if self.odom_data is None or self.goal_pose is None:
            return
        
        try:
            if self.using_trained_model:
                obs = self.make_full_observation()
            else:
                obs = self.make_simple_observation()
            
            action, _ = self.model.predict(obs, deterministic=True)
            print
            
            cmd_vel = Twist()
            
            if isinstance(action, np.ndarray) and action.size > 1:
                cmd_vel.linear.x = float(action[0])
                if action.size > 1:
                    cmd_vel.linear.y = float(action[1])
                if action.size > 2:
                    cmd_vel.angular.z = float(action[2])
            else:
                cmd_vel.linear.x = float(action[0] if isinstance(action, np.ndarray) else action)
                cmd_vel.angular.z = 0.0  # No rotation

            cmd_vel.linear.x = max(min(cmd_vel.linear.x, 0.5), -0.5)
            cmd_vel.linear.y = max(min(cmd_vel.linear.y, 0.5), -0.5)
            cmd_vel.angular.z = max(min(cmd_vel.angular.z, 1.0), -1.0)
            
            self.cmd_vel_pub.publish(cmd_vel)
            print(f"Publishing: lin_x={cmd_vel.linear.x:.2f}, lin_y={cmd_vel.linear.y:.2f}, ang_z={cmd_vel.angular.z:.2f}")
        
        except Exception as e:
            print(f"Error in update loop: {e}")
            self.cmd_vel_pub.publish(Twist())

    def make_simple_observation(self):
        """Create a simplified observation for fallback."""
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        
        goal_x = self.goal_pose.pose.position.x - robot_x
        goal_y = self.goal_pose.pose.position.y - robot_y
        
        distance_to_goal = np.sqrt(goal_x**2 + goal_y**2)
        
        angle_to_goal = np.arctan2(goal_y, goal_x)
        
        nearest_obstacle = 10.0
        
        for human in self.humans:
            human_x = human["x"]
            human_y = human["y"]
            dist = np.sqrt((human_x - robot_x)**2 + (human_y - robot_y)**2)
            if dist < nearest_obstacle:
                nearest_obstacle = dist
        
        return np.array([distance_to_goal, angle_to_goal, nearest_obstacle], dtype=np.float32)

    def make_full_observation(self):
        """Create full observation vector for SocNavGym model."""
        robot_x = self.odom_data.pose.pose.position.x
        robot_y = self.odom_data.pose.pose.position.y
        
        qx = self.odom_data.pose.pose.orientation.x
        qy = self.odom_data.pose.pose.orientation.y
        qz = self.odom_data.pose.pose.orientation.z
        qw = self.odom_data.pose.pose.orientation.w
        
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        robot_angle = np.arctan2(siny_cosp, cosy_cosp)
        
        goal_x = self.goal_pose.pose.position.x - robot_x
        goal_y = self.goal_pose.pose.position.y - robot_y
        
        obs = {}
        
        # Robot observation - [one-hot encoding, goal_x, goal_y, radius]
        robot_encoding = [1, 0, 0, 0, 0, 0]
        robot_obs = robot_encoding + [goal_x, goal_y, 0.25]
        obs["robot"] = np.array(robot_obs, dtype=np.float32)
        
        human_obs = []
        for human in self.humans:
            rel_x = human["x"] - robot_x
            rel_y = human["y"] - robot_y
            
            rel_angle = human["angle"] - robot_angle
            
            encoding = [0, 1, 0, 0, 0, 0]
            
            h_obs = encoding + [
                rel_x, rel_y, 
                np.sin(rel_angle), np.cos(rel_angle), 
                0.35,  
                0.0, 0.0,  
                0.0 
            ]
            
            human_obs.extend(h_obs)
        
        obs["humans"] = np.array(human_obs, dtype=np.float32) if human_obs else np.zeros((0,), dtype=np.float32)
        
        wall_obs = []
        for wall in self.walls:
            x1, y1, x2, y2 = wall
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            orientation = np.arctan2(y2 - y1, x2 - x1)
            rel_angle = orientation - robot_angle
            length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            
            rel_x = center_x - robot_x
            rel_y = center_y - robot_y
            
            encoding = [0, 0, 0, 0, 0, 1]
            
            w_obs = encoding + [
                rel_x, rel_y, 
                np.sin(rel_angle), np.cos(rel_angle), 
                length/2,  
                0.0, 0.0, 
                0.0 
            ]
            
            wall_obs.extend(w_obs)
        
        obs["walls"] = np.array(wall_obs, dtype=np.float32) if wall_obs else np.zeros((0,), dtype=np.float32)
        
        plant_obs = []
        for plant in self.plants:
            rel_x = plant["x"] - robot_x
            rel_y = plant["y"] - robot_y
            rel_angle = plant["angle"] - robot_angle
            
            encoding = [0, 0, 0, 0, 1, 0]
            
            p_obs = encoding + [
                rel_x, rel_y, 
                np.sin(rel_angle), np.cos(rel_angle), 
                plant["radius"],
                0.0, 0.0, 
                0.0 
            ]
            
            plant_obs.extend(p_obs)
        
        obs["plants"] = np.array(plant_obs, dtype=np.float32) if plant_obs else np.zeros((0,), dtype=np.float32)
        
        table_obs = []
        for table in self.tables:
            rel_x = table["x"] - robot_x
            rel_y = table["y"] - robot_y
            rel_angle = table["angle"] - robot_angle
            
            encoding = [0, 0, 1, 0, 0, 0]
            
            t_obs = encoding + [
                rel_x, rel_y, 
                np.sin(rel_angle), np.cos(rel_angle), 
                max(table["width"], table["length"])/2, 
                0.0, 0.0,
                0.0 
            ]
            
            table_obs.extend(t_obs)
        
        obs["tables"] = np.array(table_obs, dtype=np.float32) if table_obs else np.zeros((0,), dtype=np.float32)
        
        laptop_obs = []
        for laptop in self.laptops:
            rel_x = laptop["x"] - robot_x
            rel_y = laptop["y"] - robot_y
            rel_angle = laptop["angle"] - robot_angle
            
            encoding = [0, 0, 0, 1, 0, 0]
            
            l_obs = encoding + [
                rel_x, rel_y, 
                np.sin(rel_angle), np.cos(rel_angle), 
                max(laptop["width"], laptop["length"])/2,
                0.0, 0.0, 
                0.0 
            ]
            
            laptop_obs.extend(l_obs)
        
        obs["laptops"] = np.array(laptop_obs, dtype=np.float32) if laptop_obs else np.zeros((0,), dtype=np.float32)
        
        return self.flatten_observation(obs)
    
    def flatten_observation(self, obs_dict):
        """Flatten the observation dictionary to match the expected format."""
        keys = sorted(["robot", "humans", "laptops", "plants", "tables", "walls"])
        
        obs_list = []
        for k in keys:
            if k in obs_dict and len(obs_dict[k]) > 0:
                obs_list.append(obs_dict[k])
        
        flattened = np.concatenate([arr for arr in obs_list if arr.size > 0])
        
        if len(flattened) < 383:
            padding = np.zeros(383 - len(flattened), dtype=np.float32)
            flattened = np.concatenate([flattened, padding])
        elif len(flattened) > 383:
            flattened = flattened[:383]
            
        return flattened


def main(args=None):
    rclpy.init(args=args)
    node = SocialNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()