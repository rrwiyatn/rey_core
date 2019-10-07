#!/usr/bin/env python
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped
import copy

class PurePursuit():
    def __init__(self):
        self.K = 0.2
        self.follow_dist = 0.25
        
        # Add subscriber(s) # TODO: change topic name, message type, callback name
        self.start_planner_sub = rospy.Subscriber('/?', ArmPlanningRun, self.pure_pursuit_callback, queue_size = 1)
        self.trajectory_sub = 

        # Add publisher(s)  # TODO: change topic name
        self.car_cmd_pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)


    def wrap_angle(self, angle):
        new_angle = angle % (np.pi * 2)
        if new_angle < 0:
            new_angle = new_angle + (2 * np.pi)
        elif new_angle >= np.pi:
            new_angle = new_angle - (2 * np.pi)
        return new_angle
    

    def pure_pursuit_callback(self, env, pos, angle):
        # Return the angular velocity in order to control the Duckiebot using a pure pursuit algorithm.
        # Parameters:
        #     env: Duckietown simulator
        #     pos: global position of the Duckiebot
        #     angle: global angle of the Duckiebot
        # Outputs:
        #     v: linear veloicy in m/s.
        #     omega: angular velocity, in rad/sec. Right is negative, left is positive.
        
        
        closest_curve_point = env.unwrapped.closest_curve_point
        
        # Find the curve point closest to the agent, and the tangent at that point
        closest_point, closest_tangent = closest_curve_point(pos, angle)

        iterations = 0
        
        lookup_distance = copy.deepcopy(self.follow_dist)
        multiplier = 0.5
        curve_point = None
        
        while iterations < 10:            
            follow_point = closest_point + (lookup_distance * closest_tangent)
            curve_point, _ = closest_curve_point(follow_point, angle)
            if curve_point is not None: # If we have a valid point on the curve, stop
                break
            iterations += 1
            lookup_distance *= multiplier
        
        duck_to_point = curve_point - pos # (x,y,z)
        dist = np.linalg.norm(duck_to_point) # a scalar
        unit_duck_to_point = duck_to_point / dist # (x,y,z)
        z_comp = duck_to_point[2]
        x_comp = duck_to_point[0]
        angle_between_x_axis_and_target = np.arctan2(-z_comp,x_comp)
        alpha = angle - angle_between_x_axis_and_target
        
        omega = -(np.sin(alpha)) / (K) # Scaling dist with speed
        v = 0.5
        
        # Publish the command
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header = pose_msg.header # TODO: copy from whatever we subscribe from
        car_cmd_msg.v = v
        car_cmd_msg.omega = omega
        self.car_cmd_pub.publish(car_cmd_msg)

        return 
