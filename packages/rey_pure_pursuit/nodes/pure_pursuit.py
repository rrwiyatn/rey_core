#!/usr/bin/env python
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped, SegmentList, Segment, LanePose
import copy

class PurePursuit():
    def __init__(self):

        rospy.init_node('pure_pursuit_node', anonymous=True)

        self.K_white = 0.225 # 0.15 - 0.1
        self.K_yellow = 0.17 # 0.3 - 0.3
        self.num_lines_th = 2 # 2
        self.offset_white = 0.65 # 0.7 - 1.1 
        self.offset_yellow = 0.6 # 0.3 - 0.15
        self.v = 0.25 # 0.3 - 0.5
        
        # Add subscriber(s) # TODO: change topic name, message type, callback name
        # self.line_sub = rospy.Subscriber('/default/ground_projection/lineseglist_out', SegmentList, self.pure_pursuit_callback, queue_size = 1)
        self.line_sub = rospy.Subscriber('/bebek/lane_filter_node/seglist_filtered', SegmentList, self.pure_pursuit_callback, queue_size = 1)
        self.lane_pose_sub = rospy.Subscriber('/bebek/lane_filter_node/lane_pose', LanePose, self.lane_pose_callback, queue_size = 1)

        # Add publisher(s)  # TODO: change topic name
        self.car_cmd_pub = rospy.Publisher('/bebek/joy_mapper_node/car_cmd', Twist2DStamped, queue_size=1)


        # To store subscribed data
        # self.lines = []
        # self.d = None
        # self.phi = None
        # self.req_line = False
        # self.req_pose = False

        # To store some useful history
        self.last_omega = 0
        self.last_v = 0

        # LOG
        self.plot_data = []
        self.f = open("/code/catkin_ws/src/rey_core/packages/rey_pure_pursuit/dump4.txt","w+")

        rospy.loginfo('Initialized.')


    def lane_pose_callback(self, data):
        d = data.d
        d_ref = data.d_ref
        sigma_d = data.sigma_d
        phi = data.phi
        phi_ref = data.phi_ref
        sigma_phi = data.sigma_phi
        dump = [d,d_ref,sigma_d,phi,phi_ref,sigma_phi]
        self.plot_data.append(dump)

        # STRING
        d = str(d)
        d_ref = str(d_ref)
        sigma_d = str(sigma_d)
        phi = str(phi)
        phi_ref = str(phi_ref)
        sigma_phi = str(sigma_phi)
        str_dump = d + ',' + d_ref + ',' + sigma_d + ',' + phi + ',' + phi_ref + ',' + sigma_phi + '\n'
        self.f.write(str_dump)


    def wrap_angle(self, angle):
        new_angle = angle % (np.pi * 2)
        if new_angle < 0:
            new_angle = new_angle + (2 * np.pi)
        elif new_angle >= np.pi:
            new_angle = new_angle - (2 * np.pi)
        return new_angle
    

    # def line_sub_callback(self, data):
    #     if self.req_line:
    #         self.lines = ...
    #     self.req_line = False

    
    # def lane_pose_sub_callback(self, data):
    #     if self.req_pose:
    #         self.d = 
    #         self.phi = 
    #     self.req_pose = False

    
    # def request_lines(self):
    #     self.req_line = True
    #     while self.req_line and not rospy.is_shutdown():
    #         time.sleep(0.001)
    #     self.req_line = True
    #     while self.req_line and not rospy.is_shutdown():
    #         time.sleep(0.001)
    #     return self.lines


    # def request_lane_pose(self):
    #     self.req_pose = True
    #     while self.req_pose and not rospy.is_shutdown():
    #         time.sleep(0.001)
    #     self.req_pose = True
    #     while self.req_pose and not rospy.is_shutdown():
    #         time.sleep(0.001)
    #     return self.d, self.phi


    def pure_pursuit_callback(self, data):
        # Return the angular velocity in order to control the Duckiebot using a pure pursuit algorithm.
        # Parameters:
        #     env: Duckietown simulator
        #     pos: global position of the Duckiebot
        #     angle: global angle of the Duckiebot
        # Outputs:
        #     v: linear veloicy in m/s.
        #     omega: angular velocity, in rad/sec. Right is negative, left is positive.
        

        '''Get line segments'''
        linesegs = data.segments # A list of line segments
        lines = [] # List of lines, every line is in the format [[x1,y1],[x2,y2],color]
        for i in range(len(linesegs)):
            pt1 = [linesegs[i].points[0].x,linesegs[i].points[0].y]
            pt2 = [linesegs[i].points[1].x,linesegs[i].points[1].y]
            color = linesegs[i].color
            lines.append([pt1,pt2,color])

        '''Separate white and yellow lines for convenience'''
        if len(lines) > self.num_lines_th: # If lines are detected, separate white and yellow lines
            white_lines = []
            yellow_lines = []
            for line in lines:
                print('color: %d' % line[2])
                if line[2] == 0: # If color is white
                    white_lines.append([line[0],line[1]])
                elif line[2] == 1: # If color is yellow
                    yellow_lines.append([line[0],line[1]])
            # white_slope = 0
            # yellow_slope = 0
            # for line in white_lines:
            #     white_slope += (line[1] - line[0])
            # white_slope_sign = white_slope[1]/white_slope[0]
            
            print('white_lines: %d' % len(white_lines))
            print('yellow_lines: %d' % len(yellow_lines))

        
        '''Method 1: assume the centroid of all line points to be the follow point'''
        if len(lines) > self.num_lines_th: # If lines are detected
            if len(white_lines) > self.num_lines_th and len(yellow_lines) > self.num_lines_th: # If both white and yellow exist
                total_white_lines = np.array([0.,0.])
                total_yellow_lines = np.array([0.,0.])
                for line in white_lines:
                    total_white_lines += np.array(line[0])
                    total_white_lines += np.array(line[1])
                for line in yellow_lines:
                    total_yellow_lines[0] += np.array(line[0])
                    total_yellow_lines[1] += np.array(line[1])
                mean_white = total_white_lines / float(len(white_lines))
                mean_yellow = total_yellow_lines / float(len(yellow_lines))
                follow_point = (mean_white + mean_yellow) / 2.
                duck_to_point = follow_point
                dist = np.linalg.norm(duck_to_point) # a scalar
                unit_duck_to_point = duck_to_point / dist # (x,y,z)
                z_comp = duck_to_point[1]
                x_comp = duck_to_point[0]
                # angle_between_x_axis_and_target = np.arctan2(-z_comp,x_comp)
                # alpha = angle_between_x_axis_and_target
                # omega = -(np.sin(alpha)) / (self.K) # Scaling dist with speed
                sin_alpha = z_comp / dist
                omega = sin_alpha / self.K
                v = self.v
                self.last_omega = omega
                self.last_v = v
            elif len(white_lines) > self.num_lines_th and len(yellow_lines) <= self.num_lines_th: # If only white lines
                total_lines = np.array([0.,0.])
                for line in white_lines:
                    total_lines += np.array(line[0])
                    total_lines += np.array(line[1])
                follow_point = total_lines / (len(white_lines))
                follow_point[1] += (self.offset_white)
                duck_to_point = follow_point
                dist = np.linalg.norm(duck_to_point) # a scalar
                unit_duck_to_point = duck_to_point / dist # (x,y,z)
                z_comp = duck_to_point[1]
                x_comp = duck_to_point[0]
                # angle_between_x_axis_and_target = np.arctan2(-z_comp,x_comp)
                # alpha = angle_between_x_axis_and_target
                # omega = (np.sin(alpha)) / (self.K) # Scaling dist with speed
                sin_alpha = z_comp / dist
                omega = sin_alpha / (self.K_white)
                v = self.v
                self.last_omega = omega
                self.last_v = v
            elif len(yellow_lines) > self.num_lines_th and len(white_lines) <= self.num_lines_th: # If only yellow lines
                total_lines = np.array([0.,0.])
                for line in yellow_lines:
                    total_lines += np.array(line[0])
                    total_lines += np.array(line[1])
                follow_point = total_lines / (len(yellow_lines))
                follow_point[1] -= (self.offset_yellow)
                duck_to_point = follow_point
                dist = np.linalg.norm(duck_to_point) # a scalar
                unit_duck_to_point = duck_to_point / dist # (x,y,z)
                z_comp = duck_to_point[1]
                x_comp = duck_to_point[0]
                #angle_between_x_axis_and_target = np.arctan2(-z_comp,x_comp)
                #alpha = angle_between_x_axis_and_target
                #omega = -(np.sin(alpha)) / (self.K) # Scaling dist with speed
                sin_alpha = z_comp / dist
                omega = sin_alpha / self.K_yellow
                v = self.v
                self.last_omega = omega
                self.last_v = v
            else:
                omega = 0. # -1.0
                v = 0.15
        else:
            omega = 0. # -1.0
            v = 0.15

        # Publish the command
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header = data.header
        car_cmd_msg.v = v
        car_cmd_msg.omega = omega
        self.car_cmd_pub.publish(car_cmd_msg)

        return 

    def spin(self):
        rospy.spin()


# def main():


if __name__ == '__main__':
    try:
        node = PurePursuit()
        node.spin()
    except rospy.ROSInterruptException:
        pass