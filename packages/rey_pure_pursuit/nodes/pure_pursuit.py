#!/usr/bin/env python
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped, SegmentList, Segment, LanePose
import copy

class PurePursuit():
    def __init__(self):

        rospy.init_node('pure_pursuit_node', anonymous=True)

        # FOR PURE PURSUIT
        self.K = 0.8
        self.K_white = 0.153 # 0.13 (best in submission (4641): 0.25) (best but sketchy (4642): 0.27)
        self.K_yellow = 0.112 # 0.13 (best in submission (4641): 0.25) (best but sketchy (4642): 0.27)
        self.num_lines_th = 0 # 2 (best in submission (4641): 0)
        self.offset_white = 0.31 # 0.3 (best in submission (4641): 0.5) (best but sketchy (4642): 0.4)
        self.offset_yellow = 0.31 # 0.3 (best in submission (4641): 0.5) (best but sketchy (4642): 0.4)
        self.v = 0.15 # 5.0 (best in submission (4641): 1.) (best but sketchy (4642): 1.)
        self.v_min = 0.0 # min velocity (best in submission: 0.0)
        self.v_max = 0.65 # max velocity # 20 (best in submission (4641): 10.0) (best but sketchy (4642): 10.0)
        self.v_factor = 6.5 # to adjust v according to omega (v inversely proportional) (best in submission (4641): 2.5) (best but sketchy (4642): 2.5)
        self.x_comp_max = 0.325
        self.w_factor = 3.0
        
        # Add subscriber(s)
        # self.line_sub = rospy.Subscriber('/default/ground_projection/lineseglist_out', SegmentList, self.pure_pursuit_callback, queue_size = 1)
        self.line_sub = rospy.Subscriber('/bebek/lane_filter_node/seglist_filtered', SegmentList, self.pure_pursuit_callback, queue_size = 1)
        # self.lane_pose_sub = rospy.Subscriber('/bebek/lane_filter_node/lane_pose', LanePose, self.lane_pose_callback, queue_size = 1)

        # Add publisher(s)
        # self.car_cmd_pub = rospy.Publisher('/default/joy_mapper_node/car_cmd', Twist2DStamped, queue_size=1)
        self.car_cmd_pub = rospy.Publisher('/bebek/lane_controller_node/car_cmd', Twist2DStamped, queue_size=1)
        
        # To store some useful history
        self.last_omega = 0
        self.last_v = 0

        # LOG
        # self.plot_data = []
        # self.f = open("/code/catkin_ws/src/rey_core/packages/rey_pure_pursuit/dump6_pose.txt","w+")
        # self.t = open("/code/catkin_ws/src/rey_core/packages/rey_pure_pursuit/dump6_cmd.txt","w+")
        # rospy.loginfo('Initialized.')


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


    def pure_pursuit_callback(self, data):        

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
                # print('color: %d' % line[2])
                if line[2] == 0: # If color is white
                    white_lines.append([line[0],line[1]])
                elif line[2] == 1: # If color is yellow
                    yellow_lines.append([line[0],line[1]])
            # print('white_lines: %d' % len(white_lines))
            # print('yellow_lines: %d' % len(yellow_lines))

    
        '''Method 1: assume the centroid of all line points to be the follow point'''
        if len(lines) > self.num_lines_th: # If lines are detected
            if len(white_lines) > self.num_lines_th and len(yellow_lines) > self.num_lines_th: # If both white and yellow exist
                total_white_lines = np.array([0.,0.])
                total_yellow_lines = np.array([0.,0.])
                for line in white_lines:
                    total_white_lines += np.array(line[0])
                    total_white_lines += np.array(line[1])
                for line in yellow_lines:
                    total_yellow_lines += np.array(line[0])
                    total_yellow_lines += np.array(line[1])
                mean_white = total_white_lines / float(len(white_lines))
                mean_yellow = total_yellow_lines / float(len(yellow_lines))
                follow_point = (mean_white + mean_yellow) / 2.
                duck_to_point = follow_point
                dist = np.linalg.norm(duck_to_point) # a scalar
                unit_duck_to_point = duck_to_point / dist # (x,y,z)
                z_comp = duck_to_point[1]
                x_comp = duck_to_point[0]
                norm_x_comp = abs(x_comp) / self.x_comp_max # want this to be between [0,1] ~ if x comp is larger than 0.3, this should be 1
                norm_x_comp = np.clip(norm_x_comp,0,1)
                sin_alpha = z_comp / dist
                K_updated = self.K / ((norm_x_comp + 1e-5)**self.w_factor)  # when small horizontal error, want K to be large so robot does not turn (and vice-versa) 
                K_updated = np.clip(K_updated, 0.2, 1.0)
                omega = sin_alpha / K_updated
                
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
                norm_x_comp = abs(x_comp) / self.x_comp_max # want this to be between [0,1] ~ if x comp is larger than 0.3, this should be 1
                norm_x_comp = np.clip(norm_x_comp,0,1)
                sin_alpha = z_comp / dist
                K_updated = self.K_white / ((norm_x_comp + 1e-5)**self.w_factor)  # when small horizontal error, want K to be large so robot does not turn (and vice-versa) 
                K_updated = np.clip(K_updated, 0.2, 1.0)
                omega = sin_alpha / K_updated
        
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
                norm_x_comp = abs(x_comp) / self.x_comp_max # want this to be between [0,1] ~ if x comp is larger than 0.3, this should be 1
                norm_x_comp = np.clip(norm_x_comp,0,1)
                sin_alpha = z_comp / dist
                K_updated = self.K_yellow / ((norm_x_comp + 1e-5)**self.w_factor)  # when small horizontal error, want K to be large so robot does not turn (and vice-versa) 
                K_updated = np.clip(K_updated, 0.2, 1.0)
                omega = sin_alpha / K_updated

            v = self.v / (abs((omega + 1e-5))**self.v_factor)
            v = np.clip(v, self.v_min, self.v_max)
        
        else:
            omega = 0. # -1.0
            v = 0.1

        print('v,omega:',v,omega)

        '''Publish the command'''
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header = data.header
        car_cmd_msg.v = v
        car_cmd_msg.omega = omega
        self.car_cmd_pub.publish(car_cmd_msg)


    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = PurePursuit()
        node.spin()
    except rospy.ROSInterruptException:
        pass