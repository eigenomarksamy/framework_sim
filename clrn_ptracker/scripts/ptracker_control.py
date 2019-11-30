#! /usr/bin/python3


import rospy
import sys
import os
import time
import math
import numpy as np
import matplotlib.pyplot as plt
sys.path.append("/home/oks/catkin_ws/src/framework_sim/clrn_ptracker/scripts/")
import controller
import configurator


def init_nav():
    config = configurator.get_config()
    path = parse_path(config)
    return config, path


def parse_path(config):
    course_txt = ["wpx\n", "wpy\n", "fg\n", "cx\n", "cy\n", "cyaw\n", "ck\n", "cs\n", "tv\n", "sp\n"]
    waypoints_x = []
    waypoints_y = []
    final_goal = []
    course_x = []
    course_y = []
    course_yaw = []
    course_k = []
    course_s = []
    target_velocity = []
    speed_profile = []
    try:
        course_file = open(config.gen_files_path, 'r')
        lines = course_file.readlines()
        for line in range(len(lines) - 1):
            if lines[line] == course_txt[0]:
                for i in lines[line + 1].split():
                    waypoints_x.append(float(i))
            elif lines[line] == course_txt[1]:
                for i in lines[line + 1].split():
                    waypoints_y.append(float(i))
            elif lines[line] == course_txt[2]:
                for i in lines[line + 1].split():
                    final_goal.append(float(i))
            elif lines[line] == course_txt[3]:
                for i in lines[line + 1].split():
                    course_x.append(float(i))
            elif lines[line] == course_txt[4]:
                for i in lines[line + 1].split():
                    course_y.append(float(i))
            elif lines[line] == course_txt[5]:
                for i in lines[line + 1].split():
                    course_yaw.append(float(i))
            elif lines[line] == course_txt[6]:
                for i in lines[line + 1].split():
                    course_k.append(float(i))
            elif lines[line] == course_txt[7]:
                for i in lines[line + 1].split():
                    course_s.append(float(i))
            elif lines[line] == course_txt[8]:
                for i in lines[line + 1].split():
                    target_velocity.append(float(i))
            elif lines[line] == course_txt[9]:
                for i in lines[line + 1].split():
                    speed_profile.append(float(i))
    except:
        print("No File!")
    return [waypoints_x, waypoints_y, final_goal, course_x, course_y, course_yaw, course_k, course_s, target_velocity, speed_profile]


def exec_nav(config, path):
    wp_x = path[0]
    wp_y = path[1]
    fg = path[2]
    cx = path[3]
    cy = path[4]
    cyaw = path[5]
    ck = path[6]
    cs = path[7]
    tv = path[8]
    sp = path[9]
    waypoints = list(zip(wp_x, wp_y))
    waypoints_np = np.array(waypoints)
    wp_distance = []
    for i in range(1, waypoints_np.shape[0]):
        wp_distance.append(
                np.sqrt((waypoints_np[i, 0] - waypoints_np[i-1, 0])**2 +
                        (waypoints_np[i, 1] - waypoints_np[i-1, 1])**2))
    wp_distance.append(0)
    wp_interp      = []
    wp_interp_hash = []
    interp_counter = 0
    for i in range(waypoints_np.shape[0] - 1):
        wp_interp.append(list(waypoints_np[i]))
        wp_interp_hash.append(interp_counter)   
        interp_counter+=1
        num_pts_to_interp = int(np.floor(wp_distance[i] /\
                                     float(0.01)) - 1)
        wp_vector = waypoints_np[i+1] - waypoints_np[i]
        wp_uvector = wp_vector / np.linalg.norm(wp_vector)
        for j in range(num_pts_to_interp):
            next_wp_vector = 0.01 * float(j+1) * wp_uvector
            wp_interp.append(list(waypoints_np[i] + next_wp_vector))
            interp_counter+=1
    wp_interp.append(list(waypoints_np[-1]))
    wp_interp_hash.append(interp_counter)   
    interp_counter+=1
    num_iterations = 10
    if (10 < 1):
        num_iterations = 1


def main():
    while True:
        # try:
        print('Get Ready for the Fuckin\' Ride')
        config, path = init_nav()
        exec_nav(config, path)
        print('Done.')
        return
        # except rospy.ROSInterruptException:
            # print('ROS Connection Lost!')
            # time.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nYou chose to leave. Bye!')