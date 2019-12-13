#! /usr/bin/env python

import sys

from dbw_mkz_msgs.msg import ThrottleCmd, BrakeCmd, SteeringCmd

sys.path.append("/home/oks/catkin_ws/src/framework_sim/clrn_control/scripts/")
import car_config

class Cmd:
    def __init__(self, thrtl_obj, steer_obj, brake_obj):
        self.thrtl = thrtl_obj
        self.steer = steer_obj
        self.brake = brake_obj


def init_ros(config):
    try:
        rospy.init_node(config.llc_out_node)
        pub_obj_thrtl = rospy.Publisher(config.thrtl_topic, ThrottleCmd, queue_size=10)
        pub_obj_steer = rospy.Publisher(config.steer_topic, SteeringCmd, queue_size=10)
        pub_obj_brake = rospy.Publisher(config.brake_topic, BrakeCmd, queue_size=10)
        cmd_obj = Cmd(pub_obj_thrtl, pub_obj_steer, pub_obj_brake)
        return cmd_obj
    except rospy.ROSInterruptException:
        print('ROS Connection Lost!')
        return


def publish_cmd(cmd_obj):
    return


def main():
    while True:
        config_obj = config_config.CarConfig('fusion')
        cmd_obj = init_ros(config_obj)
        publish_cmd(cmd_obj)


if __name__ == '__main__':
    main()