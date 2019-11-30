#! /usr/bin/python3


from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dbw_mkz_msgs.msg import ThrottleCmd, BrakeCmd, SteeringCmd, GearCmd, TurnSignalCmd


class Config():
    def __init__(self, vehicle = 'mkz'):
        if vehicle == 'mkz':
            self.gen_files_path = '/home/oks/catkin_ws/src/framework_sim/gen_txtfiles/course_txt.txt'
            self.trkr_out_node = 'mkz_ptracker'
            self.trkr_out_topic =  '/mkz/ptracker'
            self.feedback_out_topic = '/mkz/odom'
            self.thrtl_topic = '/mkz/throttle_cmd'
            self.brake_topic = '/mkz/brake_cmd'
            self.steer_topic = '/mkz/steering_cmd'
            self.gear_topic = '/mkz/gear_cmd'
            self.cmd_vel_topic = 'mkz/cmd_vel'
            self.turn_signal_topic = '/mkz/turn_signal_cmd'
            self.feedback_msg_t = Odometry()
            self.thrtl_msg_t = ThrottleCmd()
            self.brake_msg_t = BrakeCmd()
            self.steer_msg_t = SteeringCmd()
            self.gear_msg_t = GearCmd()
            self.turn_signal_msg_t = TurnSignalCmd()
            self.cmd_vel_msg_t = Twist()
            self.time_step = 0.1


def get_config(is_default = True):
    if is_default == True:
        obj = Config()
    return obj