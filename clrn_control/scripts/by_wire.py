#! /usr/bin/env python

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dbw_mkz_msgs.msg import ThrottleCmd, BrakeCmd, SteeringCmd, GearCmd, TurnSignalCmd


class Vehicle_NS:
    def __init__(self, vehicle):
        self.set_vehicle_namespace(vehicle)

    def set_vehicle_namespace(self, vehicle):
        if vehicle == "mkz":
            self.trkr_out_llc_in_node = 'mkz_llc_node'
            self.trkr_out_llc_in_topic =  '/mkz/ptracker'
            self.feedback_out_llc_in_topic = '/mkz/odom'
            self.thrtl_topic = '/mkz/throttle_cmd'
            self.brake_topic = '/mkz/brake_cmd'
            self.steer_topic = '/mkz/steering_cmd'
            self.gear_topic = '/mkz/gear_cmd'
            self.twist_topic = 'mkz/cmd_vel'
            self.turn_signal_topic = '/mkz/turn_signal_cmd'
            self.trkr_msg_t = Float64MultiArray
            self.feedback_msg_t = Odometry
            self.thrtl_msg_t = ThrottleCmd
            self.brake_msg_t = BrakeCmd
            self.steer_msg_t = SteeringCmd
            self.gear_msg_t = GearCmd
            self.twist_msg_t = Twist
            self.turn_signal_msg_t = TurnSignalCmd


        elif vehicle == "fusion":
            self.trkr_out_llc_in_node = 'fusion_llc_node'
            self.trkr_out_llc_in_topic =  '/fusion/ptracker'
            self.thrtl_topic = '/fusion/throttle_cmd'
            self.brake_topic = '/fusion/brake_cmd'
            self.steer_topic = '/fusion/steering_cmd'
            self.gear_topic = '/fusion/gear_cmd'
            self.twist_topic = 'fusion/cmd_vel'
            self.turn_signal_topic = '/fusion/turn_signal_cmd'
            self.thrtl_msg_t = ThrottleCmd
            self.brake_msg_t = BrakeCmd
            self.steer_msg_t = SteeringCmd
            self.gear_msg_t = GearCmd
            self.twist_msg_t = Twist
            self.turn_signal_msg_t = TurnSignalCmd


def trkr_out_llc_in_callback(data):



def main():
    try:
        ns_obj = Vehicle_NS("mkz")
        rospy.init_node(ns_obj.trkr_out_llc_in_node, anonymous=True)
        thrtl_pub = rospy.Publisher(ns_obj.thrtl_topic, ns_obj.thrtl_msg_t, queue_size=10)
        brake_pub = rospy.Publisher(ns_obj.brake_topic, ns_obj.brake_msg_t, queue_size=10)
        steer_pub = rospy.Publisher(ns_obj.steer_topic, ns_obj.steer_msg_t, queue_size=10)
        gear_pub = rospy.Publisher(ns_obj.gear_topic, ns_obj.gear_msg_t, queue_size=10)
        twist_pub = rospy.Publisher(ns_obj.twist_topic, ns_obj.twist_msg_t, queue_size=10)
        twist_init_msg = self.twist_msg_t
        twist_init_msg.linear.x = 0.0
        twist_init_msg.linear.y = 0.0
        twist_init_msg.linear.z = 0.0
        twist_init_msg.angular.x = 0.0
        twist_init_msg.angular.y = 0.0
        twist_init_msg.angular.z = 0.0
        rospy.Subscriber(ns_obj.trkr_out_llc_in_topic, ns_obj.trkr_msg_t, trkr_out_llc_in_callback)
        rospy.Subscriber(ns_obj.feedback_out_llc_in_topic, ns_obj.feedback_msg_t, feedback_out_llc_in_callback)
        twist_pub.pub(twist_init_msg)
        rospy.spin()
    except:
        print "Traker IN LLC OUT node not running!!"


if __name__ == '__main__':
    main()