#! /usr/bin/env python


import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest


rospy.init_node('odom_mkz')

odom_pub = rospy.Publisher('/mkz_odom', Odometry, queue_size=10)

rospy.wait_for_service('/gazebo/get_model_state')

get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom = Odometry()

header = Header()

header.frame_id = '/mkz/base_footprint'

model = GetModelStateRequest()

model.model_name = 'mkz'

r = rospy.Rate(5)

while not rospy.is_shutdown():

    result = get_model_srv(model)

    odom.pose.pose = result.pose

    odom.twist.twist = result.twist

    header.stamp = rospy.Time.now()

    odom.header = header

    odom_pub.publish(odom)

    r.sleep()
