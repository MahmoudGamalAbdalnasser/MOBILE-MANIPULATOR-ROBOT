#! /usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import select,sys

freq=(2*np.pi)/60
l1, l2, l3 = 0, 0, 0
dt = 0.06
x_e = [0]
y_e = [0]
yaw_e = [0]
x = 0
y = 0
yaw = 0

def odom_callback(msg):
    global x, y, yaw
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

rospy.init_node('publish_cmd')
publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.Subscriber('/odom', Odometry, odom_callback)

eta = np.array([0, 0, 0])
zeta = np.array([0, 0, 0])

sequence = np.arange(0, 60.06, dt)

for i in sequence:
    x_d = 1 * np.sin(freq * i)
    y_d = 1 * np.sin(2 * freq * i)
    eta_d = np.array([x_d, y_d, 0])
    x_d_dot = 1 * freq * np.cos(freq * i)
    y_d_dot = 2 * freq * np.cos(2 * freq * i)
    eta_d_dot = np.array([x_d_dot, y_d_dot, 0.0])
    eta=[x,y,yaw]
    err = np.array(eta_d - eta)
    
    j = np.array([[np.cos(eta[2]), -1 * np.sin(eta[2]), 0], [np.sin(eta[2]), np.cos(eta[2]), 0], [0, 0, 1]])
    zeta = np.matmul(np.linalg.inv(j), np.add(np.matmul(np.diag(np.array([l1, l2, l3])), err), eta_d_dot))

    vel = Twist()
    vel.linear.x = zeta[0]
    vel.linear.y = zeta[1]
    vel.angular.z = 0
    rospy.loginfo(vel)
    publisher.publish(vel)
    rospy.sleep(.03)

    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        vel = Twist()
        vel.linear.x = 0
        vel.linear.y = 0
        vel.angular.z = 0
        publisher.publish(vel)
        break

vel = Twist()
vel.linear.x = 0
vel.linear.y = 0
vel.angular.z = 0
publisher.publish(vel)
rospy.spin()
