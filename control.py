#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

freq=(2*np.pi)/30
l1,l2,l3=0,0,0
dt=0.1
x_e=[0]
y_e=[0]
yaw_e=[0]

rospy.init_node('publish_cmd')
publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

eta=np.array([0,0,0])
zeta=np.array([0,0,0])

for q in range(0,300,1):
    i=q/10.0 
    x_d=1*np.sin(freq*i)
    y_d=1*np.sin(2*freq*i)
    eta_d=np.array([x_d,y_d,0])
    x_d_dot=1*freq*np.cos(freq*i)
    y_d_dot=2*freq*np.cos(2*freq*i)
    eta_d_dot=np.array([x_d_dot,y_d_dot,0.0])
    err=np.array(eta_d-eta)
    x_e.append(err[0])
    y_e.append(err[1])
    yaw_e.append(err[2])
  
    j=np.array([[np.cos(eta[2]),-1*np.sin(eta[2]),0],[np.sin(eta[2]),np.cos(eta[2]),0,],[0,0,1]])
    zeta=np.matmul(np.linalg.inv(j),np.add(np.matmul(np.diag(np.array([l1,l2,l3])),err),eta_d_dot))
    eta=eta+np.matmul(j,zeta)*dt
    
    vel=Twist()
    vel.linear.x=zeta[0]
    vel.linear.y=zeta[1]
    vel.angular.z=zeta[2]
    rospy.loginfo(vel)
    publisher.publish(vel)
    

vel=Twist()
vel.linear.x=0
vel.linear.y=0
vel.angular.z=0
publisher.publish(vel)

fig, axs = plt.subplots(2, 2)
axs[0, 0].plot(x_e,color='red')
axs[0, 0].set_title('error x ')
axs[0, 1].plot(y_e,color='green')
axs[0, 1].set_title('error y')
axs[1, 0].plot(yaw_e)
axs[1, 0].set_title('error yaw')
plt.show()