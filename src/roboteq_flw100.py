#!/usr/bin/python
#coding=utf-8

# Node that publishes the data of the sensor RoboteQ FLW100

import math
from math import sin, cos, pi

import rospy
import serial
import time
import tf
import re
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion
from geometry_msgs.msg import TwistWithCovariance, Twist, Vector3

# Init node

rospy.init_node('roboteq_flw100')

# Declaration and initialization of variables

ser = serial.Serial('/dev/RoboteQ-FLW100')
ser.baudrate = 115200
rate = rospy.Rate(5) # 50hz
odom_pub = rospy.Publisher('flw100/odom', Odometry, queue_size = 10)
odom_broadcaster = tf.TransformBroadcaster()

# Initial positions (depending on odom)
x = 0.26 # (In front of the RB1)
y = 0.0
# Angle on the z-axis (depending on odom)
th = 0.0
# Angle of all axis (quaternion)
getted_odom_quat = [0,0,0,1] # From sensor
odom_quat = [0,0,0,1] # From robot IMU

# Methods

def getSerialData():
    info = str(ser.read())
    while ser.inWaiting()>0:
        info += str(ser.read())
    return info

def is_digit(n):
    try:
        int(n)
        return True
    except ValueError:
        return False

def callback_orientation(msg):
    global odom_quat, th
    # Getting orientation from robot
    odom_quat = [msg.pose.pose.orientation.x,
                 msg.pose.pose.orientation.y,
                 msg.pose.pose.orientation.z,
                 msg.pose.pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(odom_quat)
    th = euler[2]

def main():

    rospy.Subscriber('/rb1_base_a/robotnik_base_control/odom', Odometry, callback_orientation)

    global x, y, th, getted_odom_quat, odom_quat

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    
    while not rospy.is_shutdown():

        current_time = rospy.Time.now()
        
        # Reading velocity in x

        ser.write('?SMM 1\r\n')
        getted_vx = getSerialData()
        while not is_digit(getted_vx[11:]):
            ser.write('?SMM 1\r\n')
            getted_vx = getSerialData()
        getted_vx = -int(getted_vx[11:]) / 10000.0 # In m/s

        # Reading velocity in y

        ser.write('?SMM 2\r\n')
        getted_vy = getSerialData()
        while not is_digit(getted_vy[11:]):
            ser.write('?SMM 2\r\n')
            getted_vy = getSerialData()
        getted_vy = int(getted_vy[11:]) / 10000.0 # In m/s

        # Reading angular velocity (AHRS)
        
        ser.write('?QO uu 0\r\n')
        getted_gyro = getSerialData()
        getted_gyro = getted_gyro.replace('=',':').split(":")
        if ( len(getted_gyro) == 5 and
             is_digit(getted_gyro[1]) and
             is_digit(getted_gyro[2]) and
             is_digit(getted_gyro[3]) and
             is_digit(getted_gyro[4])):
            getted_odom_quat[0] = int(getted_gyro[1]) / 1000.0
            getted_odom_quat[1] = int(getted_gyro[2]) / 1000.0
            getted_odom_quat[2] = int(getted_gyro[3]) / 1000.0
            getted_odom_quat[3] = int(getted_gyro[4]) / 1000.0

        # Just for debugging (commented):

        '''
        # Leemos calidad de imagen
        # -> Laser debería ser > 1800
        # -> LED debería ser > 1300
        
        ser.write('?IMQ\r\n')
        getted_calidad = getSerialData()
        while not is_digit(getted_calidad[9:]):
            ser.write('?IMQ\r\n')
            getted_calidad = getSerialData()
        getted_calidad = int(getted_calidad[9:])
        
        # Imprimimos los valores
        print 'X:     %-10s Y:     %-10s\n' % (getted_x,getted_y)
        print 'Vel X: %-10s Vel Y: %-10s\n' % (getted_vx,getted_vy)
        print 'Calidad: '+str(getted_calidad)+' (debería ser > 1800)'
        print '-----------------------------------'
        '''

        # compute odometry in a typical way given the velocities of the robot
        dt = (current_time - last_time).to_sec()
        delta_x = (getted_vx * cos(th) - getted_vy * sin(th)) * dt
        delta_y = (getted_vx * sin(th) + getted_vy * cos(th)) * dt

        x += delta_x
        y += delta_y

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        #odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (x, y, 0.0),
            odom_quat,
            current_time,
            "odom_flw100", # Link que se moverá en función al otro link
            "rb1_base_a_odom" # Link fijo de referencia
        )

        # next, we'll publish the odometry message over ROS
        data = Odometry()

        # set the position
        data.pose.pose.position = Point(x, y, 0)
        
        # set the orientation (if we want, we can use getted_odom_quat)
        data.pose.pose.orientation.x = odom_quat[0]
        data.pose.pose.orientation.y = odom_quat[1]
        data.pose.pose.orientation.z = odom_quat[2]
        data.pose.pose.orientation.w = odom_quat[3]
        
        # set the velocity
        data.child_frame_id = "base_link"
        data.twist.twist = Twist(Vector3(getted_vx, getted_vy, 0), Vector3(0,0,0))

        # publish the message
        odom_pub.publish(data)

        last_time = current_time
        rate.sleep()

    
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        exit()
    except rospy.ROSInterruptException:
        pass