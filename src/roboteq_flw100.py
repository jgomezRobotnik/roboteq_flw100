#!/usr/bin/python
#coding=utf-8

# Node that publishes the data of the sensor RoboteQ FLW100

import rospy
import serial
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Point
from geometry_msgs.msg import TwistWithCovariance, Vector3

ser = serial.Serial('/dev/RoboteQ-FLW100')

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

def main():
    pub = rospy.Publisher('flw100/raw_odom', Odometry, queue_size = 1)
    data = Odometry()
    
    # Init node
    rospy.init_node('raw_flw100')
    rate = rospy.Rate(50) # 50hz

    ser.baudrate = 115200

    # Cogemos el offset de x
    ser.write('?MM 1\r\n')
    x_offset = getSerialData()
    while not is_digit(x_offset[9:]):
        ser.write('?MM 1\r\n')
        x_offset = getSerialData()
    x_offset = int(x_offset[9:])
    # Cogemos el offset de y 
    ser.write('?MM 2\r\n')
    y_offset = getSerialData()
    while not is_digit(y_offset[9:]):
        ser.write('?MM 2\r\n')
        y_offset = getSerialData()
    y_offset = int(y_offset[9:])

    
    while not rospy.is_shutdown():
        try:
            # Leemos x
            ser.write('?MM 1\r\n')
            x = getSerialData()
            while not is_digit(x[9:]):
                ser.write('?MM 1\r\n')
                x = getSerialData()
            x = (int(x[9:]) - x_offset) / 10000.0 # In m
            # Leemos y 
            ser.write('?MM 2\r\n')
            y = getSerialData()
            while not is_digit(y[9:]):
                ser.write('?MM 2\r\n')
                y = getSerialData()
            y = (int(y[9:]) - y_offset) / 10000.0 # In m
            # Leemos velocidad x
            ser.write('?SMM 1\r\n')
            x_vel = getSerialData()
            while not is_digit(x_vel[11:]):
                ser.write('?SMM 1\r\n')
                x_vel = getSerialData()
            x_vel = int(x_vel[11:]) / 10000.0 # In m/s
            # Leemos velocidad y
            ser.write('?SMM 2\r\n')
            y_vel = getSerialData()
            while not is_digit(y_vel[11:]):
                ser.write('?SMM 2\r\n')
                y_vel = getSerialData()
            y_vel = int(y_vel[11:]) / 10000.0 # In m/s
            # Leemos calidad de imagen
            # -> Laser debería ser > 1800
            # -> LED debería ser > 1300
            '''
            ser.write('?IMQ\r\n')
            calidad = getSerialData()
            while not is_digit(calidad[9:]):
                ser.write('?IMQ\r\n')
                calidad = getSerialData()
            calidad = int(calidad[9:])
            
            # Imprimimos los valores
            print 'X:     %-10s Y:     %-10s\n' % (x,-y)
            print 'X_vel: %-10s Y_vel: %-10s\n' % (x_vel,-y_vel)
            print 'Calidad: '+str(calidad)+' (debería ser > 1800)'
            print '-----------------------------------'
            '''
            # Publicamos datos
            data.pose.pose.position = Point(x, -y, 0)
            data.twist.twist.linear = Vector3(x_vel, -y_vel, 0)
            pub.publish(data)
            rate.sleep()
        except KeyboardInterrupt:
            exit()

    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass