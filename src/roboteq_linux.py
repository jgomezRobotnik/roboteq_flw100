#!/usr/bin/python
#coding=utf-8
import serial
import time

ser = serial.Serial('/dev/RoboteQ-FLW100')

def getSerialData():
    info = str(ser.read())
    while ser.inWaiting()>0:
        info += str(ser.read())
    return info

def main():
    ser.baudrate = 115200

    # Cogemos el offset de x
    ser.write('?MM 1\r\n')
    x_offset = getSerialData()
    x_offset = int(x_offset[9:])
    # Cogemos el offset de y 
    ser.write('?MM 2\r\n')
    y_offset = getSerialData()
    y_offset = int(y_offset[9:])

    
    while(True):
        try:
            # Leemos x
            ser.write('?MM 1\r\n')
            x = getSerialData()
            x = (int(x[9:]) - x_offset) / 10000.0 # In m
            # Leemos y 
            ser.write('?MM 2\r\n')
            y = getSerialData()
            y = (int(y[9:]) - y_offset) / 10000.0 # In m
            # Leemos velocidad x
            ser.write('?SMM 1\r\n')
            x_vel = getSerialData()
            x_vel = int(x_vel[11:]) / 10000.0 # In m/s
            # Leemos velocidad y
            ser.write('?SMM 2\r\n')
            y_vel = getSerialData()
            y_vel = int(y_vel[11:]) / 10000.0 # In m/s
            # Leemos calidad de imagen
            # -> Laser debería ser > 1800
            # -> LED debería ser > 1300
            ser.write('?IMQ\r\n')
            calidad = getSerialData()
            calidad = int(calidad[9:])

            # Imprimimos los valores
            print 'X:     %-10s Y:     %-10s\n' % (x,-y)
            print 'X_vel: %-10s Y_vel: %-10s\n' % (x_vel,-y_vel)
            print 'Calidad: '+str(calidad)+' (debería ser > 1800)'
            print '-----------------------------------'
        except KeyboardInterrupt:
            exit()
        except:
            print("Fallo en la comunicación.\n")

    
if __name__ == "__main__":
    main()