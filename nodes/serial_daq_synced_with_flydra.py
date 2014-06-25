#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_flydra')
import rospy
from ros_flydra.msg import *
from ros_flydra.srv import *
import time

import serial

class Flydra_DAQ:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, timeout=1, parity='N', stopbits=2):
        rospy.wait_for_service("flydra_super_packet_service")
        self.get_latest_flydra_data = rospy.ServiceProxy("flydra_super_packet_service", super_packet_service)
        self.serial_connection = serial.Serial(port, baudrate, timeout, parity, stopbits)
        
        

    def collect_data(self, n_seconds_to_average=1, serial_parser=None):
        positions = []
        serial_values = []
        self.serial_connection.flush()
        
        t_start = time.time()
        while time.time() - t_start < n_seconds_to_average:
            superpacket = self.get_latest_flydra_data().packets
            
            if len(superpacket.packets) == 1:
                packet = superpacket.packets[0]
                if len(packet.objects) == 1:
                    obj = packet.objects[0]
                    pos = [obj.position.x, obj.position.y, obj.position.z]
                    positions.append(pos)
                    
                    serial_values = []
                    lines = self.serial_connection.readlines()
                    for line in lines:
                        serial_values.append(serial_parser(line))
        
        positions = np.array(positions)
        mean_position = np.mean(positions, axis=0)
                        
        serial_values = np.array(serial_values)
        mean_serial_values = np.mean(serial_values, axis=0)
        
        data = {'position': mean_position, 'daq': mean_serial_values}
        
        self.serial_connection.close()
        
        return data
                        
                
def serial_line_parsing_function(line):
    s = line.split('  ')
    val = float(s[3])
    return [val]
    
def write_data_to_file(data, filename):
    f = open(filename)
    pickle.dump(data, filename)
    f.close()

if __name__ == '__main__':
    flydra_daq = Flydra_DAQ()
    data = flydra_daq.collect_data(n_seconds_to_average=1, serial_parser=serial_line_parsing_function)
    filename = time.strftime("flydra_daq_%Y%m%d_%H%M%S",time.localtime())
    write_data_to_file(data, filename)
    
    
    
