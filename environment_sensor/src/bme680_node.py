#!/usr/bin/env python
import rospy
import os                                         
from std_msgs.msg import Empty, UInt8, Bool
from environment_sensor.msg import BME680

import math
import numpy as np
import threading
import socket
import time

FREQUENCY = 10

class TelloNode(object):
 
    def __init__(self):
        self.esp8266_ip = rospy.get_param('~esp_ip', '192.168.10.1')
        self.esp8266_port = int(rospy.get_param('~esp_port', 8889))

        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.port))
        self.sock.settimeout(2.0)

        try:
            self.wait_for_connection(timeout=self.connect_timeout_sec)
        except error.TelloError as err:
            rospy.logerr(str(err))
            rospy.signal_shutdown(str(err))
            self.quit()
            return

        # Setup topics and services
        # NOTE: ROS interface deliberately made to resemble bebop_autonomy
        self.pub_status = rospy.Publisher(
            'bme680', TelloStatus, queue_size=1, latch=True)

        rospy.loginfo('BME680 Node ready')        


    def cb_status_log(self, event, sender, data, **args):

        msg = BME680(
                    temperature=     # in *C
                    pressure=        # in hPa
                    humidity=        # in %
                    gas_resistance=      # in kOhms
                    altitude=       # Approx. Altitude in m
        )
        self.pub_bme680.publish(msg)



def main():
    rospy.init_node('bme680_node')
    sensor = BME680Node()
    rospy.Timer(rospy.Duration(1.0/FREQUENCY), controller.iteration)
    rospy.spin()                  # keep process alive


if __name__ == '__main__':
    main()