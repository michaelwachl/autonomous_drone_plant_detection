#!/usr/bin/env python
import socket
import struct
import sys

import rospy
from std_msgs.msg import Empty, UInt16, Bool
from environment_sensor.msg import SCD30

class SCD30Node(object):
    def __init__(self):
        # self.esp8266_ip = rospy.get_param('~esp_ip', '192.168.10.1')
        # self.esp8266_port = int(rospy.get_param('~esp_port', 880))

        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 8080))
        self.sock.settimeout(15.0)

        # Sensor topic
        self.pub_scd30 = rospy.Publisher(
            'scd30', SCD30, queue_size=10, latch=True)

        rospy.loginfo('SCD30 Node ready')


def main():
    rospy.init_node('scd30_node')
    sensor = SCD30Node()

    """received struct:
    float temperature
    float humidity
    uint16 co2
    """
    unpacker = struct.Struct('f f i')

    while True:
        print("####### Now listening #######")
        try:
            data = sensor.sock.recv(unpacker.size)
            # data = sensor.sock.recv(1024)
            #print('received "%s"' % binascii.hexlify(data))
            unpacked_data = unpacker.unpack(data)
            temperature = unpacked_data[0]      # in *C
            humidity = unpacked_data[1]         # in %
            co2 = unpacked_data[2]              # in PPM
            print('\n-----------------------------------------------\n')
            print('Temprature: ', temperature)
            print('\nHumidity: ', humidity)
            print('\nCO2: ', co2)

            msg = SCD30(
                    temperature=temperature,        # in *C
                    humidity=humidity,              # in %
                    co2=co2,)                        # in PPM

            # publish sensor data
            sensor.pub_scd30.publish(msg)
            rospy.loginfo('SCD30 data publised')

        except OSError as err:
            print("OS error: {0}".format(err))
        except ValueError:
            print("Could not convert data to an integer.")
        except:
            print("Unexpected error:", sys.exc_info()[0])
            raise
            # sensor.sock.close()

if __name__ == '__main__':
    main()