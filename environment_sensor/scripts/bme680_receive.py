#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty, UInt8, Bool
from environment_sensor.msg import BME680

import socket
import struct
import sys

FREQUENCY = 10

class BME680Node(object):
    def __init__(self):
        # self.esp8266_ip = rospy.get_param('~esp_ip', '192.168.10.1')
        # self.esp8266_port = int(rospy.get_param('~esp_port', 880))

        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 8080))
        self.sock.settimeout(10.0)

        # Sensor topic
        self.pub_bme680 = rospy.Publisher(
            'bme680', BME680, queue_size=10, latch=True)

        rospy.loginfo('BME680 Node ready')


def main():
    rospy.init_node('bme680_node')
    sensor = BME680Node()

    """received struct:
    float temperature
    float humidity
    float pressure
    float gas
    float altitude
    int iaq
    """
    unpacker = struct.Struct('f f f f f i')

    while True:
        print("####### Now listening #######")
        try:
            data = sensor.sock.recv(unpacker.size)
            # data = sensor.sock.recv(1024)
            #print('received "%s"' % binascii.hexlify(data))

            unpacked_data = unpacker.unpack(data)
            print('unpacked:', unpacked_data)

            msg = BME680(
                    temperature=unpacked_data[0],       # in *C
                    pressure=unpacked_data[1],          # in hPa
                    humidity=unpacked_data[2],          # in %
                    gas_resistance=unpacked_data[3],    # in kOhms
                    altitude=unpacked_data[4],          # Approx. Altitude in m
                    iaq = unpacked_data[5])             # Air quality score 0 - 100

            # publish sensor data
            sensor.pub_bme680.publish(msg)
            rospy.loginfo('BME680 publised')


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