#!/usr/bin/env python

"""
pip install rssi
also on local
"""
import rssi

class WifiInfo:
    def __init__(self, interface, ssid):
        self.interface = interface
        self.ssid = ssid
        self.rssi_scanner = rssi.RSSI_Scan(interface)
        print("init rssi scanner")

    def get_info(self):
        ap_info = self.rssi_scanner.getAPinfo(networks=self.ssid)
        if type(ap_info) == bool:
            return -90, 0
        try:
            if len(ap_info) > 0:
                signal = ap_info[0]['signal']
                quality = ap_info[0]['quality']
                return signal, quality
            else:
                # return bad signal and 0 quality
                return -90, 0
        except TypeError:
            print('Not connected to drone Wi-Fi')


def main():
    """Pretty prints the output of iwlist scan into a table"""
    interface = "wlp2s0"
    ssid_names = "TELLO-FD143A"

    infow = WifiInfo(interface, ssid_names)
    (sig, qual) = infow.get_info()
    print(sig, qual)


if __name__ == '__main__':
    main()
