#!/usr/bin/env python
import psutil


class GetUtility:
    def __init__(self):
        pass

    def get_cpu(self):
        return psutil.cpu_percent(interval=1)

    def get_memory(self):
        return psutil.virtual_memory().percent

    def get_temp(self):
        current_temps = psutil.sensors_temperatures()
        try:
            # temp = current_temps['current_temps'][0].current
            temp = current_temps['k10temp'][0].current
            return temp
        except ValueError, KeyError:
            return 0

    def get_data(self):
        return self.get_cpu(), self.get_memory(), self.get_temp()


