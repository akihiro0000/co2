#!/usr/bin/env python

import os
import sys
from time import sleep

from CCS811 import CCS811

class AirConditionMonitor:
    CO2_PPM_THRESHOLD_1 = 1000
    CO2_PPM_THRESHOLD_2 = 2000

    CO2_LOWER_LIMIT  =  400
    CO2_HIGHER_LIMIT = 8192

    CO2_STATUS_CONDITIONING = 'CONDITIONING'
    CO2_STATUS_LOW          = 'LOW'
    CO2_STATUS_HIGH         = 'HIGH'
    CO2_STATUS_TOO_HIGH     = 'TOO HIGH'
    CO2_STATUS_ERROR        = 'ERROR'


    def __init__(self):
        self._ccs811 = CCS811()
        self.co2_status = self.CO2_STATUS_LOW

    def status(self, co2):
        if co2 < self.CO2_LOWER_LIMIT or co2 > self.CO2_HIGHER_LIMIT:
            return self.CO2_STATUS_CONDITIONING
        elif co2 < self.CO2_PPM_THRESHOLD_1:
            return self.CO2_STATUS_LOW
        elif co2 < self.CO2_PPM_THRESHOLD_2:
            return self.CO2_STATUS_HIGH
        else:
            return self.CO2_STATUS_TOO_HIGH

    def execute(self):
        while not self._ccs811.available():
            pass

        while True:
            if not self._ccs811.available():
                sleep(1)
                continue

              if not self._ccs811.readData():
                  co2 = self._ccs811.geteCO2()
                  co2_status = self.status(co2)
                  co2 = '"' + "CO2[ppm]" + '"' + ":" + '"' + str(co2) + '"'

                  if co2_status != self.co2_status:
                      self.co2_status = co2_status
                  return co2
              else:
                   pass


if __name__ == '__main__':
    air_condition_monitor = AirConditionMonitor()
    air_condition_monitor.execute()
