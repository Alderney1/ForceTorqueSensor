__author__ = "Mats Larsen"
__copyright__ = "SINTEF, NTNU 2012"
__credits__ = ["Morten Lind"]
__license__ = "GPL"
__maintainer__ = "Mats Larsen"
__email__ = "matsla@{ntnu.no}"
__status__ = "Development"
#--------------------------------------------------------------------
#File: test_ft_sensor.py
#Module Description
"""
This module test the ft_sensor.py
"""
#--------------------------------------------------------------------
#IMPORT
#--------------------------------------------------------------------
import traceback
import time
import numpy as np
from  ft_sensor import FT_Sensor as FT
#--------------------------------------------------------------------
#CONSTANTS
#--------------------------------------------------------------------
LOG_LEVEL = 2 # Information level
FILE = 'ft_sensor'
ATI_DEVICE = '192.168.0.74'
LOCAL_HOST = '127.0.0.1'
ATI_PORT = 49152
#--------------------------------------------------------------------
#METHODS
#--------------------------------------------------------------------
"""Print a message, and track, where the log is invoked
Input:
-msg: message to be printed, ''
-log_level: informationlevel"""
def log(msg, log_level=LOG_LEVEL):
    global LOG_LEVEL
    if log_level <= LOG_LEVEL:
        print(str(log_level) + ' : ' +  FILE + ' ::' +
              traceback.extract_stack()[-2][2] + ' : ' + msg)

sensor = FT(name='test')

log('Begin to start FT',2)
#sensor.set_mode(mode='LOGGING_DATA',number_samplings=10)
#sensor.wait_logging_mode()
#time.sleep(3)
sensor.set_mode(mode='UPDATE_BIAS')
sensor.wait_for_idle()
sensor.set_mode(mode='AVG_REALTIME')
for i in range(0,10):
    #print(sensor.get_force_contact(sync=True))
    pass
time.sleep(4)
"""

print('Current streaming '+ sensor.get_streaming())
print(sensor.get_running())


sensor.set_streaming('STOP')
print('Current streaming '+ sensor.get_streaming())
sensor.set_streaming('INFINITE')
old = 0
datalist = list()
for x in range (0,500):
    header, force, torque = sensor.get_ft()
    new = float((time.time() * 1000))
   # print('time = ' + str(( new - old)))
    datalist.append(new-old)
    old = new
data = np.array(datalist[4:])
print(data)
print(np.mean(data))
print(np.std(data))
print(force)
print(torque)

"""""
