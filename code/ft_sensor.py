__author__ = "Mats Larsen"
__copyright__ = "SINTEF, NTNU 2013"
__credits__ = ["Morten Lind"]
__license__ = "GPL"
__maintainer__ = "Mats Larsen"
__email__ = "matsla@{ntnu.no}"
__status__ = "Development"
#--------------------------------------------------------------------
#File: ft_sensor.py
#Module Description
"""
This module handle the force/torque sensor independet of the connector.
Choose of typeconnector and streamming type and sampling periode
"""
#--------------------------------------------------------------------
#IMPORT
#--------------------------------------------------------------------
import traceback
import time
import threading
import numpy as np
from timermanager import TimerManager as TM
from ati_interface import ATI_INTERFACE
from bias_handler import BiasHandler
import math3d as m3d
from math3d.vector import Vector
from math3d.transform import Transform
from math3d.dynamics.wrench import OrigoWrench
from chain import Chain
from pymoco.controller_manager import ControllerManager as CM

#--------------------------------------------------------------------
#CONSTANTS
#--------------------------------------------------------------------
LOG_LEVEL = 2 # Information level
ALWAYS_LOG_LEVEL = 1 # print allways
FILE_NAME = 'ft_sensor'
NUMBER_SIGNALS = 6 # number of signals
ATI_TIMEOUT = 1.5 # timeout to wait for ati_interface
CONTROLLER_PROCESSING_TIME = 0.3 #  the time as pymoco takes to handle a control
MODES = {'STOP' : '_stop_mode',
        'UPDATE_BIAS' : '_update_bias',
        'AVG_REALTIME' : '_averaging_realtime_mode',
        'REALTIME' : '_realtime_mode',
        'INFINITE_REALTIME' : '_infinite_realtime_mode',
        'REALTIME_BUFFERED' : '_realtime_buffered_mode',
        'LOGGING_DATA' : '_logging_data_mode',
        'INFINITE_REALTIME_&_LOGGING_DATA' : '_infinite_realtime_logging_data_mode',
        }
#--------------------------------------------------------------------
#METHODS
#--------------------------------------------------------------------
def log(msg, log_level=LOG_LEVEL):
    """ Print a message, and track, where the log is invoked
    Input:
    -msg: message to be printed, ''
    -log_level: informationlevel"""
    global LOG_LEVEL
    if log_level <= LOG_LEVEL:
        print(str(log_level) + ' : ' + FILE_NAME + '.py::' +
              traceback.extract_stack()[-2][2] + ' : ' + msg)

class FT_Sensor(threading.Thread):
    """Class for ft-sensor."""
    class Error(Exception):
        """Exception class."""
        def __init__(self, message):
            self.message = message
            Exception.__init__(self, self.message)
        def __repr__(self):
            return self.message
    def __init__(self,name='1', # name of the instance
                 host='127.0.0.1', # host of the ft-sensor
                 port=49152, # port to the ft-sensor
                 mode='STOP', # streaming mode
                 typeConnector='ATI', # type of connector
                 N = 6, # number of noise periods
                 buffersize=10, #size of the buffer
                 bias_type='static',
                 time_store_data = 4, # seconds for storing data
                 collision_transform=Transform(),
                 no_data_timeout = 1.5, # timeout for the sensor to wait for data
                 force_threshold=2, # to detect collision by a threshold
                 sensor_timeout=0.2,
                 cm=None, # pymoco controller
                 log_level=3): # information level
        """Init will set the streaming to 'STOP', Has to enabled afterwards.
        Initilize connector here."""
        #assignment
        self._name = 'ft_sensor#' + name # name of the instance
        self._host = host
        self._port = port
        self._mode = mode # streaming type
        self._log_level = log_level # information level
        self._typeConnector = typeConnector # type of connector(ATI)
        self._N = N # the noise periods
        self._buffersize = buffersize
        self._type = bias_type # which type of bias handler mode
        self._time_logging = time_store_data
        self._collision_transform = collision_transform
        self._no_data_timeout = no_data_timeout
        self._sensor_timeout = sensor_timeout
        self._force_threshold = force_threshold
        self._cm = cm
        #Event
        self._contact_translation = threading.Condition() # trigger event for contact change
        self._contact_status = threading.Condition() # status on contact
        self._ft_status = threading.Condition() # trigger to incidate new data for ft data

        self._event_bias = threading.Event() # trigger to update bias
        self._event_bias.clear()
        self._thread_alive = threading.Event() # status for the thread
        self._thread_alive.clear()
        self._thread_terminated = threading.Event() # status for the thread
        self._thread_terminated.clear()
        self._mode_trigger = threading.Event() # when new mode is selected
        self._mode_trigger.clear()
        self._logging_terminated = threading.Event() # trigger to indicate when logging mode is complete
        self._logging_terminated.clear()

        self._wait_stop_mode = threading.Event() # status for stop mode
        self._wait_updatebias_mode = threading.Event() # status for stop mode
        self._wait_avg_mode = threading.Event() # status for stop mode
        self._wait_buffered_mode = threading.Event() # status for buffered mode
        self._wait_logging_data_mode = threading.Event() # status for logdata mode
        self._operational_mode = 'STOP'
        self._old_operational_mode = None
        #Reset
        self._wait_stop_mode.clear()
        self._wait_updatebias_mode.clear()
        self._wait_avg_mode.clear()
        self._wait_buffered_mode.clear()
        self._wait_logging_data_mode.clear()
        self._current_mode = 'STOP'
        self._previous_contact = None
        self._ft = None # force and torque
        self._force = None # force
        self._torque = None # torque
        self._wrench = OrigoWrench() # initlize wrench

        #Initialize the the connector
        if self._typeConnector == 'ATI':
            self._ft = ATI_INTERFACE(host=self._host,
                           port=self._port,
                           name=name,
                           timestamps=None,
                           timestamps_reciver=None,
                           headerstamps=False,
                           socket_timeout=3.5,
                           timeout=self._sensor_timeout,
                           buffersize=self._buffersize,
                           log_level=self._log_level)
            log('ATI INTERFACE ' + self._ft.name + ' is initialized.', ALWAYS_LOG_LEVEL)
        else:
            raise Exception('A unknown TypeConnector is choosen ')
        # Bias Handler
        self._bh = BiasHandler(log_level=self._log_level)
        log('BIAS_HANDLER ' + self._bh.name + ' is initialized.', ALWAYS_LOG_LEVEL)
        #The sensor should not start with activ streamming
        if self._mode == 'STOP':
            self._ft.set_streaming(streaming=self._mode)
        else:
            raise Exception('A unknown StartUp mode')
        #Thread settings
        threading.Thread.__init__(self) # initialize this thread
        self.daemon = True # contine if main loop stops
        self.start() # start up this instance
        log('FT-sensor ' + self._name + ' is created', self._log_level)

    def wait_for_control(self):
        """Put the process to wait until the controller has send data to
        the robot system."""
        log('Performing wait for control...', self._log_level)
        self._cm.robot_facade.wait_for_control(timeout=None)

    def get_bias_force(self):
        """Returning bias force."""
        return self._bh.get_bias_force()

    def get_bias_torque(self):
        """Returning bias torque."""
        return self._bh.get_bias_torque()


    def get_ft(self, sync=None):
        """ Get the status of the collision. True = collision.
            False = no collision"""
        log('Performing Get ft' , self._log_level)
        if sync != 0:
            self._ft_status.acquire()
            if self._ft_status.wait(sync) == True:
                self._ft_status.release()
                return self._force_data, self._torque_data
            else:
                return None,None

        else:
            log('Not SYNC', self._log_level)
            return self._force_data, self._torque_data
    ft = property(get_ft, "Contact property.")

    def get_mode(self):
        """This property returning the streaming setting"""
        return self._streaming
    mode = property(get_mode, "Streaming Property")

    def set_mode(self,mode='REALTIME',number_samplings=1):
        log('Set streaming is set to ' + mode, self._log_level)
        """ Property setting the streaming to one of the 5 modes of the
        ft-sensor."""
        if self._mode != mode:
            if self._ft != None: # A sensor has to exist
                self._mode = mode
                self._number_samplings = number_samplings
                self._mode_trigger.set() # trigger mode
            else:
                raise Exception(self._name + ': ' + 'Streaming setup for ATI is ' + 'correct or not known')

    def _break_mode(self):
        log('BreakMode', self._log_level)
        """ Property setting the streaming to one of the 5 modes of the
        ft-sensor."""

        self._ft.set_streaming(streaming='STOP',samples=0) # set to buffer mode
        self._current_mode = 'BREAK'
        #self._mode_trigger.set() # trigger mode


    def set_bias(self):
        """Update the bias, my setting the trigger event."""
        self._event_bias.set()


    def get_bias(self):
        """Returning the bias."""
        return self._ft.get_bias()
    bias = property(set_bias,'Bias Property.')

    def investigate_signal(self):
        """ Investigate the signal, to find the noise band and peak to
        peak and freqency of the noise. It will be run over N number
        of noise periods. """
        log('Performing Investigate signal',ALWAYS_LOG_LEVEL)
        self._ft.set_streaming(streaming='REALTIME_BUFFERED',samples=0) # set to buffer mode
        signal = Chain(name='force_torque') # list with all 6 signals
        for i in range(0,NUMBER_SIGNALS): # make 6 chain for f and t
            signal.add_tail(Chain(name='Signals_'+str(i)))
        for i in range (self._N): # loop all periods
            data = self._ft.get_data_ATI(sync=True,timeout=None,chain=True) # get data from ati-box
            for m in range(0,NUMBER_SIGNALS):
                signal.get_data(m).add_chain(other=data.get_data(m+2)) # combine data
        self._ft.set_streaming(streaming='STOP') # stop the streaming
        self._bh.investigate_signal(s=signal, fs=1000) # call biashandler to analyse signal

    def get_connector(self):
        """Return the connector."""
        return self._ft

    def get_name(self):
        """Return name of this."""
        return self._name
    name = property(get_name, "Name Property")

    def set_collistion_transform(self, new_transform):
        self._collision_transform = new_transform

    def get_contact_translation(self, sync = False):
        """ Get the status of the collision. True = collision.
            False = no collision"""
        log('Performing Get contact' , self._log_level)
        if sync == True:
            self._contact_translation.acquire()
            self._contact_translation.wait()
            self._contact_translation.release()
            return self._contact
        else:
            return self._contact
    translation = property(get_contact_translation, "Contact property.")

    def get_contact(self, sync=None):
        """ Get the status of the collision. True = collision.
            False = no collision"""
        log('Performing Get contact' , self._log_level)
        if sync != 0:
            self._contact_status.acquire()
            if self._contact_status.wait(sync) == True:
                self._contact_status.release()
                log(str(self._force_contact), self._log_level)
                return self._force_contact
            else:
                return None

        else:
            log('Not SYNC', self._log_level)
            return self._force_contact
    contact = property(get_contact, "Contact property.")

    def wait_logging_mode(self):
        """Wait until logging mode is terminated."""
        self._logging_terminated.wait() # wait until logging mode ended
        self._logging_terminated.clear()

    def wait_for_idle(self, mode,timeout=1):
        log('Performing wait for ilde, mode : ' + str(mode),self._log_level)
        """Wait for a specific mode."""
        if mode == 'UPDATE_BIAS':
            e = self._wait_updatebias_mode.wait(timeout)
        elif mode == 'STOP':
            e = self._wait_stop_mode.wait(timeout)
        elif mode == 'AVG_REALTIME':
            e = self._wait_avg_mode.wait(timeout)
        elif mode == 'REALTIME_BUFFERED':
            e = self._wait_buffered_mode.wait(timeout)
        elif mode == 'LOGGING_DATA':
            e = self._wait_logging_data_mode.wait(timeout)
        else:
            raise self.Error('Given mode is not known : ' + '"{}"'.format(mode))

        if e == True:
            log(mode + ' is in process :'  + '"{}"'.format(mode), ALWAYS_LOG_LEVEL)
        else:
            raise self.Error(mode + ' is not swicted in the given time :: '
                             + '"{}"'.format(timeout))

    def _infinite_realtime_mode(self):
        """Infinite realtime mode for ft-sensor."""
        log('Performing INFINITE REALTIME_MODE',self._log_level)
        if self._current_mode != self._mode: # when current mode is different from set mode
            self._ft.set_streaming(streaming='REALTIME',samples=0)
            self._current_mode = self._mode # when current mode is different from set mode
        h,i,fx,fy,fz,tx,ty,tz = self._ft.get_data_ATI(sync=True,timeout=ATI_TIMEOUT) # get data from atibox
        if h == None:
            raise self.Error('No response from the sensor, timeout : ' + '"{}"'.format(str(self._no_data_timeout)))
        offset = self._bh.offset # get offset
        self._force = Vector(fx.tail - offset.get_data(0),fy.tail - offset.get_data(1),fz.tail - offset.get_data(2)) # create a vector of the force
        self._wrench.set_force(new_force=self._force.copy())
        self._torque = Vector(tx.tail - offset.get_data(3) ,ty.tail - offset.get_data(4),tz.tail - offset.get_data(5)) # create a Vector of the torque
        self._wrench.set_moment(new_moment=self._torque.copy())
        self._wrench.equivalent(ref=self._collision_transform)
        log('Offset',self._log_level)
        #print(offset)
        log('Threshold',self._log_level)
        #print(self._force_threshold)
        log('Force',self._log_level)
        #print(self._force)
        #print(self._wrench)

        #force contact
        if self._wrench.force.length > self._force_threshold:
            self._force_contact = True # Contact
        else:
            self._force_contact = False # No contact
        #torque contact
        if self._wrench.moment.length > self._torque_threshold:
            self._torque_contact = True # Contact
        else:
            self._torque_contact = False # No Contact
        self._notify() # Notify to other threads

    def _realtime_mode(self):
        """realtime mode for the ft-sensor."""
        log('Performing REALTIME_MODE',self._log_level)
        h,i,fx,fy,fz,tx,ty,tz = self._ft.get_data_ATI(sync=True,timeout=ATI_TIMEOUT) # get data from atibox
        self._force = Vector(fx.get_data(fx),fy.get_data(fy),fz.get_data(fz)) # create a vector of the force
        self._torque = Vector(tx.get_data(tx),ty.get_data(ty),tz.get_data(tz)) # create a Vector of the torque
        #force contact
        if self._force.length > force_threshold:
            self._force_contact = True # Contact
        else:
            self._force_contact = False # No contact
        #torque contact
        if self._torque.length > torque_threshold:
            self._torque_contact = True # Contact
        else:
            self._torque_contact = False # No Contact
        self._notify() # Notify to other threads

    def _realtime_buffered_mode(self):
        """Realtime buffered for the ft sensor."""
        log('Performing realtime buffered mode.',ALWAYS_LOG_LEVEL)
        if self._wait_buffered_mode.isSet() == False:
            if self._current_mode != self._mode: # when current mode is different from set mode
                self._ft.set_streaming(streaming='REALTIME_BUFFERED',samples=self._number_samplings)
                self._current_mode = self._mode # when current mode is different from set mode
                self._ft.wait_for_mode(mode='REALTIME_BUFFERED',timeout=ATI_TIMEOUT)
                f_group,t_group = self._ft.get_data_ATI(sync=False,data_type='force_torque') # get data from atibox
            for i in range(1,self._number_samplings):
                print('----------------------------------------')
                f,t = self._ft.get_data_ATI(sync=True,timeout=self._no_data_timeout,data_type='force_torque') # get data from atibox
                if f == None:
                    self._force_contact = None
                    self._error()
                    raise self.Error('No response from the sensor, timeout, after ' + str(i) + ' samplings  : ' + '"{}"'.format(str(self._no_data_timeout)))
                f_group = np.append(f_group,f,axis=0)
                t_group = np.append(t_group,t,axis=0)
            bias_f = self._bh.force_bias
            bias_t = self._bh.torque_bias
            f_group = f_group - bias_f
            t_group = t_group - bias_t
            self._operational_mode = 'REALTIME_BUFFERED'
            self._force_data = f_group
            self._torque_data = t_group
            # Notify to other threads
            self._ft_status.acquire()
            self._ft_status.notify()
            self._ft_status.release()
        else:
            self._mode_trigger.clear()

    def _logging_data_mode(self):
        """Store data  mode for the ft-sensor."""
        log('Performing logging data mode.',ALWAYS_LOG_LEVEL)
        if self._current_mode != self._mode: # when current mode is different from set mode
            self._ft.set_streaming(streaming='REALTIME_BUFFERED',samples=0)
            self._current_mode = self._mode
            # files to store data
            c_time = time.time()
            head = open('Header.txt', 'w')
            internalsamples = open('Internal', 'w')
            fx = open('Force x-axis.txt', 'w')
            fy = open('Force y-axis.txt', 'w')
            fz = open('Force z-axis.txt', 'w')
            mx = open('Torque x-axis.txt', 'w')
            my = open('Torque y-axis.txt', 'w')
            mz = open('Torque z-axis.txt', 'w')
            time_file = open('Time.txt','w')
            start_time = time.time()
            secs = 0
        while secs < self._time_logging:
            end_time = time.time()
            diff = int(end_time - start_time)
            secs = diff % 60
            log('Logging of data in seconds :' + str(secs),self._log_level)
            h,f,t = self._ft.get_data_ATI(sync=True) # get data from atibox
            n_time = time.time()
            for i in range(0,self._buffersize):
                head.write(str(h[i,0]) + '\n')
                internalsamples.write(str(h[i,1]) + '\n')
                fx.write(str(f[i,0]) + '\n')
                fy.write(str(f[i,1]) + '\n')
                fz.write(str(f[i,2]) + '\n')
                mx.write(str(t[i,0]) + '\n')
                my.write(str(t[i,1]) + '\n')
                mz.write(str(t[i,2]) + '\n')
            time_file.write(str(n_time-c_time)  + '\n')
        self._logging_terminated.set()
        self.set_mode(mode='STOP')
        head.close()
        fx.close()
        fy.close()
        fz.close()
        mx.close()
        my.close()
        mz.close()
        time_file.close()
        self._operational_mode = 'LOGGING_DATA'

        log('Storing of data is ended', self._log_level)

    def _error(self):
        """When error has happened. To nodity the others, with that
        the data is none."""
        self._contact_status.acquire()
        self._contact_status.notify()
        self._contact_status.release()
        self._contact_translation.acquire()
        self._contact_translation.notify()
        self._contact_translation.release()

    def _notify(self):
        """Notify to other threads."""
        log('Performing notify', self._log_level)
        self._contact_status.acquire()
        self._contact_status.notify()
        self._contact_status.release()
        if self._previous_contact != self._force_contact:
            self._previous_contact = self._force_contact
            #Notify change in status of contact
            self._contact_translation.acquire()
            self._contact_translation.notify()
            self._contact_translation.release()

    def _update_bias(self):
        """Update bias, there are two modes, static and dynamic."""
        log('Performing Update Bias.', ALWAYS_LOG_LEVEL)
        if self._wait_updatebias_mode.isSet() == False:
            if self._current_mode != self._mode: # when current mode is different from set mode
                self._ft.set_streaming(streaming='REALTIME_BUFFERED',samples=0) # set to buffer mode
                self._ft.wait_for_mode(mode='REALTIME_BUFFERED',timeout=ATI_TIMEOUT)
                self._current_mode = self._mode

            if self._type == 'static':
                group_force, group_torque = self._ft.get_data_ATI(sync=True,timeout=7,data_type='force_torque') # get data from ati-box
                for i in range (1,self._N): # loop all periods
                    f,t = self._ft.get_data_ATI(sync=True,timeout=ATI_TIMEOUT,data_type='force_torque') # get data from ati-box
                    if f == None:
                        raise self.Error('No response from the sensor, timeout : ' + '"{}"'.format(str(ATI_TIMEOUT)))

                    group_force = np.append(group_force,f,axis=0)
                    group_torque = np.append(group_torque,t,axis=0)

                self._bh.set_bias_force(s=group_force)
                self._bh.set_bias_torque(s=group_torque)
                print(self._bh.get_bias_force())
            self._operational_mode = 'UPDATE_BIAS'
        else:
            self._mode_trigger.clear()
            #self._mode_trigger.wait() # wait until mode trigger is set

    def _stop_mode(self):
        """Stop mode for the ft-sensor."""
        log('Performing stop mode', self._log_level)
        if self._current_mode != self._mode: # when current mode is different from set mode
            self._ft.set_streaming(self._mode) # set the streaming to stop mode
            self._current_mode = self._mode # when current mode is different from set mode
            self._ft.wait_for_mode(mode='STOP',timeout=ATI_TIMEOUT) # wait until ati is in stop mode
            self._operational_mode = 'STOP' # set operation mode to stop

        self._mode_trigger.clear()

    def _averaging_realtime_mode(self):
        """Mode avg. the data from the buffer to avoid noise or others
        disturbines. The streaming will be the realtime buffered."""
        log('Performing average realtime mode.',self._log_level)
        if self._current_mode != self._mode: # when current mode is different from set mode
            self._ft.set_streaming(streaming='REALTIME_BUFFERED',samples=0)
            self._current_mode = self._mode # when current mode is different from set mode
            self._ft.wait_for_mode(mode='REALTIME_BUFFERED',timeout=ATI_TIMEOUT)

        f = self._ft.get_data_ATI(sync=True,timeout=self._no_data_timeout,data_type='force') # get data from atibox
        if f == None:
            self._force_contact = None
            self._error()
            raise self.Error('No response from the sensor, timeout : '
                             + '"{}"'.format(str(self._no_data_timeout)))
        bias_force = self._bh.get_bias_force() # bias force
        f_mean = np.average(f,axis=0)
        biased = f_mean - bias_force
        if np.linalg.norm(biased) > self._force_threshold:
            self._force_contact = True # Contact
        else:
            self._force_contact = False # No contact
        self._notify() # Notify to other threads
        #print(biased)
        self._operational_mode = 'AVG_REALTIME'

    def _set_operational_mode(self):
        """Update the events for when a specific mode is in process."""
        log('Performing set operational mode', self._log_level)
        if self._old_operational_mode != self._operational_mode:
            if self._operational_mode == 'STOP':
                self._wait_stop_mode.set()
            else:
                self._wait_stop_mode.clear()

            if self._operational_mode == 'UPDATE_BIAS':
                self._wait_updatebias_mode.set()
            else:
                self._wait_updatebias_mode.clear()

            if self._operational_mode == 'AVG_REALTIME':
                self._wait_avg_mode.set()
            else:
                self._wait_avg_mode.clear()
            if self._operational_mode == 'REALTIME_BUFFERED':
                self._wait_buffered_mode.set()
            else:
                self._wait_buffered_mode.clear()
            if self._operational_mode == 'LOGGING_DATA':
                self._wait_logging_data_mode.set()
            else:
                self._wait_logging_data_mode.clear()
            self._old_operational_mode = self._operational_mode

    def run(self):
        """ Running thread for the sensor. """
        log(self._name +' is running', ALWAYS_LOG_LEVEL)
        if self._ft.wait_startup(1) == False: # wait until the connector is started up.
              raise self.Error('Could not startup ATI. : ' + '"{}"'.format(str(self.name)))
        else:
            log(self._ft.name + ' is started', ALWAYS_LOG_LEVEL)
        if self._type == 'static':
            log(self._name + ' is in ' + self._type + ' mode.', ALWAYS_LOG_LEVEL)
        elif self._type == 'dynamic':
            log(self._name + ' is in ' + self._type + ' mode.', ALWAYS_LOG_LEVEL)

        if self._thread_alive.isSet(): #the thread is aldready runnning
            raise self.Error(self._name +' is already running !!!! : ' + '"{}"'.format(self._name))
        else: #set the thread to be running
            self._thread_alive.set() # active thread
            self._thread_terminated.clear() # deactive terminated thread

        while self._thread_alive.isSet():
            log(self._name + ' is runnin------------------------------------------------------g!!!', self._log_level)
            #print(self._mode)

            if self._mode !='LOGGING_DATA' and self._mode !='STOP':
                self._break_mode()
                self.wait_for_control() # wait for the controll to complete
                pass
            #start = time.time()
            try:
                method = getattr(self,MODES[self._mode])
            except AttributeError:
                raise self.Error(method + ' not found !!!! : ' + '"{}"'.format(self._name))
            else:
                method() # call task
            self._set_operational_mode()
            #end = time.time()
            #print(end - start)

            self._mode_trigger.wait() # wait until mode trigger is set

            """
            if self._mode !='LOGGING_DATA' and self._mode !='STOP' :

                end = time.time()*1000
                print(end - start)
                if (end - start) < CONTROLLER_PROCESSING_TIME*1000/2.0:
                    for i in range(1,int(CONTROLLER_PROCESSING_TIME*1000/np.ceil((end - start)))):
                        print('LOOOOOOOOOOOP')
                        print(end - start)
                        print(round(end - start))
                        print(int(3.0/(end - start)))
                        try:
                            method = getattr(self,MODES[self._mode])
                        except AttributeError:
                            raise self.Error(method + ' not found !!!! : ' + '"{}"'.format(self._name))
                        else:
                            method() # call task
                        print('TIMMMMMMMMMMMMMMMMMMMMMMMME')
                        end = time.time()
                    print(end - start)
            """
        log('Closing ' + self._name, ALWAYS_LOG_LEVEL)
        self._thread_alive.clear()
        self._thread_terminated.set()

    def stop(self):
        """Stop the thread."""
        log('Trying to stop ' + self._name, ALWAYS_LOG_LEVEL)
        self._thread_alive.clear()

    def wait_startup(self,timeout=None):
        """Wait to this thread is started up, expect
        if a timeout is given.
        Inputs:
        timeout:float-> timeout given in secs."""
        if self._thread_alive.wait(timeout):
            return True
        else:
            return False

    def wait_terminated(self,timeout=None):
        """Wait to this thread is terminated, expect
        if a timeout is given.
        Inputs:
        timeout:float-> timeout given in secs."""
        self.stop()
        if self._thread_terminated.wait(timeout):
            return True
        else:
            return False

    def __repr__(self):
        """String represenstation of the FT-sensor."""
        return ('Name:{self._name}, Force Threshold:{self._force_threshold}, Torque Threshold:{self._torque_threshold}').format(self=self)
