__author__ = "Mats Larsen"
__copyright__ = "SINTEF, NTNU 2013"
__credits__ = ["Morten Lind"]
__license__ = "GPL"
__maintainer__ = "Mats Larsen"
__email__ = "matsla@{ntnu.no}"
__status__ = "Development"
#--------------------------------------------------------------------
#File: bias_handler.py
#Module Description
"""
This module is handle the bias, to adjust the bias by analysing the
noise band.
"""
#--------------------------------------------------------------------
#IMPORT
#--------------------------------------------------------------------
import traceback
import threading
import numpy as np

from freqclass import FrequencyAnalysis as FA
from chain import Chain
#import matplotlib.pyplot as plt

#--------------------------------------------------------------------
#CONSTANTS
#--------------------------------------------------------------------
LOG_LEVEL = 2 # Information level
FILE = 'handle_bias'
NAME = 'BIAS_HANDLER'
NUMBER_SIGNALS = 6 # number of the signals from the sensor
#--------------------------------------------------------------------
#METHODS
#-------------------------------------------------------------------
def log(msg, log_level=LOG_LEVEL):
    """Print a message, and track, where the log is invoked
    Input:
    -msg: message to be printed, ''
    -log_level: informationlevel"""
    global LOG_LEVEL
    if log_level <= LOG_LEVEL:
        print(str(log_level) + ' : handle_bias.py::' + traceback.extract_stack()[-2][2] + ' : ' + msg)

class BiasHandler(threading.Thread):
    """Class that handles all bias operations"""
    class Error(Exception):
        """Exception class."""
        def __init__(self, message):
            self.message = message
            Exception.__init__(self, self.message)
        def __repr__(self):
            return self.message

    def __init__(self,name='Biashandler',t_freq=50,n = 20,
                 prediction_size=4, handler_type='static',
                 log_level=3):
        """Initlize the bias handler.
        Input:
        name:string->Name of the instance.
        t_freq:float->threshold for the significant frequencies. """
        #assignment
        self._name = NAME + '#' + name
        self._t_freq = t_freq # threshold for frquency in %
        #self._size = filter_size
        self._prediction_size = prediction_size
        self._N = n # number of noise periods
        self._type = handler_type # define whihc type of handler
        self._log_level = log_level # information level

        # freq analyse class
        self._fa = FA(name='FA#1',log_level=self._log_level)

        #Threading
        threading.Thread.__init__(self)
        self.daemon = True
        #Event
        self._thread_status = threading.Event()
        self._thread_status.clear()
        #Type of handler
        if self._type == 'static':
            pass
        elif self._type == 'dynamic':

            self._noise_band = threading.Event() # signal in noise band
            self._noise_band.clear()

        #Reset
        self._min_noise = Chain(name='Min_noise',elements=np.zeros(NUMBER_SIGNALS),data_type=np.float)
        self._max_noise = Chain(name='Max_noise',elements=np.zeros(NUMBER_SIGNALS),data_type=np.float)
        self._peak_to_peak = Chain(name='Peak_to_peak',elements=np.zeros(NUMBER_SIGNALS),data_type=np.float)
        self._std = Chain(name='Standard Deviation',elements=np.zeros(NUMBER_SIGNALS),data_type=np.float)
        self._mean = Chain(name='Mean',elements=np.zeros(NUMBER_SIGNALS),data_type=np.float)
        self._T = Chain(name='Threshold_lowest freqency',elements=np.zeros(NUMBER_SIGNALS),data_type=np.float)
        self._offset = np.zeros(NUMBER_SIGNALS,dtype=np.float)

        # self._y = np.zeros(weighed_filter_size)
        #self._x = np.zeros(filter_size)
    def get_is_in_noiseband(self):
        if self._noise_band.isSet():
            return True
        else:
            return False

    def get_threshold_force(self,threshold_type):
        """Return the threshold for force."""

        if threshold_type=='double_std':
            std = self._std
            force_mean = 0.0
            for i in range(0,3):
                force_mean += std.get_data(i)
            force_threshold = force_mean / 3 * 2.0 # threshold for force
        return force_threshold

    def get_threshold_torque(self,threshold_type):
        """Return the threshold for torque."""
        if threshold_type=='double_std':
            std = self._std
            torque_mean = 0.0
            for i in range(0,3):
                torque_mean += std.get_data(i)
            torque_threshold = torque_mean / 3 * 2.0 # threshold for force
        return torque_threshold

    def set_mean_samplings(self,s):
        """Mean samplings in s."""
        log('Performing mean samplings',self._log_level)
        signal = Chain(name='FT_signals')
        for i in range(0,NUMBER_SIGNALS): # make 6 chain for f and t
            signal.add_tail(Chain(name='Signals_'+str(i)))
        for i in range(0,NUMBER_SIGNALS):
            mean = 0
            for n in range(0,s.get_data(i).len):
                mean += s.get_data(i).get_data(n)
            mean = mean / s.get_data(i).len
            self._offset.set_data(number=i,data=mean)

    def set_bias_force(self,s):
        """Set the bias for the force.
        Inputs:
        s:np-> a graoup of samplings."""
        self._bias_force = np.average(s,axis=0)

    def get_bias_force(self):
        """Return the bias for force"""
        return self._bias_force
    force_bias = property(get_bias_force,'Bias Force')

    def set_bias_torque(self,s):
        """Set the bias for the torque.
        Inputs:
        s:np-> a graoup of samplings."""
        self._bias_torque = np.average(s,axis=0)

    def get_bias_torque(self):
        """Return the bias for torque"""
        return self._bias_torque
    torque_bias = property(get_bias_torque,'Bias torque')
    
    def get_offset(self):
        """Return the offset"""
        return self._offset
    offset = property(get_offset,'Offset Property')

    def update_bias(self, x):
        log('Perfrom Update Bias', self._log_level)
        self._x = np.roll(self._x,1)
        #print(self._x)
        self._x[0] = x
        print('X')
        #print(self._x)

        if self.check_signal() == True:
            log('In noise band', self._log_level)
            low = self.low_pass_filter()
            log('Low pass Filter = ' + str(low), self._log_level)
            self._noise_band.set()
        else:
            log('Not In noise band', self._log_level)



            p =  self.prediction()
            log('Prediction = ' + str(p), self._log_level)
            #print(p)
            r = self.ratio_error()
            log('RATIO = ' + str(r), self._log_level)

            w = self.gaussian()
            #log('Gaussian = ' + str(w), self._log_level)

            wfitler = self.weighed_filter()

            self._noise_band.clear()

            #log(' Wieght Filter = ' + str(wfitler), self._log_level)

            #print(self._y)
        log('N = ' + str(self._N), self._log_level)
        log('T = ' + str(self._T), self._log_level)

    def check_signal(self):
        y = self._y[self._N*self._T -1]
        diff = np.absolute(self._x[0] - y)
        log('DIFF = '+ str(diff), self._log_level)
        if diff < self._peak_to_peak/2.0:

            mean = self.mean()
            log('Mean = ' + str(mean))
            for i in range (0,self._N):
                if np.absolute(self._x[i] - mean) < self._peak_to_peak/2.0:
                    #low = self.low_pass_filter()
                    pass
                else:
                    return False
            return True
        else:
            return False


    def mean(self):
        m = 0
        for i in range (self._N):
            m = m + self._x[i]
        m = m/self._N
        return m
    def low_pass_filter(self):
        j = 0
        for i in range ( int(self._N*self._T)):
            j = j + self._x[i]
        self._low = 1.0/(self._N * self._T - 1) * j
        return self._low

    def prediction(self):
        log('Performing Prediction', self._log_level)
        loop = self._prediction_size
        m=0
        for i in range(1,loop):
           m = m + 1.0/i * (self._y[1] - self._y[1+i])
        m = m/loop
        self._p = self._y[1] + m
        return self._p

    def ratio_error(self):
        log('Performing Ratio', self._log_level)
        print(self._p)
        print(self._x[0])
        print(self._peak_to_peak/2.0)
        self._r_e = np.absolute(self._p - self._x[0]) / self._peak_to_peak/2.0
        self._r_e.astype(np.float)
        print(self._r_e)
        return self._r_e

    def weighed_filter(self):
        self._y = np.roll(self._y,1)
        self._y[0] = self._w * self._p + (1 - self._w) * self._x[0]
        return self._y[0]

    def gaussian(self):
        self._w = np.exp(-np.power(self._r_e,2) / (2*np.power(self._std,2)))
        return self._w

    def get_std(self):
        """Returning the Standard Deviation."""
        return self._std
    std = property(get_std,'STD Property.')

    def investigate_signal(self, s, fs):
        """ Investigate the signal to find the noise band and lowest
        frequency.
        Inputs:
        s:array-> an array contain the signal.
        fs:float-> sampling frequency."""
        log('Performing Intestigate signal', self._log_level)

        for n in range(0,s.len):
            cs = s.get_data(n).convert(to_type='array')
            f , pFFT =  self._fa.get_fft(cs, nfft=256, fs=fs) # get frequencies and power
            log('Frequencies = ' + str(f), self._log_level)
            log('Power = ' + str(pFFT), self._log_level)
            low_f = np.zeros([])

            t_f = np.amax(pFFT) /100.0 * self._t_freq # threshold for significant frequencies
            for i in range (len(pFFT)): # loop all power though
                if pFFT[i]  > t_f:
                    low_f =  np.append(low_f, f[i])
                    #print(low_f)
            min_f = np.amin(low_f) # find the lowest frequency
            if min_f <= 0:
                min_f = 1
            log('Min Freq = ' + str(min_f), self._log_level)
            #Noise band """
            self._min_noise.set_data(number=n,data=np.amin(cs)) # get min noise
            self._max_noise.set_data(number=n,data=np.amax(cs)) # get max noise
            self._peak_to_peak.set_data(number=n,data=self._max_noise.get_data(n) - self._min_noise.get_data(n)) # get peak
            #Standard Deviation
            self._std.set_data(number=n,data=np.std(cs))
            #MEAN
            self._mean.set_data(number=n,data=np.mean(cs))
            # threshold for lowest frequency
            self._T.set_data(number=n,data= np.float(fs/min_f))
        #self._T = self._T.astype(np.float)

        #self._y = np.zeros(self._T * self._N)
        #self._x = np.zeros(self._T * self._N)
        #Print data
        """
        print(self._min_noise)
        print(self._max_noise)
        print(self._peak_to_peak)
        print(self._std)
        print(self._mean)
        print(self._T)
        """
        """
        #Plot of the Histigram
        plt.figure('hist')
        count, bins, ignored = plt.hist(s, 30, normed=True)
        plt.plot(bins, 1/(std * np.sqrt(2 * np.pi)) *
                np.exp( - (bins - mean)**2 / (2 * std**2) ),
          linewidth=2, color='r')
        #plt.show()
        """
    def run(self):
        log('Starting Thread, ' + self.get_name(), self._log_level)
        """ Run the thread, that checks if a collision occur """

        if self._thread_status.isSet():
            raise self.Error(
                'Could not Start Thread' + self.get_name()
                +' its already running !!!  : '
                + '"{}"'.format(str(self.get_name())))

        self._thread_status.set()
        while (self._thread_status.isSet()):
            pass


    def stop(self):
        """ Stopping the thread """
        log('Stopping Thread, ' + self.get_name(), self._log_level)
        self._thread_status.clear()


    def get_thread_status(self):
        """ Returning the thread status """
        return self._thread_status
    thread = property(run,stop,get_thread_status, 'Thread property')
