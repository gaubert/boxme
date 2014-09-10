'''
Created on Sep 10, 2014

@author: guillaume.aubert@gmail.com
'''

import DCM
import parsing

class Detector(object):
    '''
    classdocs
    '''


    def __init__(self, source, activate_debug):
        '''
        Constructor
        '''
        # file or serial source
        self._source = source
        self._debug  = activate_debug
        self._is_running = False
        
        if self._debug:
            self._parser = parsing.DebugParser()
        else:
            self._parser = parsing.CardSingleLineParser()
    
    def start(self):
        """
           Do the detection 
        """
        
        self._is_running = True
        
        #get first line
        line = self._source.read_line()
        vals = self._parser.parse_line(line)
        # Initialisation 
        acc_vec    = [vals['ax'], vals['ay'], vals['az']]
        mag_vec    = [vals['mx'], vals['my'], vals['mz']]
        gyr_vec    = [vals['gx'], vals['gy'], vals['gz']]
        dt         = vals['dt'] #integration time (Delta time between each measure
        
        # Create and initialize DCMizer
        dcmizer = DCM.DCMizer(acc_vec, mag_vec)
        
        dcmizer.get_dcm(acc_vec, mag_vec, gyr_vec, dt)
        
        fixed_acc_vec = dcmizer.project_to_inertial_frame(acc_vec)
        
        print("1st accel_vec= %s\r" % (acc_vec))
        print("1st fixed_accel_vec= %s\n" % (fixed_acc_vec))
                
        
        while self._is_running:
            line = self._source.read_line()
            # Loop through normal steps
            vals = self._parser.parse_line(line)
                        
            # Extract values
            acc_vec     = [vals['ax'], vals['ay'], vals['az']]
            mag_vec     = [vals['mx'], vals['my'], vals['mz']]
            gyr_vec     = [vals['gx'], vals['gy'], vals['gz']]
            dt          = vals['dt'] #integration time (Delta time between each measure
            
            dcmizer.get_dcm(acc_vec, mag_vec, gyr_vec, dt)
        
            fixed_acc_vec = dcmizer.project_to_inertial_frame(acc_vec)            
            print("accel_vec= %s\r" % (acc_vec))
            print("fixed_accel_vec= %s\n" % (fixed_acc_vec))

    def stop(self):
        """
           Stop the dectector
        """
        self._is_running = False