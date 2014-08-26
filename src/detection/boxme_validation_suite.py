'''
Created on Aug 26, 2014

@author: gaubert
'''
import unittest
import numpy as np
import DCM as dcm

class Test(unittest.TestCase):


    def setUp(self):
        pass


    def tearDown(self):
        pass


    def test_get_euler_angles_1(self):
        """
           first euler angles test
        """
        gyro_dt     = 0.34  #integration time (Delta time between each measure
        gyro_vec    = np.array([47.00,62.00,0.00])
        accel_vec   = np.array([-22.00,13.00,265.00])
        mag_vec     = np.array([47.00,242.00,724.00])
        
        #starting values (they are used for every steps)
        pitch    = 0.11 
        roll     = 0.06  
        yaw      = -1.00
    
        omega_p = np.array([0.018096,-0.034260,-0.000514])
        omega_i = np.array([0.000018,-0.000034,0.000004])
     
        input_dm = np.matrix("0.538803 0.842359 0.011088; -0.835298 0.535904 -0.122815; -0.109396 0.056911 0.992368")

        dcmizer = dcm.DCMizer(omega_p, omega_i, pitch, roll, yaw, input_dm)
        
        (n_pitch, n_roll, n_yaw) = dcmizer.compute_euler_angles(gyro_dt, gyro_vec, accel_vec, mag_vec)
        
        print("Calculate Euler Angles pitch: %s rad, roll: %s rad, yaw: %s rad" % (n_pitch, n_roll, n_yaw))
        
        #self.assertEquals()

    def test_acceleration_referential_change(self):
        """
           first euler angles test
        """
        gyro_dt     = 0.34  #integration time (Delta time between each measure
        gyro_vec    = np.array([47.00,62.00,0.00])
        accel_vec   = np.array([-22.00,13.00,265.00])
        mag_vec     = np.array([47.00,242.00,724.00])
        
        #starting values (they are used for every steps)
        pitch    = 0.11 
        roll     = 0.06  
        yaw      = -1.00
    
        omega_p = np.array([0.018096,-0.034260,-0.000514])
        omega_i = np.array([0.000018,-0.000034,0.000004])
     
        input_dm = np.matrix("0.538803 0.842359 0.011088; -0.835298 0.535904 -0.122815; -0.109396 0.056911 0.992368")

        dcmizer = dcm.DCMizer(omega_p, omega_i, pitch, roll, yaw, input_dm)
        
        (n_pitch, n_roll, n_yaw) = dcmizer.compute_euler_angles(gyro_dt, gyro_vec, accel_vec, mag_vec)
        
        print("Calculate Euler Angles pitch: %s rad, roll: %s rad, yaw: %s rad" % (n_pitch, n_roll, n_yaw))
        
        # yaw is rotation on x axis, pitch rotation on z axis, roll rotation y axis
        
        n_accel_vec = [ (accel_vec[0] * n_yaw) , (accel_vec[1] * n_roll) , (accel_vec[2] * n_pitch) ]
         
        
         


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()