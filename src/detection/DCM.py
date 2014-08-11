'''
Created on Aug 11, 2014

@author: guillaume.aubert@gmail.com

https://github.com/jmesmon/imu_9drazor/blob/master/src/SF9DOF_AHRS/SF9DOF_AHRS.pde

'''

import numpy as np

def dcm_implementation():
    """
       tentative implementation of DCM
    """
    
    #init dcm matrix 
    dcm_matrix = np.matrix('1 0 0 ; 0 1 0 ; 0 0 1')
    
    update_matrix = np.matrix('0 1 2 ; 3 4 5 ; 6 7 8')
    
    
    #gyro integration time in ms
    gyro_Dt = 129 #what to put here ?
    gyro_vector  = np.array([30.00,46.00,-464.00])
    acce_vector  = np.array([6.00,9.00,266.00])
    omega_vector = np.array([0,0,0]) 
    
    omega_integr = np.array([0,0,0]) #omega integrator
    omega_propor = np.array([0,0,0]) #omega proportional
    
    #add omega integrator and proportional
    omega_vector = gyro_vector + omega_integr + omega_propor
    
    #currently no drift correction
    update_matrix = np.matrix([[ 0 , - gyro_Dt * gyro_vector[2], gyro_Dt * gyro_vector[1]] ,
                               [ gyro_Dt * gyro_vector[2], 0, - gyro_Dt * gyro_vector[0]] ,
                               [ - gyro_Dt * gyro_vector[1], gyro_Dt * gyro_vector[0], 0]])
    
    tempo_matrix = dcm_matrix * update_matrix
    
    dcm_matrix = dcm_matrix + update_matrix
    
    
    
    
    
    print("dcm = %s" %(update_matrix))
    

if __name__ == '__main__':
    dcm_implementation()