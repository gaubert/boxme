'''
Created on Aug 11, 2014

@author: guillaume.aubert@gmail.com

https://github.com/jmesmon/imu_9drazor/blob/master/src/SF9DOF_AHRS/SF9DOF_AHRS.pde

'''

import numpy as np

def normalize_matrix(a_mat):
    """
       Normalize something
    
    /**************************************************/
void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;

  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19

  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19

  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20

  renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

  renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);

  renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}
    
    """
    
    result = np.matrix('0 0 0 ; 0 0 0; 0 0 0')
    
    tempo = np.matrix('0 0 0 ; 0 0 0; 0 0 0')
    
    error = - np.vdot(a_mat[0][0], a_mat[1][0]) * .5 #eq.19
    
    tempo[0][0] = a_mat[1][0] * error #eq.19
    tempo[1][0] = a_mat[0][0] * error #eq.19
    
    tempo[0][0] += a_mat[0][0] #eq.19
    tempo[1][0] += a_mat[1][0] #eq.19
    
    tempo[2][0] = np.cross(tempo[0][0],tempo[1][0]) #eq.20
    
    renorm = .5 * (3 - np.vdot(tempo[0][0],tempo[0][0])) #eq.21
    result[0][0] = tempo[0][0] * renorm
    
    renorm = .5 * (3 - np.vdot(tempo[1][0],tempo[1][0])) #eq.21
    result[1][0] = tempo[1][0] * renorm
    
    renorm = .5 * (3 - np.vdot(tempo[2][0],tempo[2][0])) #eq.21
    result[2][0] = tempo[2][0] * renorm
    
    return result
    

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
    
    #Matrix Update
    #currently no drift correction
    update_matrix = np.matrix([[ 0 , - gyro_Dt * gyro_vector[2], gyro_Dt * gyro_vector[1]] ,
                               [ gyro_Dt * gyro_vector[2], 0, - gyro_Dt * gyro_vector[0]] ,
                               [ - gyro_Dt * gyro_vector[1], gyro_Dt * gyro_vector[0], 0]])
    
    tempo_matrix = dcm_matrix * update_matrix
    
    dcm_matrix = dcm_matrix + update_matrix
    
    #normalize the matrix
    dcm_matrix = normalize_matrix(dcm_matrix)
    
    
    
    
    print("dcm = %s" %(update_matrix))
    

if __name__ == '__main__':
    dcm_implementation()