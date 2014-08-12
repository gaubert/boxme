'''
Created on Aug 11, 2014

@author: guillaume.aubert@gmail.com

https://github.com/jmesmon/imu_9drazor/blob/master/src/SF9DOF_AHRS/SF9DOF_AHRS.pde

'''

import numpy as np
from cmath import sqrt

def drift_correction(accel_vector, gravity, err_roll_pitch):
    """
    /**************************************************/
        void Drift_correction(void)
        {
          float mag_heading_x;
          float mag_heading_y;
          float errorCourse;
          //Compensation the Roll, Pitch and Yaw drift. 
          static float Scaled_Omega_P[3];
          static float Scaled_Omega_I[3];
          float Accel_magnitude;
          float Accel_weight;
          
          
          //*****Roll and Pitch***************
        
          // Calculate the magnitude of the accelerometer vector
          Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
          Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
          // Dynamic weighting of accelerometer info (reliability filter)
          // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
          Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //  
        
          Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
          Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
          
          Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
          Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
          
          //*****YAW***************
          // We make the gyro YAW drift correction based on compass magnetic heading
         
          mag_heading_x = cos(MAG_Heading);
          mag_heading_y = sin(MAG_Heading);
          errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
          Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
          
          Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
          Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
          
          Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
          Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
        }
    """
    
    #calculate the magnitude of the accelerometer vector
    accel_magnitude = sqrt( (accel_vector[0]*accel_vector[0]) + (accel_vector[1]*accel_vector[1]) + accel_vector[2]*accel_vector[2])
    #scale to gravity
    accel_magnitude = accel_magnitude / gravity
    
    # Dynamic weighting of accelerometer info (reliability filter)
    # Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
    #Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //  
        
    accel_weigth =  (1 - 2 * abs(1 - accel_magnitude))
    
    np.cross(a, b, axisa, axisb, axisc, axis)
          
          Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
          Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
          
          Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
          Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);   
   
    

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