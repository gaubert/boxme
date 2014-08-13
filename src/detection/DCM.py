'''
Created on Aug 11, 2014

@author: guillaume.aubert@gmail.com

https://github.com/jmesmon/imu_9drazor/blob/master/src/SF9DOF_AHRS/SF9DOF_AHRS.pde

'''

import numpy as np
import math

KP_ROLLPITCH = 0.02
KI_ROLLPITCH = 0.00002
KP_YAW       = 1.2
KI_YAW       = 0.00002
GRAVITY      = 248  # this equivalent to 1G in the raw data coming from the accelerometer

def compass_heading(magnetom, roll, pitch):
    """ update compas values
    
      float mag_x;
      float mag_y;
      float cos_roll;
      float sin_roll;
      float cos_pitch;
      float sin_pitch;
    
      cos_roll = cos(roll);
      sin_roll = sin(roll);
      cos_pitch = cos(pitch);
      sin_pitch = sin(pitch);
    
      // Tilt compensated magnetic field X
      mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
      // Tilt compensated magnetic field Y
      mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
      // Magnetic Heading
      MAG_Heading = atan2(-mag_y, mag_x);
      
    """
    cos_roll  = math.cos(roll)
    sin_roll  = math.sin(roll)
    cos_pitch = math.cos(pitch)
    sin_pitch = math.sin(pitch)
    
    # Tilt compensated magnetic field X
    mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch
    
    # Tilt compensated magnetic field Y
    mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
    
    # Magnetic Heading
    return math.atan2(-mag_y, mag_x);
   

def euler_angles(dcm_matrix):
    """
	   Get euler angles
	   void Euler_angles(void)
        {
          pitch = -asin(DCM_Matrix[2][0]);
          roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
          yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
        }
    """
    pitch = - math.asin(dcm_matrix[2][0])
    roll  = math.atan2(dcm_matrix[2][1], dcm_matrix[2][2])
    yaw   = math.atan2(dcm_matrix[1][0], dcm_matrix[0][0]);
    
    return (pitch, roll, yaw)
    
def drift_correction(accel_vector, dcm_matrix, err_roll_pitch, err_yaw, omega_p, omega_i):
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
    scaled_omega_p = np.array([0,0,0])
    scaled_omega_i = np.array([0,0,0])
     
    #ROLL and PITCH
    #calculate the magnitude of the accelerometer vector
    accel_magnitude = math.sqrt( (accel_vector[0]*accel_vector[0]) + (accel_vector[1]*accel_vector[1]) + accel_vector[2]*accel_vector[2])
    #scale to gravity
    accel_magnitude = accel_magnitude / GRAVITY
    
    # Dynamic weighting of accelerometer info (reliability filter)
    # Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
    #Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //  
        
    accel_weigth =  (1 - 2 * abs(1 - accel_magnitude))
    
    #adjust ground reference
    err_roll_pitch = np.cross(accel_vector[0], dcm_matrix)
    
    omega_p[0] = err_roll_pitch[0] * (KP_ROLLPITCH * accel_weigth)
    
    scaled_omega_i[0] = err_roll_pitch[0] * (KI_ROLLPITCH * accel_weigth) 
    
    omega_i += scaled_omega_i
    
    #YAW   
    #We make the gyro YAW drift correction based on compass magnetic heading
    
    mag_heading_x = math.cos(mag_heading)
    mag_heading_y = math.sin(mag_heading)
    
    #calculate yaw error
    error_course = (dcm_matrix[2][0] * mag_heading_y) - (dcm_matrix[1][0] * mag_heading_x)
    
    #Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
    error_yaw = dcm_matrix[2][0] * error_course
    
    #.01proportional of YAW.
    scaled_omega_p[0] = error_yaw[0] * KP_YAW
    #Adding Proportional 
    omega_p += scaled_omega_p
    #.00001Integrator
    scaled_omega_i[0] = error_yaw[0] * KI_YAW
    
    #Adding integrator to the Omega_I
    omega_i += scaled_omega_i
    
        
    

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
    

def dcm_implementation(mag_heading, magnetom, roll, pitch, yaw, omega_p, omega_i, err_roll_pitch, err_yaw, omega_vector):
    """
       tentative implementation of DCM
    """
    #init dcm matrix 
    dcm_matrix = np.matrix('1 0 0 ; 0 1 0 ; 0 0 1')
    
    #MATRIX update to be done
    update_matrix = np.matrix('0 1 2 ; 3 4 5 ; 6 7 8')
    
        
    #gyro integration time in ms
    gyro_Dt = 129 #what to put here ?
    gyro_vector  = np.array([30.00,46.00,-464.00])
    accel_vector  = np.array([6.00,9.00,266.00])
    
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
    
    dcm_matrix = dcm_matrix + tempo_matrix
    
    ######## End of update matrix
    
    #NORMALIZE the matrix
    dcm_matrix = normalize_matrix(dcm_matrix)
    
    #DRIFT_CORRECTION
    drift_correction(accel_vector, dcm_matrix, err_roll_pitch, err_yaw, omega_p, omega_i)
    
    #Euler angles
    (pitch, roll, yaw) = euler_angles(dcm_matrix)
    
    
    print("dcm = %s" %(update_matrix))
    

if __name__ == '__main__':
    
    #values from magnetometer to be read
    magnetom = np.array(0,0,0)
    #roll angle in rad
    roll     = 0.5
    pitch    = 0.5
    yaw      = 0.5
    
    err_yaw = np.array(0,0,0)
    err_roll_pitch = np.array(0,0,0)
    
    omega_vector = np.array(0,0,0) #Corrected Gyro_Vector data
    omega_p = np.array(0, 0, 0) # Omega Proportional correction
    omega_i= np.array(0,0,0) # Omega Integrator

    #init values
    mag_heading = compass_heading()
    
    dcm_implementation(mag_heading, magnetom, roll, pitch, yaw, omega_p, omega_i, err_roll_pitch, err_yaw, omega_vector )