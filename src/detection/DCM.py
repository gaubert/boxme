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

class DCMizer(object):


    def __init__(self):
        """
        """
        #init dcm matrix
        self._dcm_matrix     = np.matrix('1 0 0 ; 0 1 0 ; 0 0 1')
        
        self._err_roll_pitch = np.array([0,0,0])
        self._err_yaw        = np.array([0, 0, 0])
        
        self._omega_p        = np.array([0, 0, 0]) # Omega Proportional correction
        self._omega_i        = np.array([0,0,0])   # Omega Integrator
    
    @classmethod
    def compass_heading(cls, magnetom, roll, pitch):
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
    
    @classmethod
    def euler_angles(cls, a_dcm_matrix):
        """
           Get euler angles
           void Euler_angles(void)
            {
              pitch = -asin(DCM_Matrix[2][0]);
              roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
              yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
            }
        """
        #TODO Need also to resize properly the matrix as a 1D array
        # Use the vector trigonometric operators 
        #pitch = - math.asin( a_dcm_matrix[2][0] )
        #roll  = math.atan2(  (np.array(a_dcm_matrix[2][1]))[0], (np.array(a_dcm_matrix[2][2])[0]) )
        #yaw   = math.atan2(  (np.array(a_dcm_matrix[1][0]))[0], (np.array(a_dcm_matrix[0][0]))[0] );
    
        val   = np.array(a_dcm_matrix[2][0])[0]
        pitch = np.array([-math.asin(val[0]), -math.asin(val[1]), -math.asin(val[2])])
        
        val1  = np.array(a_dcm_matrix[2][1])[0]
        val2  = np.array(a_dcm_matrix[2][2])[0]
        roll  = np.array( [ math.atan2(val1[0], val2[0]), 
                            math.atan2(val1[1], val2[1]), 
                            math.atan2(val1[1], val[2])] )
        
        
        val1 = np.array(a_dcm_matrix[1][0])[0]
        val2 = np.array(a_dcm_matrix[0][0])[0]
        yaw = np.array( [ math.atan2(val1[0], val2[0]), 
                          math.atan2(val1[1], val2[1]), 
                          math.atan2(val1[2], val2[2])] )
    
    
        return (pitch, roll, yaw)


    @classmethod
    def normalize_matrix(cls, a_mat):
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
    
    def drift_correction(self, accel_vector, a_omega_p, a_omega_i):
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
        ret_omega_p = a_omega_p
        ret_omega_i = a_omega_i
        
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
        print("dcm_matrix = %s" % (self._dcm_matrix[2][0]))
        
        #need to resize that part using numpy features when I will have the doc
        calc_val = np.cross(accel_vector, self._dcm_matrix[2][0])
        self._err_roll_pitch = calc_val[0]
        
        v = self._err_roll_pitch[0] * (KP_ROLLPITCH * accel_weigth)
        ret_omega_p[0] = v
        
        v1 = self._err_roll_pitch[0] * (KI_ROLLPITCH * accel_weigth) 
        scaled_omega_i[0] = v1
        
        ret_omega_i += scaled_omega_i
        
        #YAW   
        #We make the gyro YAW drift correction based on compass magnetic heading
        
        mag_heading_x = math.cos(mag_heading)
        mag_heading_y = math.sin(mag_heading)
        
        #calculate yaw error
        #TODO. It is a matrix [[]] when it should be an array
        #change dimension
        error_course = (self._dcm_matrix[2][0] * mag_heading_y) - (self._dcm_matrix[1][0] * mag_heading_x)
        error_course = np.array(error_course)[0]
        
        #Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
        #TODO Need to resize dcm_matrix[2][0] as an array to have the rigth dim
        dum = self._dcm_matrix[2][0]
        
        self._err_yaw = np.array(self._dcm_matrix[2][0])[0] * error_course
        
        #.01proportional of YAW.
        scaled_omega_p[0] = self._err_yaw[0] * KP_YAW
        #Adding Proportional 
        ret_omega_p += scaled_omega_p
        #.00001Integrator
        scaled_omega_i[0] = self._err_yaw[0] * KI_YAW
        
        #Adding integrator to the Omega_I
        ret_omega_i += scaled_omega_i
        
        return (ret_omega_p, ret_omega_i)
    
    def _matrix_update(self, omega, gyro_vector, omega_vector, gyro_Dt, accel_vector, use_omega):
        """
           matrix update
           void Matrix_update(void)
            {
              Gyro_Vector[0]=Gyro_Scaled_X(read_adc(0)); //gyro x roll
              Gyro_Vector[1]=Gyro_Scaled_Y(read_adc(1)); //gyro y pitch
              Gyro_Vector[2]=Gyro_Scaled_Z(read_adc(2)); //gyro Z yaw
            
              Accel_Vector[0]=accel_x;
              Accel_Vector[1]=accel_y;
              Accel_Vector[2]=accel_z;
            
              Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
              Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term
            
              //Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement
            
             #if OUTPUTMODE==1
              Update_Matrix[0][0]=0;
              Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
              Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
              Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
              Update_Matrix[1][1]=0;
              Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
              Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
              Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
              Update_Matrix[2][2]=0;
             #else                    // Uncorrected data (no drift correction)
              Update_Matrix[0][0]=0;
              Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
              Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
              Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
              Update_Matrix[1][1]=0;
              Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
              Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
              Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
              Update_Matrix[2][2]=0;
             #endif
            
              Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c
            
              for(int x=0; x<3; x++) //Matrix Addition (update)
              {
                for(int y=0; y<3; y++)
                {
                  DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
                }
              }
            }
        """
                
        #TODO need to change that
        #add omega integrator and proportional
        omega[0]        = gyro_vector[0] + self._omega_i[0]
        omega_vector[0] = omega[0] + self._omega_p[0]
        
        # if output mode == 1 in original code ?
        if use_omega:
            vec = omega_vector
        else:
            vec = gyro_vector
            
        #Matrix Update
        #currently no drift correction
        update_matrix = np.matrix([[ 0 , - gyro_Dt * vec[2], gyro_Dt * vec[1]] ,
                                   [ gyro_Dt * vec[2], 0, - gyro_Dt * vec[0]] ,
                                   [ - gyro_Dt * vec[1], gyro_Dt * vec[0], 0]])
                
        tempo_matrix = self._dcm_matrix * update_matrix
        
        self._dcm_matrix = self._dcm_matrix + tempo_matrix
        
    def compute_dcm(self, mag_heading, magnetom, omega, gyro_Dt, gyro_vector):
        """
           tentative implementation of DCM
        """     
        use_omega = False # OUTPUTMODE=1 use omega or gyro
        
        # Corrected Gyro Vector
        # Add omega integrator and proportional
        omega_vector = gyro_vector + self._omega_i + self._omega_p
        
        # update dcm matrix 
        self._matrix_update(omega, gyro_vector, omega_vector, gyro_Dt, accel_vector, use_omega)
        
        #NORMALIZE the matrix
        self._dcm_matrix = DCMizer.normalize_matrix(self._dcm_matrix)
        
        #DRIFT_CORRECTION                              
        (self._omega_p, self._omega_i) = self.drift_correction(accel_vector, self._omega_p, self._omega_i)
        
        #Euler angles
        (pitch, roll, yaw) = DCMizer.euler_angles(self._dcm_matrix)
        
        
        print("pitch = %s, roll = %s, yaw = %s" %(pitch, roll, yaw))
        
        return (pitch, roll, yaw)  
    
def raw_to_rad(a_in):
    """
       Transform raw val to Rad
    """
    return a_in * 0.01745329252 # pi/180

if __name__ == '__main__':
    
    """
       info: The full-scale range of the gyro sensors is preset to +-2000 degrees per second (deg/s). 
       acceleration => divide by 256 to get a value in m/s-2 (x, y, z)
       gyro => rad/s-1
       Voila les donnees:
        Example:
        #T-gyr#178#Gm-Raw#24.00,42.00,-2.00#T-acc#182#Am-Raw#7.00,-3.00,265.00#T-mag#188#Mm-Raw#-119.00,66.00,585.00
        #YPR=#T-YPR#198-151.41,-1.21,-0.39
        
        Ca veut dire: 
        au temps 178ms, le gyro a donne en RAW 24.00,42.00,-2.00
        au temps 182ms, l accel a donne en RAW 7.00,-3.00,265.00
        au temps 188ms, le mag a donne en RAW -119.00,66.00,585.00
        
        Ces donnees ont ete utilisees par l algo sur la carte et donnent les angles d Euler suivant au temps:
        198ms: -151.41,-1.21,-0.39
        
        
        #T-YPR#149.00#YPR=#-129.58,-1.24,-0.60#T-acc#143.00#Am-Raw#8.19,-3.07,271.36#T-mag#143.00#Mm-Raw#-12.83,17.50,93.83#T-gyr#141.00#Gm-Raw#23.00,44.00,-628.00#DCM:#-0.64,0.77,0.02,-0.77,-0.64,0.01,0.02,-0.01,1.00,

    """
    
    ### Input values
    
    #values from magnetometer to be read
    magnetom      = np.array([-12.83, 17.50, 93.83])
    gyro_Dt       = 0.2  #integration time (Delta time between each measure
    gyro_vector   = np.array([raw_to_rad(23.00), raw_to_rad(44.00), raw_to_rad(-628.00)]) #to be measured
    accel_vector  = np.array([(8.19/256),(-3.07/256),(271.36/256)]) #to be measured
    
    #starting values (they are used for every steps)
    #init values are null
    roll     = 0.0 
    pitch    = 0.0
    yaw      = 0.0
    
    err_yaw = np.array([0,0,0])
    err_roll_pitch = np.array([0,0,0])
    
    omega = np.array([0,0,0])

    #init values
    mag_heading = DCMizer.compass_heading(magnetom, roll, pitch)
    
    dcmizer = DCMizer()
    
    dcmizer.compute_dcm(mag_heading, magnetom, omega, gyro_Dt, gyro_vector)
