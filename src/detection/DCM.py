'''
Created on Aug 11, 2014

@author: guillaume.aubert@gmail.com

https://github.com/jmesmon/imu_9drazor/blob/master/src/SF9DOF_AHRS/SF9DOF_AHRS.pde

'''
import Parser
import numpy as np
import math

KP_ROLLPITCH = 0.02
KI_ROLLPITCH = 0.00002
KP_YAW       = 1.2
KI_YAW       = 0.00002
GRAVITY      = 256.00  # this equivalent to 1G in the raw data coming from the accelerometer

# SENSOR CALIBRATION
#*****************************************************************/
# How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
# Put MIN/MAX and OFFSET readings for your board here!
# Accelerometer
# "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
ACCEL_X_MIN = -250.0
ACCEL_X_MAX = 250.0
ACCEL_Y_MIN = -250.0
ACCEL_Y_MAX = 250.0
ACCEL_Z_MIN = -250.0
ACCEL_Z_MAX = 250.0

# Magnetometer (standard calibration mode)
# "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
MAGN_X_MIN  = -600.0
MAGN_X_MAX = 600
MAGN_Y_MIN = -600
MAGN_Y_MAX = 600
MAGN_Z_MIN = -600
MAGN_Z_MAX = 600

# Gyroscope
# "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
GYRO_AVERAGE_OFFSET_X = 0.0
GYRO_AVERAGE_OFFSET_Y = 0.0
GYRO_AVERAGE_OFFSET_Z = 0.0

# Sensor calibration scale and offset values
ACCEL_X_OFFSET = (ACCEL_X_MIN + ACCEL_X_MAX) / 2.0
ACCEL_Y_OFFSET = (ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0
ACCEL_Z_OFFSET = (ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0
ACCEL_X_SCALE  = GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET)
ACCEL_Y_SCALE  = GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET)
ACCEL_Z_SCALE  = GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET)

MAGN_X_OFFSET = (MAGN_X_MIN + MAGN_X_MAX) / 2.0
MAGN_Y_OFFSET = (MAGN_Y_MIN + MAGN_Y_MAX) / 2.0
MAGN_Z_OFFSET = (MAGN_Z_MIN + MAGN_Z_MAX) / 2.0
MAGN_X_SCALE  = 100.0 / (MAGN_X_MAX - MAGN_X_OFFSET)
MAGN_Y_SCALE  = 100.0 / (MAGN_Y_MAX - MAGN_Y_OFFSET)
MAGN_Z_SCALE  = 100.0 / (MAGN_Z_MAX - MAGN_Z_OFFSET)

GYRO_GAIN = 0.06957 # same gain on all axes

#utility functions
def deg_to_rad(a_in):
    """
       Transform degrees val to radians
    """
    return a_in * 0.01745329252 # pi/180

def rad_to_deg(a_in):
    """
       rad to deg (180/pi)
    """
    return a_in * 57.2957795131

def gyro_scaled_rad(a_gyro_val):
    """ 
       Gain for gyroscope 
       Calculate the scaled gyro readings in radians per second
    """
    # Gain for gyroscope (ITG-3200)
    return a_gyro_val * deg_to_rad(GYRO_GAIN)

class DCMizer(object):


    def __init__(self, omega_p, omega_i, yaw, roll, pitch, input_dcm = None):
        """
          @omega_p   : initial omega proportional correction
          @omega_i   : initial omega integrator 
          @input_dcm : initial DCM matrix
        """
        self.reset(omega_p, omega_i, yaw, roll, pitch, input_dcm) # init is a standard reset
    
    def reset(self, omega_p, omega_i, yaw, roll, pitch, input_dcm = None):
        """
           reset internal values passing the following arguments
        """
        #init dcm matrix
        if input_dcm is not None:
            self._dcm_matrix = input_dcm 
        else: 
            np.matrix('1 0 0 ; 0 1 0 ; 0 0 1')
        
        self._err_roll_pitch = np.array([0,0,0])
        self._err_yaw        = np.array([0, 0, 0])
        
        self._omega_p        = omega_p # Omega Proportional correction
        self._omega_i        = omega_i # Omega Integrator
        
        self._pitch          = pitch
        self._roll           = roll
        self._yaw            = yaw
        
    
    @classmethod
    def init_rotation_matrix(cls, yaw, roll, pitch):
        """
        
           // Init rotation matrix using euler angles
            void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll)
            {
              float c1 = cos(roll);
              float s1 = sin(roll);
              float c2 = cos(pitch);
              float s2 = sin(pitch);
              float c3 = cos(yaw);
              float s3 = sin(yaw);
            
              // Euler angles, right-handed, intrinsic, XYZ convention
              // (which means: rotate around body axes Z, Y', X'') 
              m[0][0] = c2 * c3;
              m[0][1] = c3 * s1 * s2 - c1 * s3;
              m[0][2] = s1 * s3 + c1 * c3 * s2;
            
              m[1][0] = c2 * s3;
              m[1][1] = c1 * c3 + s1 * s2 * s3;
              m[1][2] = c1 * s2 * s3 - c3 * s1;
            
              m[2][0] = -s2;
              m[2][1] = c2 * s1;
              m[2][2] = c1 * c2;
            }

        
        """
        
        c1 = math.cos(roll)
        s1 = math.sin(roll);
        
        c2 = math.cos(pitch)
        s2 = math.sin(pitch);
        
        c3 = math.cos(yaw)
        s3 = math.sin(yaw);
        
        # Euler angles, right-handed, intrinsic, XYZ convention
        # (which means: rotate around body axes Z, Y', X'')     
        return np.matrix([[ c2*c3 , c3*s1*s2 - c1*s3, s1*c3 + c1 *c3 *s2] ,
                            [ c2*s3, c1*c3 + s1*s2*s3, c1*s2*s3 - c3*s1] ,
                            [ -s2, c2*s1, c1*c2]])
        
    
    @classmethod
    def reset_sensor_fusion(cls, accel, magnetom):
        """
            // Read every sensor and record a time stamp
            // Init DCM with unfiltered orientation
            // TODO re-init global vars?
            void reset_sensor_fusion() {
              float temp1[3];
              float temp2[3];
              float xAxis[] = {1.0f, 0.0f, 0.0f};
            
              read_sensors();
              timestamp = millis();
              
              // GET PITCH
              // Using y-z-plane-component/x-component of gravity vector
              pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
                
              // GET ROLL
              // Compensate pitch of gravity vector 
              Vector_Cross_Product(temp1, accel, xAxis);
              Vector_Cross_Product(temp2, xAxis, temp1);
              // Normally using x-z-plane-component/y-component of compensated gravity vector
              // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
              // Since we compensated for pitch, x-z-plane-component equals z-component:
              roll = atan2(temp2[1], temp2[2]);
              
              // GET YAW
              Compass_Heading();
              yaw = MAG_Heading;
              
              // Init rotation matrix
              init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
            }
        """
        
        x_axis = np.array([1.0, 0.0, 0.0])
        # GET PITCH
        #Using y-z-plane-component/x-component of gravity vector
        pitch = - math.atan2(accel[0], math.sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
                
        # GET ROLL
        # Compensate pitch of gravity vector 
        temp1 = np.cross(accel, x_axis);
        temp2 = np.cross(x_axis, temp1);
        
        # Normally using x-z-plane-component/y-component of compensated gravity vector
        # roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
        # Since we compensated for pitch, x-z-plane-component equals z-component:
        roll = math.atan2(temp2[1], temp2[2]);
              
        # GET YAW
        yaw = cls.compass_heading(magnetom, roll, pitch);
        
        dcm_matrix = cls.init_rotation_matrix(roll, pitch, yaw)
        
        return (dcm_matrix, yaw, pitch, roll)
        
    
    
    def _compass_heading(self, magnetom, pitch, roll):
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
        
        print("magnetom[0]=%s, magnetom[1]=%s, magnetom[2]=%s\n" %(magnetom[0], magnetom[1], magnetom[2]))
        
        print("cos_roll=%s, sin_roll=%s, cos_pitch=%s, sin_pitch=%s\n" % (cos_roll, sin_roll, cos_pitch, sin_pitch))
        
        print("mag_x=%s, mag_y=%s, mag_heading=%s\n" % (mag_x, mag_y, math.atan2(-mag_y, mag_x)))
  
        # Magnetic Heading
        return math.atan2(-mag_y, mag_x);
    
    @classmethod
    def dcm_to_euler_angles(cls, a_dcm_matrix):
        """
           Get euler angles
           void Euler_angles(void)
            {
              pitch = -asin(DCM_Matrix[2][0]);
              roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
              yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
            }
        """
      
        pitch = - math.asin(a_dcm_matrix[2,0])
        
        roll  = math.atan2(a_dcm_matrix[2,1],a_dcm_matrix[2,2])
        
        yaw   = math.atan2(a_dcm_matrix[1,0],a_dcm_matrix[0,0])

        return (pitch, roll, yaw)


    
    def _normalize_matrix(self, a_mat):
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

        
        error = - np.vdot(a_mat[0][0], a_mat[1][0]) * .5 #eq.19
        
        lign1 = a_mat[1][0] * error #eq.19
        
        lign2 = a_mat[0][0] * error #eq.19
        
        
        lign1 += a_mat[0][0] #eq.19
        lign2 += a_mat[1][0] #eq.19
        
        lign3 = np.cross(lign1, lign2) #eq.20
       
        
        renorm = .5 * (3 - np.vdot(lign1,lign1)) #eq.21
        lign1  = lign1 * renorm
        
        renorm = .5 * (3 - np.vdot(lign2, lign2)) #eq.21
        lign2  = lign2 * renorm
        
        renorm = .5 * (3 - np.vdot(lign3,lign3)) #eq.21
        lign3 = lign3 * renorm
        
        lign1 = np.squeeze(np.array(lign1))
        lign2 = np.squeeze(np.array(lign2))
        lign3 = np.squeeze(np.array(lign3))
        
        self._dcm_matrix = np.matrix([lign1, lign2, lign3])
        
    
    
    def _compensate_sensor_error(self, accel, magnetom, gyro):
        """
        
        // Apply calibration to raw sensor readings
        void compensate_sensor_errors() {
            // Compensate accelerometer error
            accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
            accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
            accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;
        
            // Compensate magnetometer error
        #if CALIBRATION__MAGN_USE_EXTENDED == true
            for (int i = 0; i < 3; i++)
              magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
            Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
        #else
            magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
            magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
            magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
        #endif
        
            // Compensate gyroscope error
            gyro[0] -= GYRO_AVERAGE_OFFSET_X;
            gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
            gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
        }
        
        """
        
        res_accel = np.array([ (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE ,
                               (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE ,
                               (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE
                             ])
        
        #currently only use the simple error compensation
        res_magnetom = np.array([ (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE,
                                  (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE,
                                  (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE
                                ])
        
        #compensate gyro erro
        res_gyro = np.array( [ gyro[0] - GYRO_AVERAGE_OFFSET_X,
                               gyro[1] - GYRO_AVERAGE_OFFSET_Y,
                               gyro[2] - GYRO_AVERAGE_OFFSET_Z
                             ])
        
        return (res_accel, res_magnetom, res_gyro)
        
    
    def _drift_correction(self, accel_vector, a_omega_p, a_omega_i, mag_heading):
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
        ret_omega_i = a_omega_i
         
        #ROLL and PITCH
        #calculate the magnitude of the accelerometer vector
        accel_magnitude = math.sqrt( (accel_vector[0]*accel_vector[0]) + (accel_vector[1]*accel_vector[1]) + accel_vector[2]*accel_vector[2])
        #scale to gravity
        accel_magnitude = accel_magnitude / GRAVITY
        
        # Dynamic weighting of accelerometer info (reliability filter)
        # Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
        accel_weigth = (1 - 2 * abs(1 - accel_magnitude))
        if accel_weigth < 0:
            accel_weigth = 0
        elif accel_weigth > 1:
            accel_weigth = 1
    
        #adjust ground reference
        print("dcm_matrix = %s" % (self._dcm_matrix[2][0]))
        
        #need to resize that part using numpy features when I will have the doc
        self._err_roll_pitch = np.cross(accel_vector, np.squeeze(np.array(self._dcm_matrix[2][0])))
        
        ret_omega_p = self._err_roll_pitch * (KP_ROLLPITCH * accel_weigth)
       
        scaled_omega_i = self._err_roll_pitch * (KI_ROLLPITCH * accel_weigth) 
        
        ret_omega_i += scaled_omega_i
        
        #YAW   
        #We make the gyro YAW drift correction based on compass magnetic heading
        
        """
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
        
        mag_heading_x = math.cos(mag_heading)
        mag_heading_y = math.sin(mag_heading)
        
        #calculate yaw error
        error_course = (self._dcm_matrix[0,0] * mag_heading_y) - (self._dcm_matrix[1,0] * mag_heading_x)
       
        #Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.   
        self._err_yaw = np.squeeze(np.array(self._dcm_matrix[2][0])) * error_course
        
        #.01proportional of YAW.
        # can be simplified in 
        ret_omega_p += self._err_yaw * KP_YAW
        
        #.00001Integrator
        #Adding integrator to the Omega_I
        ret_omega_i += self._err_yaw * KI_YAW
        
        self._omega_p = ret_omega_p
        self._omega_i = ret_omega_i
    
    
    def _matrix_update(self, dcm_matrix, acc_vec, mag_vec, gyr_vec, dt, use_omega, omega_p, omega_i):
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
        
        #get scaled gyro values
        # Corrected Gyro Vector
        gyr_vec[0] = gyro_scaled_rad(gyr_vec[0]); #gyro x roll
        gyr_vec[1] = gyro_scaled_rad(gyr_vec[1]); #gyro y pitch
        gyr_vec[2] = gyro_scaled_rad(gyr_vec[2]); #gyro z yaw
        
        # Add omega integrator and proportional
        omega_vector = gyr_vec + omega_i + omega_p
        
        print("omega_vector[0] = %s, omega_vector[1] = %s, omega_vector[2] = %s\n" % (omega_vector[0], omega_vector[1], omega_vector[2]))
        
        # if output mode == 1 in original code ?
        if use_omega:
            vec = omega_vector
        else:
            vec = gyr_vec
            
        #Matrix Update
        update_matrix = np.matrix([[ 0 , - dt * vec[2], dt * vec[1]] ,
                                   [ dt * vec[2], 0, - dt * vec[0]] ,
                                   [ - dt * vec[1], dt * vec[0], 0]])
                
        tempo_matrix = self._dcm_matrix * update_matrix
        
        self._dcm_matrix = self._dcm_matrix + tempo_matrix
        
#         
#         print("Tempo matrix = %s\n" % (tempo_matrix))
#         print("DCM matrix = %s\n" % (self._dcm_matrix))
#         print("End of _update_matrix \n")
        
                   
    def _compute_dcm(self, mag_heading, acc_vec, mag_vec, gyr_vec, dt):
        """
           tentative implementation of DCM
        """     
        use_omega = True # OUTPUTMODE=1 use omega or gyro
        
        # update dcm matrix 
        self._matrix_update(gyr_vec, dt, acc_vec, use_omega)
        
        #NORMALIZE the matrix
        self._normalize_matrix(self._dcm_matrix)
        
        #DRIFT_CORRECTION                              
        self._drift_correction(acc_vec, self._omega_p, self._omega_i, mag_heading)
            
    
    
    def get_current_euler_angles(self):
        """
           return self._pitch, self._roll, self._yaw in radians
        """
        return (self._yaw, self._roll, self._pitch)
    
    def _project_to_inertial_frame(self, vec):
        """
           Inertial frame axes are the Earth fixed axis
           USe the dcm to do the conversion
        """
        return self._dcm_matrix.dot(vec)
    
        

#TO BE REMOVED           
def test_dcm():
    """
       test DCM
    """
    
    #Init setup (add arguments accel and magnetom
    #reset_sensor_fusion()
    
    #init values
    gyro_Dt     = 0.35 #integration time (Delta time between each measure
    
    gyro_vec    = np.array([26.00,45.00,1.00])
    accel_vec   = np.array([20.00,-255.00,76.00])
    mag_vec     = np.array([1342.00,-1055.00,779.00])
    
    omega_i = np.array([-0.000235,-0.000042,-0.000065])
    omega_p = np.array([-0.000862,-0.056547,0.064988])
    
    input_dm = np.matrix("0.925042 0.176073 0.336595 ;-0.371136 0.230046 0.899631 ;0.080968 -0.957119 0.278149")
     
    """  
     Note: roll and pitch, yaw are not set as in the software. Check how they are set
    """ 
   
    #starting values (they are used for every steps)
    #need to get the init values following the method
    pitch    = 0.11 #got it with acos(0.98)
    roll     = 0.06  #got it with acos(0.93)
    yaw      = -1.00
    
    accel_vec, mag_vec, gyro_vec = DCMizer.compensate_sensor_error(accel_vec, mag_vec, gyro_vec)
    
    print("After compensate sensor error\n")
    print("accel_vec= %s" % (accel_vec))
    print("mag_vec= %s" % (mag_vec))
    print("gyro_vec= %s" %(gyro_vec))

    #init values
    mag_heading = DCMizer.compass_heading(mag_vec, roll, pitch)
     
    print("mag_heading = %s\n" % (mag_heading))
    
    dcmizer = DCMizer(omega_p, omega_i, pitch, roll, yaw, input_dm)
          
    dcmizer._compute_dcm(mag_heading, mag_vec, gyro_Dt, gyro_vec, accel_vec)


def read_data(filename):
    """
       test DCM data by reading a file containing end product data and comparing with the data generated with this code
    """

    the_dir = "."
    file_path = "%s/etc/samples/%s" % (the_dir, filename)
    
    parser = Parser.CardSingleLineParser()

    first_run = True
    
    for line in open(file_path):
        if first_run :
            first_run = False
            # Loop through init debug lines (4 lines)
            vals = parser.parse_line(line)
            
            # Initialisation 
            iacc_vec    = [vals['ax'], vals['ay'], vals['az']]
            imag_vec    = [vals['mx'], vals['my'], vals['mz']]
            igyr_vec    = [vals['gx'], vals['gy'], vals['gz']]
            idt         = vals['dt'] #integration time (Delta time between each measure
            omega_p = 0
            omega_i = 0
            
            idcm_matrix, iyaw, ipitch, iroll = DCMizer.reset_sensor_fusion(iacc_vec, imag_vec)
            dcmizer = DCMizer(omega_p, omega_i, iyaw, ipitch, iroll, idcm_matrix)
            
            # compensate_sensor_error()
            iacc_vec, imag_vec, igyr_vec = dcmizer._compensate_sensor_error(iacc_vec, imag_vec, igyr_vec)
            
            # compass_heading()
            imag_heading = dcmizer._compass_heading(imag_vec)
            
            # Computer_dcm() (Matrix_update();   Normalize();   Drift_correction();   Euler_angles();
            dcmizer._compute_dcm(imag_heading, iacc_vec, imag_vec, igyr_vec, idt)
            
            #Project accel data from mobile to fixed inertial reference frame
            ifixed_acc_vec = dcmizer._project_to_inertial_frame(iacc_vec)
            
        else: 
            # Loop through normal steps
            vals = parser.parse_line(line)
                        
            # Extract values
            acc_vec     = [vals['ax'], vals['ay'], vals['az']]
            mag_vec     = [vals['mx'], vals['my'], vals['mz']]
            gyr_vec     = [vals['gx'], vals['gy'], vals['gz']]
            dt          = vals['dt'] #integration time (Delta time between each measure
            
            # compensate_sensor_error()
            acc_vec, mag_vec, gyr_vec = dcmizer._compensate_sensor_error(acc_vec, mag_vec, gyr_vec)
            
            # compass_heading()
            mag_heading = dcmizer._compass_heading(mag_vec)
            
            # Computer_dcm() (Matrix_update();   Normalize();   Drift_correction();   Euler_angles();
            dcmizer._compute_dcm(mag_heading, acc_vec, mag_vec, gyr_vec, dt)
            
            #Project accel data from mobile to fixed inertial reference frame
            fixed_acc_vec = dcmizer._project_to_inertial_frame(acc_vec)


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
#     np.set_printoptions(precision=7)
#     test_dcm()
    read_data("test_sample")


    
