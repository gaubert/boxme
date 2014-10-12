/* This file is part of the Razor AHRS Firmware */

// DCM algorithm

/**************************************************/
void Normalize(void)
{
    if(DEBUG_P){
  Serial.print("##L#Norm()"); }

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
  if(DEBUG_P){
  Serial.print("#DCM_Matrix[0][0], [1][0], [2][0]:"); 
  Serial.print(DCM_Matrix[0][0],6); Serial.print(",");
  Serial.print(DCM_Matrix[1][0],6); Serial.print(",");
  Serial.print(DCM_Matrix[2][0],6); Serial.println();}
}

/**************************************************/
void Drift_correction(void)
{
    if(DEBUG_P){
  Serial.print("##L#Drift_corr()"); }

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
  if(DEBUG_P){  Serial.print("#errorCourse:"); 
  Serial.print(errorCourse,6); }
  
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  if(DEBUG_P){  Serial.print("#errorYaw[0],[1],[2]:"); 
  Serial.print(errorYaw[0],6); Serial.print(",");
  Serial.print(errorYaw[1],6); Serial.print(",");
  Serial.print(errorYaw[2],6); }
  
  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
  if(DEBUG_P){  Serial.print("#Scaled_Omega_P[0],[1],[2]:"); 
  Serial.print(Scaled_Omega_P[0],6); Serial.print(",");
  Serial.print(Scaled_Omega_P[1],6); Serial.print(",");
  Serial.print(Scaled_Omega_P[2],6); 
  }
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  if(DEBUG_P){  Serial.print("#Omega_P[0],[1],[2]:"); 
  Serial.print(Omega_P[0],6); Serial.print(",");
  Serial.print(Omega_P[1],6); Serial.print(",");
  Serial.print(Omega_P[2],6); }
  
  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
    if(DEBUG_P){Serial.print("#Scaled_Omega_I[0],[1],[2]:"); 
  Serial.print(Scaled_Omega_I[0],6); Serial.print(",");
  Serial.print(Scaled_Omega_I[1],6); Serial.print(",");
  Serial.print(Scaled_Omega_I[2],6); }
    
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
    if(DEBUG_P){Serial.print("#Omega_I[0],[1],[2]:"); 
  Serial.print(Omega_I[0],6); Serial.print(",");
  Serial.print(Omega_I[1],6); Serial.print(",");
  Serial.print(Omega_I[2],6); Serial.println();}
  
}

void Matrix_update(void)
{
    if(DEBUG_P){
  Serial.print("##L#Matr_updt()"); 
    }

  Gyro_Vector[0]=GYRO_SCALED_RAD(gyro[0]); //gyro x roll
  Gyro_Vector[1]=GYRO_SCALED_RAD(gyro[1]); //gyro y pitch
  Gyro_Vector[2]=GYRO_SCALED_RAD(gyro[2]); //gyro z yaw
  
  Accel_Vector[0]=accel[0];
  Accel_Vector[1]=accel[1];
  Accel_Vector[2]=accel[2];
    if(DEBUG_P){
  Serial.print("#omega_i[0],[1],[2]:");  
  Serial.print(Omega_I[0],6); Serial.print(",");
  Serial.print(Omega_I[1],6); Serial.print(",");
  Serial.print(Omega_I[2],6); Serial.print("])");  
  
  Serial.print("#omega_p[0],[1],[2]:");  
  Serial.print(Omega_P[0],6); Serial.print(",");
  Serial.print(Omega_P[1],6); Serial.print(",");
  Serial.print(Omega_P[2],6); Serial.print("])"); 
    }
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term
  
  
#if DEBUG__NO_DRIFT_CORRECTION == true // Do not use drift correction
  if(DEBUG_P){  Serial.print("#(Drift correction NOT used)#");}

  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Gyro_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Gyro_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Gyro_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Gyro_Vector[0];
  Update_Matrix[2][0]=-G_Dt*Gyro_Vector[1];
  Update_Matrix[2][1]=G_Dt*Gyro_Vector[0];
  Update_Matrix[2][2]=0;
    if(DEBUG_P){
  Serial.print("#G_Dt:");  Serial.print(G_Dt);
  Serial.print("#Gyro_Vector[0]:");  Serial.print(Gyro_Vector[0],6);
  Serial.print("#Gyro_Vector[1]:");  Serial.print(Gyro_Vector[1],6);
  Serial.print("#Gyro_Vector[2]:");  Serial.print(Gyro_Vector[2],6);}
#else // Use drift correction
  if(DEBUG_P){  Serial.print("#(Drift correction used)#");}

  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;
  if(DEBUG_P){  Serial.print("#G_Dt:");  Serial.print(G_Dt);  
  Serial.print("#Omega_Vector[0]:");  Serial.print(Omega_Vector[0],6);
  Serial.print("#Omega_Vector[1]:");  Serial.print(Omega_Vector[1],6);
  Serial.print("#Omega_Vector[2]:");  Serial.print(Omega_Vector[2],6);}
#endif


  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c
  if(DEBUG_P){
  Serial.print("#DM_after_matrix_mult():");}
  for(int x=0; x<3; x++) //Matrix Addition (update)
  {
    for(int y=0; y<3; y++)
    {  
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
       if(DEBUG_P){ Serial.print(DCM_Matrix[x][y],6); Serial.print(",");}
    } 
      if(DEBUG_P){Serial.print(";");}
  }

    if(DEBUG_P){Serial.println();}
}

void Euler_angles(void)
{
   if(DEBUG_P){ Serial.print("##L#Eul_ang()");}
  
  pitch = -asin(DCM_Matrix[2][0]);
  roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
  yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
    if(DEBUG_P){
  Serial.print("#pitch:");  Serial.print(pitch,6);
  Serial.print(",#roll:");  Serial.print(roll,6);
  Serial.print(",#yaw:");  Serial.print(yaw,6);
  
  Serial.print("#final_DM:");
    
  for(int x=0; x<3; x++) 
  {
    for(int y=0; y<3; y++)
    {
      Serial.print(DCM_Matrix[x][y],6);Serial.print(",");
    } 
    Serial.print(";");
  }
    }

}
