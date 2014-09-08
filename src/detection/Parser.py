import re

class CardOutParser(object):
    
    SETUP_READSEN   = "##S#read_sens()#T-acc:(?P<itacc>.*)#Am-Raw:(?P<iamx>.*),(?P<iamy>.*),(?P<iamz>.*)#T-gyr:(?P<itgyr>.*)#Gm-Raw:(?P<igx>.*),(?P<igy>.*),(?P<igz>.*)#T-mag:(?P<itmag>.*)#Mm-Raw:(?P<imx>.*),(?P<imy>.*),(?P<imz>.*)"
    SETUP_COMPHEAD  = "##S#comp_head()#mag_x:(?P<imag_x>.*)#mag_y:(?P<imag_y>.*)#MAG_Heading:(?P<imag_head>.*)"
    SETUP_IYPR      = "##S#iYPR()#pitch:(?P<ipitch>.*)#roll:(?P<iroll>.*)#yaw:(?P<iyaw>.*)"
    SETUP_IDM       = "##S#init_rot_mat:(?P<iDM00>.*),(?P<iDM01>.*),(?P<iDM02>.*),;(?P<iDM10>.*),(?P<iDM11>.*),(?P<iDM12>.*),;(?P<iDM20>.*),(?P<iDM21>.*),(?P<iDM22>.*),;"
  
    LOOP_TIM        = "##L#old_ts:(?P<tstep_old>.*)#new_ts:(?P<tstep_new>.*)#gyro_Dt:(?P<dt>.*)"
    LOOP_READSEN    = "##L#read_sens()#T-acc:(?P<tacc>.*)#Am-Raw:(?P<amx>.*),(?P<amy>.*),(?P<amz>.*)#T-gyr:(?P<tgyr>.*)#Gm-Raw:(?P<gx>.*),(?P<gy>.*),(?P<gz>.*)#T-mag:(?P<tmag>.*)#Mm-Raw:(?P<mx>.*),(?P<my>.*),(?P<mz>.*)"
    LOOP_COMP       = "##L#Comp_error()#(NOTcompensate magn error)##AFTER##acc[0,1,2]:(?P<acc_vec0>.*),(?P<acc_vec1>.*),(?P<acc_vec2>.*)#magnetom[0,1,2]:(?P<mag_vec0>.*),(?P<mag_vec1>.*),(?P<mag_vec2>.*)#gyro[0,1,2]:(?P<gyr_vec0>.*),(?P<gyr_vec1>.*),(?P<gyr_vec2>.*)"
    LOOP_COMPHEAD   = "##L#comp_head()#mag_x:(?P<mag_x>.*)#mag_y:(?P<mag_y>.*)#MAG_Heading:(?P<mag_head>.*)"
    LOOP_UPDMAT     = "##L#Matr_updt()#omega_i[0],[1],[2]:(?P<omega_i0>.*),(?P<omega_i1>.*),(?P<omega_i2>.*)#omega_p[0],[1],[2]:(?P<omega_p0>.*),(?P<omega_p1>.*),(?P<omega_p2>.*)#(Drift correction used)##G_Dt:(?P<dt>.*)#Omega_Vector[0]:(?P<omega_vec0>.*)#Omega_Vector[1]:(?P<omega_vec1>.*)#Omega_Vector[2]:(?P<omega_vec2>.*)#DM_after_matrix_mult():(?P<tDM00>.*),(?P<tDM01>.*),(?P<tDM02>.*),;(?P<tDM10>.*),(?P<tDM11>.*),(?P<tDM12>.*),;(?P<tDM20>.*),(?P<tDM21>.*),(?P<tDM22>.*),;"
    LOOP_NORM       = "##L#Norm()#DCM_Matrix[0][0], [1][0], [2][0]:(?P<DM00>.*),(?P<DM01>.*),(?P<DM02>.*),;(?P<DM10>.*),(?P<DM11>.*),(?P<DM12>.*),;(?P<DM20>.*),(?P<DM21>.*),(?P<DM22>.*),;"
    LOOP_DRIFT      = "##L#Drift_corr()#errorCourse:(?P<err_course>.*)#errorYaw[0],[1],[2]:(?P<errorYaw0>.*),(?P<errorYaw1>.*),(?P<errorYaw2>.*)#Scaled_Omega_P[0],[1],[2]:(?P<Scaled_Omega_P0>.*),(?P<Scaled_Omega_P1>.*),(?P<Scaled_Omega_P2>.*)#Omega_P[0],[1],[2]:(?P<Omega_P0>.*),(?P<Omega_P1>.*),(?P<Omega_P2>.*)#Scaled_Omega_I[0],[1],[2]:(?P<Scaled_Omega_I0>.*),(?P<Scaled_Omega_I1>.*),(?P<Scaled_Omega_I2>.*)#Omega_I[0],[1],[2]:(?P<Omega_I0>.*),(?P<Omega_I1>.*),(?P<Omega_I2>.*)"
    LOOP_EULER      = "##L#Eul_ang()#pitch:(?P<pitch>.*)#roll:(?P<roll>.*)#yaw:(?P<yaw>.*)#final_DM:(?P<fDM00>.*),(?P<fDM01>.*),(?P<fDM02>.*),;(?P<fDM10>.*),(?P<fDM11>.*),(?P<fDM12>.*),;(?P<fDM20>.*),(?P<fDM21>.*),(?P<fDM22>.*)"
    
    dispatch = {
                0: match_line_0,
                1: match_line_1,
                2: match_line_2
    }
    
    def __init__(self):
        """
           
        """
        self._nb_line = 0
        
    def match_line_0(self, reg_exp, line):
        
        expre_matched = CardOutParser.reg_exp.match(line)
       
        if expre_matched:
            ax        = float(expre_matched.group('r_ax'))
            ay        = float(expre_matched.group('r_ay'))
            az        = float(expre_matched.group('r_az'))
            
            data_elems =      {
                               "ax" : ax,
                               "ay" : ay,
                               "az" : az,
                               }
            
        return data_elems

    def match_line_1(self, arg, line):
        
        expre_matched = CardOutParser.reg_exp.match(line)
       
        if expre_matched:
            ax        = float(expre_matched.group('r_ax'))
            ay        = float(expre_matched.group('r_ay'))
            az        = float(expre_matched.group('r_az'))
            
            data_elems =      {
                               "ax" : ax,
                               "ay" : ay,
                               "az" : az,
                               }
            
        return data_elems
   
    def match_line_2(self, arg, line):
        expre_matched = CardOutParser.reg_exp.match(line)
       
        if expre_matched:
            ax        = float(expre_matched.group('r_ax'))
            ay        = float(expre_matched.group('r_ay'))
            az        = float(expre_matched.group('r_az'))
            
            data_elems =      {
                               "ax" : ax,
                               "ay" : ay,
                               "az" : az,
                               }
            
        return data_elems
        
        
    def parse_line(self, line, loop_iter):
        """
           try to parse a formatted line
        """
        debug_types = {0: 'SETUP_READSEN', 
                       1: 'SETUP_COMPHEAD', 
                       2: 'SETUP_IYPR', 
                       3: 'SETUP_IDM', 
                       4: 'LOOP_TIM', 
                       5: 'LOOP_READSEN',
                       6: 'LOOP_COMP', 
                       7: 'LOOP_COMPHEAD', 
                       8: 'LOOP_UPDMAT', 
                       9: 'LOOP_NORM', 
                       10: 'LOOP_DRIFT', 
                       11: 'LOOP_EULER'}
        
        reg_exp = re.compile(debug_types[loop_iter])
        map(self.dispatch[loop_iter](reg_exp, line))
        

   
    def nb_lines(self):
        """
           return nb lines parsed
        """
        return self._nb_line
    
    def reset(self):
        """
           Reset parser
        """
        self._nb_line = 0
