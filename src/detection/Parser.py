import re

class CardOutParser(object):
    
    SETUP_READSEN   = "##S#read_sens()#T-acc:(?P<itacc>.*)#Am-Raw:(?P<iamx>.*),(?P<iamy>.*),(?P<iamz>.*)#T-gyr:(?P<itgyr>.*)#Gm-Raw:(?P<igx>.*),(?P<igy>.*),(?P<igz>.*)#T-mag:(?P<itmag>.*)#Mm-Raw:(?P<imx>.*),(?P<imy>.*),(?P<imz>.*)"
    lre_sreadsen    = re.compile(SETUP_READSEN)
    
    SETUP_COMPHEAD  = "##S#comp_head()#mag_x:(?P<imag_x>.*)#mag_y:(?P<imag_y>.*)#MAG_Heading:(?P<imag_head>.*)"
    lre_scomphead   = re.compile(SETUP_COMPHEAD)
    
    SETUP_IYPR      = "##S#iYPR()#pitch:(?P<ipitch>.*)#roll:(?P<iroll>.*)#yaw:(?P<iyaw>.*)"
    lre_siypr       = re.compile(SETUP_IYPR)
    
    SETUP_IDM       = "##S#init_rot_mat:(?P<iDM00>.*),(?P<iDM01>.*),(?P<iDM02>.*),;(?P<iDM10>.*),(?P<iDM11>.*),(?P<iDM12>.*),;(?P<iDM20>.*),(?P<iDM21>.*),(?P<iDM22>.*),;"
    lre_sidm        = re.compile(SETUP_IDM)
    
    LOOP_TIM        = "##L#old_ts:(?P<tstep_old>.*)#new_ts:(?P<tstep_new>.*)#gyro_Dt:(?P<dt>.*)"
    lre_ltiming     = re.compile(LOOP_TIM)
    
    LOOP_READSEN    = "##L#read_sens()#T-acc:(?P<tacc>.*)#Am-Raw:(?P<amx>.*),(?P<amy>.*),(?P<amz>.*)#T-gyr:(?P<tgyr>.*)#Gm-Raw:(?P<gx>.*),(?P<gy>.*),(?P<gz>.*)#T-mag:(?P<tmag>.*)#Mm-Raw:(?P<mx>.*),(?P<my>.*),(?P<mz>.*)"
    lre_ldata       = re.compile(LOOP_READSEN)
    
    LOOP_COMP       = "##L#Comp_error()#(NOTcompensate magn error)##AFTER##acc[0,1,2]:(?P<acc_vec0>.*),(?P<acc_vec1>.*),(?P<acc_vec2>.*)#magnetom[0,1,2]:(?P<mag_vec0>.*),(?P<mag_vec1>.*),(?P<mag_vec2>.*)#gyro[0,1,2]:(?P<gyr_vec0>.*),(?P<gyr_vec1>.*),(?P<gyr_vec2>.*)"
    lre_lcompensate = re.compile(LOOP_COMP)
    
    LOOP_COMPHEAD   = "##L#comp_head()#mag_x:(?P<mag_x>.*)#mag_y:(?P<mag_y>.*)#MAG_Heading:(?P<mag_head>.*)"
    lre_lcomphead   = re.compile(LOOP_COMPHEAD)
    
    LOOP_UPDMAT     = "##L#Matr_updt()#omega_i[0],[1],[2]:(?P<omega_i0>.*),(?P<omega_i1>.*),(?P<omega_i2>.*)#omega_p[0],[1],[2]:(?P<omega_p0>.*),(?P<omega_p1>.*),(?P<omega_p2>.*)#(Drift correction used)##G_Dt:(?P<dt>.*)#Omega_Vector[0]:(?P<omega_vec0>.*)#Omega_Vector[1]:(?P<omega_vec1>.*)#Omega_Vector[2]:(?P<omega_vec2>.*)#DM_after_matrix_mult():(?P<tDM00>.*),(?P<tDM01>.*),(?P<tDM02>.*),;(?P<tDM10>.*),(?P<tDM11>.*),(?P<tDM12>.*),;(?P<tDM20>.*),(?P<tDM21>.*),(?P<tDM22>.*),;"
    lre_lmatrix     = re.compile(LOOP_UPDMAT)
    
    LOOP_NORM       = "##L#Norm()#DCM_Matrix[0][0], [1][0], [2][0]:(?P<DM00>.*),(?P<DM01>.*),(?P<DM02>.*),;(?P<DM10>.*),(?P<DM11>.*),(?P<DM12>.*),;(?P<DM20>.*),(?P<DM21>.*),(?P<DM22>.*),;"
    lre_Lnorm       = re.compile(LOOP_NORM)
    
    LOOP_DRIFT      = "##L#Drift_corr()#errorCourse:(?P<err_course>.*)#errorYaw[0],[1],[2]:(?P<errorYaw0>.*),(?P<errorYaw1>.*),(?P<errorYaw2>.*)#Scaled_Omega_P[0],[1],[2]:(?P<Scaled_Omega_P0>.*),(?P<Scaled_Omega_P1>.*),(?P<Scaled_Omega_P2>.*)#Omega_P[0],[1],[2]:(?P<Omega_P0>.*),(?P<Omega_P1>.*),(?P<Omega_P2>.*)#Scaled_Omega_I[0],[1],[2]:(?P<Scaled_Omega_I0>.*),(?P<Scaled_Omega_I1>.*),(?P<Scaled_Omega_I2>.*)#Omega_I[0],[1],[2]:(?P<Omega_I0>.*),(?P<Omega_I1>.*),(?P<Omega_I2>.*)"
    lre_ldrift       = re.compile(LOOP_DRIFT)
    
    LOOP_EULER      = "##L#Eul_ang()#pitch:(?P<pitch>.*)#roll:(?P<roll>.*)#yaw:(?P<yaw>.*)#final_DM:(?P<fDM00>.*),(?P<fDM01>.*),(?P<fDM02>.*),;(?P<fDM10>.*),(?P<fDM11>.*),(?P<fDM12>.*),;(?P<fDM20>.*),(?P<fDM21>.*),(?P<fDM22>.*)"
    lre_leuler       = re.compile(LOOP_EULER)
    
    def __init__(self):
        """
           
        """
        self._nb_line = 0
        
    def parse_line(self, line):
        """
           try to parse a formatted line
        """
        timing_matched = CardOutParser.lre_timing.match(line)
        data_matched = CardOutParser.lre_data.match(line)
        debug_compensate_matched = CardOutParser.lre_compensate.match(line)
        debug_compass_matched = CardOutParser.lre_compass.match(line)
        debug_matrix_matched = CardOutParser.lre_matrix.match(line)
        debug_norm_matched = CardOutParser.lre_norm.match(line)
        debug_drift_matched = CardOutParser.lre_drift.match(line)
        debug_euler_matched = CardOutParser.lre_euler.match(line)
        
        
        if data_matched:
            #Timestamp
            try:
                timestamp = float(data_matched.group('timestamp_tacc'))
            except:
                print("Error cannot convert %s in float" % (data_matched.group('timestamp_tacc')))
                timestamp = -1
            #ax
            ax        = float(data_matched.group('r_ax'))
            #ay 
            ay        = float(data_matched.group('r_ay'))
            #az
            az        = float(data_matched.group('r_az'))
            
            #gx
            gx        = float(data_matched.group('r_gx'))
            #gy 
            gy        = float(data_matched.group('r_gy'))
            #gz
            gz        = float(data_matched.group('r_gz'))
            
            #mx
            mx        = float(data_matched.group('r_mx'))
            #my 
            my        = float(data_matched.group('r_my'))
            #mz
            mz        = float(data_matched.group('r_mz'))
            
            data_elems =      {"ts" : timestamp,
                               "ax" : ax,
                               "ay" : ay,
                               "az" : az,
                               "gx" : gx,
                               "gy" : gy,
                               "gz" : gz,
                               "mx" : mx,
                               "my" : my,
                               "mz" : mz
                               }
            self._nb_line += 1
        
        return data_elems
    
   
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
