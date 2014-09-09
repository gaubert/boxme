import re


class Parser(object):
    
    def __init__(self):
        """
           Do nothing for the moment
        """
        self._nb_line = 0
    
    def parse_line(self, line, loop_iter):
        """
           to be implemented in 
        """
        raise NotImplementedError("Please implement parse_line in child")
    


class CardSingleLineParser(Parser):
    """
       Class allowing to parse always the same line for the card in operational mode
                      
    """             
    LOOP_READSEN    = "##L#read_sens()#gDt:(?P<dt>.*)#T-acc:(?P<tacc>.*)#Am-Raw:(?P<amx>.*),(?P<amy>.*),(?P<amz>.*)#T-gyr:(?P<tgyr>.*)#Gm-Raw:(?P<gx>.*),(?P<gy>.*),(?P<gz>.*)#T-mag:(?P<tmag>.*)#Mm-Raw:(?P<mx>.*),(?P<my>.*),(?P<mz>.*)"
    LOOP_READSEN_RE = re.compile(LOOP_READSEN)
    
    def __init__(self):
        """
           constructor
        """
        super(CardSingleLineParser, self).__init__() 
        
    def parse_line(self, line):
        """
           try to parse a formatted line
        """
        matched = CardSingleLineParser.LOOP_READSEN_RE.match(line)
        if matched:
            try:
                tacc = float(matched.group('tacc'))
            except:
                print("Error cannot convert %s in float" % (matched.group('tacc')))
                tacc = -1
            
            ax        = float(matched.group('amx'))
            ay        = float(matched.group('amy'))
            az        = float(matched.group('amz'))
            
            try:
                tgyr = float(matched.group('tgyr'))
            except:
                print("Error cannot convert %s in float" % (matched.group('tgyr')))
                tgyr = -1
            
            gx        = float(matched.group('gx'))
            gy        = float(matched.group('gy'))
            gz        = float(matched.group('gz'))
            
            try:
                tmag = float(matched.group('tmag'))
            except:
                print("Error cannot convert %s in float" % (matched.group('tmag')))
                tmag = -1
            
            mx        = float(matched.group('mx'))
            my        = float(matched.group('my'))
            mz        = float(matched.group('mz'))
            
            try:
                dt = float(matched.group('dt'))
            except:
                print("Error cannot convert %s in float" % (matched.group('timestamp_tacc')))
                dt = -1
            
            data_elems =      {"tacc" : tacc,
                               "ax" : ax,
                               "ay" : ay,
                               "az" : az,
                               "tgyr" : tgyr,
                               "gx" : gx,
                               "gy" : gy,
                               "gz" : gz,
                               "tmag" : tmag,
                               "mx" : mx,
                               "my" : my,
                               "mz" : mz,
                               "dt" : dt
                               }
            self._nb_line += 1
        
        return data_elems


class DebugParser(Parser):
    """
       Class allowing to parse a card in debug mode with lots of information provided
    """
    
    SETUP_READSEN   = "##S#read_sens()#T-acc:(?P<itacc>.*)#Am-Raw:(?P<iamx>.*),(?P<iamy>.*),(?P<iamz>.*)#T-gyr:(?P<itgyr>.*)#Gm-Raw:(?P<igx>.*),(?P<igy>.*),(?P<igz>.*)#T-mag:(?P<itmag>.*)#Mm-Raw:(?P<imx>.*),(?P<imy>.*),(?P<imz>.*)"
    SETUP_READSEN_RE = re.compile(SETUP_READSEN)
    
    SETUP_COMPHEAD  = "##S#comp_head()#mag_x:(?P<imag_x>.*)#mag_y:(?P<imag_y>.*)#MAG_Heading:(?P<imag_head>.*)"
    SETUP_COMPHEAD_RE = re.compile(SETUP_COMPHEAD)
    
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
    
    
    LINE_SEQ = [ 'SETUP_READSEN', 'SETUP_COMPHEAD', 'SETUP_IYPR', 'SETUP_IDM', 'LOOP_TIM', 
                 'LOOP_READSEN', 'LOOP_COMP', 'LOOP_COMPHEAD', 'LOOP_UPDMAT',  'LOOP_NORM', 
                 'LOOP_DRIFT', 'LOOP_EULER']
    
    
    def __init__(self):
        """
           
        """
        self._cursor = 0
        
        self._dispatcher = {
                          'SETUP_READSEN'  : getattr(self, 'match_line_0'),
                          'SETUP_COMPHEAD' : getattr(self, 'match_line_1'),
                        }
        
    def test(self):
        
        self.parse_line("my line")
    
    def reset_cursor(self):
        """
           reinitialise the cursor
        """
    
    
    def match_line_0(self, line):
        
        expre_matched = DebugParser.SETUP_READSEN_RE.match(line)
       
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
        
        expre_matched = DebugParser.SETUP_COMPHEAD_RE.match(line)
       
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
        expre_matched = DebugParser.reg_exp.match(line)
       
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
        
        
    def parse_line(self, line):
        """
           try to parse a formatted line
        """
        try:
            #use the dispatcher to find the right line to call
            result = self._dispatcher[ self.LINE_SEQ[self._cursor] ](line)
        finally:
            # increment cursor whatever happen
            if self._cursor < len(self.LINE_SEQ)-1:
                self._cursor += 1
            else:
                #reset the cursor to parse a new line sequence
                self._cursor = 0
            
            self._nb_line += 1
                  
        return result
   
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


if __name__ == '__main__':
    
    parser = DebugParser()
    
    parser.test()