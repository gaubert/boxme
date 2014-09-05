import re

class CardOutParser(object):
    
    LINEEXPR    = "##T-step: old_ts#(?P<tstep_old>.*), new_ts#(?P<tstep_new>.*), gyro_Dt#(?P<dt>.*)"
    lre_timing  = re.compile(LINEEXPR)
    
    LINEEXPR    = "#T-acc#(?P<timestamp_tacc>.*)#Am-Raw#(?P<r_ax>.*),(?P<r_ay>.*),(?P<r_az>.*)#T-gyr#(?P<timestamp_tgyr>.*)#Gm-Raw#(?P<g_ax>.*),(?P<g_ay>.*),(?P<g_az>.*)#T-mag#(?P<timestamp_tmag>.*)#Mm-Raw#(?P<m_ax>.*),(?P<m_ay>.*),(?P<m_az>.*)"
    lre_data  = re.compile(LINEEXPR)
    
    LINEEXPR    = "#Am-comp#(?P<r_acx>.*),(?P<r_acy>.*),(?P<r_acz>.*)#Gm-comp#(?P<g_acx>.*),(?P<g_acy>.*),(?P<g_acz>.*)#Mm-comp#(?P<m_acx>.*),(?P<m_acy>.*),(?P<m_acz>.*)"
    lre_compensate  = re.compile(LINEEXPR)
    
    LINEEXPR    = "#mag_x#(?P<max_x>.*)#mag_y#(?P<mag_y>.*)#mag_head#(?P<mag_head>.*)"
    lre_compass  = re.compile(LINEEXPR)
    
    LINEEXPR    = "#T-acc:(?P<timestamp_tacc>.*)#Am-Raw#(?P<r_ax>.*),(?P<r_ay>.*),(?P<r_az>.*)#T-gyr#(?P<timestamp_tgyr>.*)#Gm-Raw#(?P<g_ax>.*),(?P<g_ay>.*),(?P<g_az>.*)#T-mag#(?P<timestamp_tmag>.*)#Mm-Raw#(?P<m_ax>.*),(?P<m_ay>.*),(?P<m_az>.*)"
    lre_matrix  = re.compile(LINEEXPR)
    
    LINEEXPR    = "#T-acc:(?P<timestamp_tacc>.*)#Am-Raw#(?P<r_ax>.*),(?P<r_ay>.*),(?P<r_az>.*)#T-gyr#(?P<timestamp_tgyr>.*)#Gm-Raw#(?P<g_ax>.*),(?P<g_ay>.*),(?P<g_az>.*)#T-mag#(?P<timestamp_tmag>.*)#Mm-Raw#(?P<m_ax>.*),(?P<m_ay>.*),(?P<m_az>.*)"
    lre_norm  = re.compile(LINEEXPR)
    
    LINEEXPR    = "#T-acc:(?P<timestamp_tacc>.*)#Am-Raw#(?P<r_ax>.*),(?P<r_ay>.*),(?P<r_az>.*)#T-gyr#(?P<timestamp_tgyr>.*)#Gm-Raw#(?P<g_ax>.*),(?P<g_ay>.*),(?P<g_az>.*)#T-mag#(?P<timestamp_tmag>.*)#Mm-Raw#(?P<m_ax>.*),(?P<m_ay>.*),(?P<m_az>.*)"
    lre_drift  = re.compile(LINEEXPR)
    
    LINEEXPR    = "#T-acc:(?P<timestamp_tacc>.*)#Am-Raw#(?P<r_ax>.*),(?P<r_ay>.*),(?P<r_az>.*)#T-gyr#(?P<timestamp_tgyr>.*)#Gm-Raw#(?P<g_ax>.*),(?P<g_ay>.*),(?P<g_az>.*)#T-mag#(?P<timestamp_tmag>.*)#Mm-Raw#(?P<m_ax>.*),(?P<m_ay>.*),(?P<m_az>.*)"
    lre_euler = re.compile(LINEEXPR)
    
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
