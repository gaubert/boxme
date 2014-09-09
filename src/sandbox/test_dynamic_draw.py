'''
Created on Aug 28, 2014

@author: gaubert
'''

import numpy as np
import time
from matplotlib import pyplot as plt
 
plt.ion # set plot to animated
 
ydata = [0] * 50
ax1=plt.axes  
 
# make plot
line, = plt.plot(ydata)
plt.ylim([10,40])
 
# start data collection
all_data = ["0.0","1.1","2.2","3.3","4.4","5.5","6.6","7.7","8.8","9.9","10.10","0.0","1.1","2.2","3.3","4.4","5.5","6.6","7.7","8.8","9.9","10.10","0.0","1.1","2.2","3.3","4.4","5.5","6.6","7.7","8.8","9.9","10.10","0.0","1.1","2.2","3.3","4.4","5.5","6.6","7.7","8.8","9.9","10.10","0.0","1.1","2.2","3.3","4.4","5.5","6.6","7.7","8.8","9.9","10.10","0.0","1.1","2.2","3.3","4.4","5.5","6.6","7.7","8.8","9.9","10.10","0.0","1.1","2.2","3.3","4.4","5.5","6.6","7.7","8.8","9.9","10.10"]



while len(all_data) > 0:  
    # port and strip line endings
    data = all_data.pop(0)
    if len(data.split(".")) == 2:
        ymin = float(min(ydata))-10
        ymax = float(max(ydata))+10
        plt.ylim([ymin,ymax])
        ydata.append(data)
        del ydata[0]
        line.set_xdata(np.arange(len(ydata)))
        line.set_ydata(ydata)  # update the data
        plt.draw # update the plot
    
    print("sleep for 5s")
    time.sleep(0.05)
    
