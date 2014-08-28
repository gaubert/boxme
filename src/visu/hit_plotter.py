'''
Created on Aug 28, 2014

@author: gaubert
'''
import numpy as np
from collections import deque
from matplotlib import pyplot as plt

class AnalogData:

    def __init__(self, maxLen):
        self.ax = deque([0.0]*maxLen)
        self.ay = deque([0.0]*maxLen)
        self.az = deque([0.0]*maxLen)
        self.maxLen = maxLen
 
    def addToBuf(self, buf, val):
        if len(buf) < self.maxLen:
            buf.append(val)
        else:
            buf.pop()
            buf.appendleft(val)
            
    def add(self, data):
        assert(len(data) == 3)
        self.addToBuf(self.ax, data[0])
        self.addToBuf(self.ay, data[1])
        self.addToBuf(self.az, data[2])
        
    def add_x(self,val):
        """
        """
        self.addToBuf(self.ax, val)
    
    def add_y(self,val):
        """
        """
        self.addToBuf(self.ay, val)
    
    def add_z(self,val):
        """
        """
        self.addToBuf(self.az, val)

class Plotter:
    
    def __init__(self, analogData, min_s = None, max_s = None):
        """
           constructor
        """
        #set number of points limits
        if min_s == None:
            min_s = 0
        
        if max_s == None:
            max_s = len(analogData.ax)
        
        self._xdata = None
        self._ydata = None
        self._zdata = None
        
        self._sub_plot_x = None
        self._sub_plot_y = None
        self._sub_plot_z = None
        
        self._axline = None
        self._ayline = None
        self._azline = None
        
        self._create_plot()
            
    
    def _create_plot(self):
        """
           create the plot
        """
        
        self._ydata = [0] * 100
        self._xdata = [0] * 100
        self._zdata = [0] * 100
        
        plt.ion()
        fig = plt.figure(figsize=(20, 15))
            
        self._sub_plot_x = fig.add_subplot(311)
        self._sub_plot_x.set_autoscaley_on(False)
        self._sub_plot_x.set_ylim([-4096, 4096])
        self._sub_plot_x.set_title('x axis acceleration')
        self._axline, = plt.plot(self._xdata, color = "blue")
        
        self._sub_plot_y = fig.add_subplot(312)
        self._sub_plot_y.set_autoscaley_on(False)
        self._sub_plot_y.set_ylim([-4096, 4096])
        self._sub_plot_y.set_title('y axis acceleration')
        self._ayline, = plt.plot(self._ydata, color = "red")
        
        self._sub_plot_z = fig.add_subplot(313)
        self._sub_plot_z.set_autoscaley_on(False)
        self._sub_plot_z.set_ylim([-4096, 4096])
        self._sub_plot_z.set_title('z axis acceleration')
        self._azline, = plt.plot(self._ydata, color = "green")
        
        plt.ylim([10,40])
        
    def clean(self):
        """
           clean matplotlib resources
        """
        plt.close()
        
    def update(self, analogData):
        """
           Update the plot
        """
        
        x_ymin = float(min(self._xdata))-10
        x_ymax = float(max(self._xdata))+10
        self._sub_plot_x.set_ylim([x_ymin,x_ymax])
        self._xdata.append(analogData.ax.pop(0))
        del self._xdata[0]
        self._axline.set_xdata(np.arange(len(self._xdata)))
        self._axline.set_ydata(self._xdata)  # update the data
    
        
        y_ymin = float(min(self._ydata))-50
        y_ymax = float(max(self._ydata))+50
        self._sub_plot_y.set_ylim([y_ymin,y_ymax])
        self._ydata.append(analogData.ay.pop(0))
        del self._ydata[0]
        self._ayline.set_xdata(np.arange(len(self._ydata)))
        self._ayline.set_ydata(self._ydata)  # update the data
        
        z_ymin = float(min(self._zdata))-50
        z_ymax = float(max(self._zdata))+50
        self._sub_plot_z.set_ylim([z_ymin,z_ymax])
        self._zdata.append(analogData.az.pop(0))
        del self._zdata[0]
        self._azline.set_xdata(np.arange(len(self._zdata)))
        self._azline.set_ydata(self._zdata)  # update the data
    
        plt.draw() # update the plot


if __name__ == '__main__':
    
    # start data collection
    data_x = xrange(0,500)
    data_y = xrange(500,1000)
    data_z = xrange(1000,1500)
    
    analogData = AnalogData(5000)
    
    plotter = Plotter(analogData, 0, 2000)

    for i in xrange(0,500):
        analogData.add((data_x[i], data_y[i], data_z[i]))
    
    while len(analogData.ax) > 0:
        plotter.update(analogData)
     
    