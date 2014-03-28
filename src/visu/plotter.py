'''
Created on Mar 28, 2014

@author: guillaume.aubert@gmail.com
'''
import re
import time

import numpy
from collections import deque
from matplotlib import pyplot as plt

# class that holds analog data for N latest samples
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

class AnalogDynamicPlotter:
    
    def __init__(self, analogData):
        # set plot to animated
        plt.ion() 
        self.axline, = plt.plot(analogData.ax)
        self.ayline, = plt.plot(analogData.ay)
        self.azline, = plt.plot(analogData.az)
        
        #set default plt dim
        plt.ylim([-4096, 4096])
 
    def update(self, analogData):
        self.axline.set_ydata(analogData.ax)
        self.ayline.set_ydata(analogData.ay)
        self.azline.set_ydata(analogData.az)
        plt.draw()
        
    def set_plt_dim(self, min_y, max_y):
        """
           set the min/max y dim for sizing the plot
        """
        plt.ylim([min_y, max_y])
    
    def draw(self):
        """
          draw a plot with the existing data
        """
        plt.draw()
        
    def show(self):
        """
        """
        plt.show()

class AnalogStaticPlotter:
    
    def __init__(self, analogData):
        # set plot to animated
        fig = plt.figure(figsize=(20, 15))
        sub_plot = fig.add_subplot(311)
        self.axline, = plt.plot(analogData.ax)
        sub_plot.set_title('x axis acceleration')
        sub_plot = fig.add_subplot(312)
        sub_plot.set_title('y axis acceleration')
        self.ayline, = plt.plot(analogData.ay, color = "red")
        sub_plot = fig.add_subplot(313)
        sub_plot.set_title('z axis acceleration')
        self.azline, = plt.plot(analogData.az, color = "green")
        
        
        # axis
        #y_axis = numpy.arange(-4096, 5010, 20)
        #plt.axis(y_axis)
        #set default plt dim
        #plt.ylim([-4096, 4096])
 
    def update(self, analogData):
        self.axline.set_ydata(analogData.ax)
        self.ayline.set_ydata(analogData.ay)
        #self.azline.set_ydata(analogData.az)
        plt.draw()
        
    def set_plt_dim(self, min_y, max_y):
        """
           set the min/max y dim for sizing the plot
        """
        #plt.ylim([min_y, max_y])
        
    
    def draw(self):
        """
          draw a plot with the existing data
        """
        plt.draw()
        
    def save_fig(self, filepath):
        """
           save fig
        """
        plt.savefig(filepath, bbox_inches='tight')
        
    def show(self):
        """
        """
        plt.show()
        
"""
plt.figure(1)
plt.subplot(211)
plt.plot(t1, f(t1), 'bo', t2, f(t2), 'k')

plt.subplot(212)
plt.plot(t2, np.cos(2*np.pi*t2), 'r--')
plt.show()
"""
        

#Timestamp:15773#A-Raw=62.00,263.00,101.00

line_expr = "Timestamp:(?P<timestamp>.*)#A-Raw=(?P<ax>.*),(?P<ay>.*),(?P<az>.*)"
line_re = re.compile(line_expr)


def update_min_max(elem, new_val):
    """
       update the min max vals if necessary
       elem is a { "min": v1, "max": v2 }
    """
    if new_val < elem["min"]:
        elem["min"] = new_val
    
    if new_val > elem["max"]:
        elem["max"] = new_val
    
    return elem

def read_sample_data(file_path):
    """
       Read the data sample
    """
    the_file = open(file_path)
    
    data_elems = []
    
    nb_line = 0
    
    #min_max_x = { "min" : 4096.00, "max": -4096.00, }
    #min_max_y = { "min" : 4096.00, "max": -4096.00, }
    #min_max_z = { "min" : 4096.00, "max": -4096.00, }
    
    min_max_all_axes = { "min" : 4096.00, "max": -4096.00, }
    
    for line in the_file:
        #print("The line = %s" % (line))
        
        matched = line_re.match(line)
        if matched:
            timestamp = float(matched.group('timestamp'))
            
            ax        = float(matched.group('ax'))
            
            #min_max_all_axes = update_min_max(min_max_all_axes, ax)
             
            ay        = float(matched.group('ay'))
            
            min_max_all_axes = update_min_max(min_max_all_axes, ay)
            
            az        = float(matched.group('az'))
            
            #min_max_all_axes = update_min_max(min_max_all_axes, az)
            
            data_elems.append({"ts" : timestamp,
                               "ax" : ax,
                               "ay" : ay,
                               "az" : az
                               })
            nb_line += 1
            
            #print("ts = %s, ax = %s, ay = %s, az = %s" % (timestamp, ax, ay, az))
        
    print("nb lines in file: %d " % (nb_line))
    print("min all axes = %f, max all axes = %f" %(min_max_all_axes['min'], min_max_all_axes['max']))
    
    return data_elems, min_max_all_axes

def test_1():
    """
       play fix set of data
    """
  

    analogData = AnalogData(100)
    analogPlot = AnalogPlotter(analogData)
 
    print 'plotting data...'
  
    data_elems = [(10, 30, 40) , (20,40,50), (30,50,60), (10, 30, 20), (23, 52, 41), (58,70,24), (150, 100, 250), (320, 123, 50)]
  
  
    while True:
        try:
            #line = ser.readline()
            for data in data_elems:
                if (len(data) == 3):
                    analogData.add(data)
                    analogPlot.update(analogData)
        except KeyboardInterrupt:
            print 'exiting'
            break 

def test_2():
    """
       read a sample file and play it in loop
    """
    the_dir = "."
    file_path = "%s/etc/data_sample1" % (the_dir)
    
    analogData = AnalogData(5000)
    analogPlot = AnalogStaticPlotter(analogData)
    
    data_elems, min_max = read_sample_data(file_path)
    
    while True:
        try:
            #line = ser.readline()
            for data in data_elems:
                analogData.add((data['ax'], data['ay'], data['az']))
                analogPlot.update(analogData)
        except KeyboardInterrupt:
            print 'exiting'
            break 

def test_sample1():
    """
       draw a plot once from a sample file
    """    
    the_dir = "."
    file_path = "%s/etc/data_sample1" % (the_dir)
    
    analogData = AnalogData(1962)
    data_elems, min_max = read_sample_data(file_path)
    
    
    for data in data_elems:
        analogData.add((data['ax'], data['ay'], data['az']))
    
    analogPlot = AnalogStaticPlotter(analogData)
    analogPlot.set_plt_dim(min_max["min"] - 100, min_max["max"] + 100)
    
    try:
        analogPlot.save_fig("/tmp/sample_plot.png")
        analogPlot.show()
        #time.sleep(20)
    except KeyboardInterrupt:
        print 'existing'
        


if __name__ == '__main__':
    #test1()
    #test_2()
    test_sample1()