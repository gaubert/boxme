'''
Created on Aug 28, 2014

@author: gaubert
'''
import numpy as np
import time
from matplotlib import pyplot as plt

if __name__ == '__main__':
    
    ydata = [0] * 50
    xdata = [0] * 50
    zdata = [0] * 50
    
    plt.ion()
    
    fig = plt.figure(figsize=(20, 15))
        
    sub_plot_x = fig.add_subplot(311)
    sub_plot_x.set_autoscaley_on(False)
    sub_plot_x.set_ylim([-4096, 4096])
    axline, = plt.plot(xdata, color = "blue")
    sub_plot_x.set_title('x axis acceleration')
    
    sub_plot_y = fig.add_subplot(312)
    sub_plot_y.set_autoscaley_on(False)
    sub_plot_y.set_ylim([-4096, 4096])
    sub_plot_y.set_title('y axis acceleration')
    ayline, = plt.plot(ydata, color = "red")
    sub_plot_z = fig.add_subplot(313)
    sub_plot_z.set_autoscaley_on(False)
    sub_plot_z.set_ylim([-4096, 4096])
    sub_plot_z.set_title('z axis acceleration')
    azline, = plt.plot(ydata, color = "green")
    
    plt.ylim([10,40])
 
    # start data collection
    all_data_x = ["0.0","1.1","2.2","3.3","4.4","5.5","6.6","7.7","8.8","9.9","10.10","0.0","1.1","2.2","3.3","4.4","5.5","6.6","7.7","8.8","9.9","10.10","0.0","1.1","2.2","3.3","4.4","5.5","6.6","7.7","8.8","9.9","10.10"]

    all_data_y = ["100.100", "110.110", "120.120", "130.130", "140.140", "150.150", "160.160", "170.170", "180.180", "190.190", "200.200","100.100", "110.110", "120.120", "130.130", "140.140", "150.150", "160.160", "170.170", "180.180", "190.190", "200.200"]

    all_data_z = ["100.100", "110.110", "120.120", "130.130", "140.140", "150.150", "160.160", "170.170", "180.180", "190.190", "200.200","100.100", "110.110", "120.120", "130.130", "140.140", "150.150", "160.160", "170.170", "180.180", "190.190", "200.200"]

    while len(all_data_x) > 0 or len(all_data_y) > 0 or len(all_data_z) > 0:  
        # port and strip line endings
        #data = all_data_x.pop(0)
        
        if len(all_data_x) > 0:
            x_ymin = float(min(xdata))-10
            x_ymax = float(max(xdata))+10
            sub_plot_x.set_ylim([x_ymin,x_ymax])
            xdata.append(all_data_x.pop(0))
            del xdata[0]
            axline.set_xdata(np.arange(len(xdata)))
            axline.set_ydata(xdata)  # update the data
        
        if len(all_data_y) > 0:
            y_ymin = float(min(ydata))-50
            y_ymax = float(max(ydata))+50
            sub_plot_y.set_ylim([y_ymin,y_ymax])
            ydata.append(all_data_y.pop(0))
            del ydata[0]
            ayline.set_xdata(np.arange(len(ydata)))
            ayline.set_ydata(ydata)  # update the data
        
        if len(all_data_z) > 0:
            z_ymin = float(min(zdata))-50
            z_ymax = float(max(zdata))+50
            sub_plot_z.set_ylim([z_ymin,z_ymax])
            zdata.append(all_data_z.pop(0))
            del zdata[0]
            azline.set_xdata(np.arange(len(ydata)))
            azline.set_ydata(ydata)  # update the data
        
        plt.draw() # update the plot
    
        print("sleep for 5s")
        time.sleep(0.01)
    