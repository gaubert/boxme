#!/usr/bin/env python

'''
 DCM and plotting of the Gyro/Accel data from a Motion+ & Nunchuck

 History     : 21/1/2010 - V 0.1 - First Release

 License     : Copyright 2010 Simon Wood <simon@mungewell.org>

               This example python script is free software: you can
               redistribute it and/or modify it under the terms of the GNU
               General Public License as published by the Free Software
               Foundation, either version 2 of the License, or (at your
               option) any later version.

               The example python script is distributed in the hope that
               it will be useful, but WITHOUT ANY WARRANTY; without even the
               implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
               PURPOSE.  See the GNU General Public License for more details.

               You should have received a copy of the GNU General Public
               License. If not, see <http://www.gnu.org/licenses/>.
'''

import sys
import time
from math import sqrt
from numpy import *

# http://rtgraph.sourceforge.net/
import gtk
import rtgraph, time, math

# Note on axis
# acc[0] - X = +ve acceleration toward nose
# acc[1] - Y = +ve acceleration toward right
# acc[2] - Z = +ve acceleration toward feet

acc = zeros(3)
accv = zeros(3)
accl = 0

# Note on axis
# gyro[0] - X = roll, +ve right wing up (red)
# gyro[1] - Y = pitch, +ve nose up (green)
# gyro[2] - Z = yaw, +ve nose right (blue)

gyro = zeros(3)
timestamp = 0

offsets = zeros(3)
rotation = zeros(3)
euler = zeros(3)

error = zeros(3)
error_d = zeros(3)
trim = zeros(3)

dcm = mat([     [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0] ])

x_vec = array([1.0, 0.0, 0.0])
y_vec = array([0.0, 1.0, 0.0])
z_vec = array([0.0, 0.0, 1.0])

last = zeros(3)
time_delta = 0
nexttime = 0

# Linear plot of rotations
class PlotRotation(rtgraph.Channel):
    def __init__(self, input, color):
        rtgraph.Channel.__init__(self, color=color)
        self.input = input

    def getValue(self):
        global rotation
        return rotation[self.input]

# Vector plot of Pitch, Roll and Yaw
class PlotRotationVector(rtgraph.Channel):
    def __init__(self, input, color):
        rtgraph.Channel.__init__(self, color=color)
        self.input = input

    def getValue(self):
    global euler
    return (1, -euler[self.input])

# Linear plot of raw gyro data
class PlotGyro(rtgraph.Channel):
    def __init__(self, input, color):
        rtgraph.Channel.__init__(self, color=color)
        self.input = input

    def getValue(self):
    global gyro
    return gyro[self.input]

# Linear plot of gyro error 
class PlotError(rtgraph.Channel):
    def __init__(self, input, color):
        rtgraph.Channel.__init__(self, color=color)
        self.input = input

    def getValue(self):
    global error
    return error[self.input]

# 3D vector plot of accelerometers
class PlotAccv(rtgraph.Channel):
    def __init__(self, input, color):
        rtgraph.Channel.__init__(self, color=color)
        self.input = input

    def getValue(self):
    global accv
    return (accv + ones(3))/2

# Linear plot of raw acc data, plus total acceleration size
class PlotAcc(rtgraph.Channel):
    def __init__(self, input, color):
        rtgraph.Channel.__init__(self, color=color)
        self.input = input

    def getValue(self):
    global acc, accv, accl, gyro, timestamp
    global offsets, rotation, euler
    global error, error_d, trim
    global dcm
    global last, time_delta, nexttime

    if (self.input == 0):
        if (time.time() > nexttime):
            data = sys.stdin.readline()
        
            data_list = data.split(",")

            # Compute the data update rate
            time_delta = eval(data_list[0]) - timestamp
            timestamp = eval(data_list[0])

            nexttime = time.time() + 0.005

            # store Gyro values (wrong order to suit my config)
            gyro[0] = eval(data_list[3])    # roll
            gyro[1] = -eval(data_list[1])    # pitch
            gyro[2] = -eval(data_list[2])    # yaw

            # store Acc values if they exist
            if len(data_list) > 4:
                acc[0] = -eval(data_list[4])
                acc[1] = -eval(data_list[5])
                acc[2] = -eval(data_list[6])

                accl = sqrt(dot(acc, acc))
                accv = acc / accl

            # clear error integral
            if data_list[len(data_list)-1] == ' CZ  \n':
                error_d = zeros(3)

            # calibrate Gyro offset rates
            if data_list[len(data_list)-1] == ' C-  \n':
                offsets = gyro.copy()

            # reset rotations on request
            if data_list[len(data_list)-1] == ' -Z  \n':
                rotation[:] = 0

                # reset DCM to zero
                dcm = mat([ [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0] ])

            # wait 2s before processing DCM
            if timestamp >= 2 :
                delta = zeros(3)

                # apply gyro rate correction
                rates = gyro - offsets

                # Simple integration for rotations
                # delta = (rates * time_delta)

                # Trapezoid integration gives degrees
                for n in [0,1,2]:
                    if (last[n] * rates[n]) < 0:
                        # different signs
                        delta[n] = (last[n] * time_delta) - (0.5 * time_delta * (last[n] - rates[n]))
                    else:
                        if (last[n]**2 > rates[n]**2):
                            square = rates[n] * time_delta
                            triangle = (0.5 * time_delta * (last[n] - rates[n]))
                        else:
                            square = last[n] * time_delta
                            triangle = (0.5 * time_delta * (rates[n] - last[n]))

                        delta[n] = square + triangle

                # add in delta (degrees)
                rotation = rotation + delta

                # check limits and perform wrap-around
                while (rotation.min() <= -180):
                    rotation = rotation + (360*(rotation <= -180))

                while (rotation.max() > 180):
                    rotation = rotation - (360*(rotation > 180))

                # record gyro values for next round
                last = rates

                # DCM starts here -----------------------------------------
                omega = ((gyro - offsets) * pi/180) - error

                # rotate dcm by delta 
                update = array([ [0, -time_delta * omega[2], time_delta * omega[1] ],
                    [time_delta * omega[2], 0, time_delta * omega[0]],
                    [-time_delta * omega[1], time_delta * omega[0], 0]])

                dcm = dcm + (dcm * update)

                # renormalisation of rows
                renorm = dcm[0] * (dcm[1].T)

                Xo = dcm[0] - multiply(renorm/2, dcm[1])
                Yo = dcm[1] - multiply(renorm/2, dcm[0])

                # Cross product
                Zo = mat( [(Xo[0,1] * Yo[0,2]) - (Xo[0,2] * Yo[0,1]),
                    (Xo[0,2] * Yo[0,0]) - (Xo[0,0] * Yo[0,2]),
                    (Xo[0,0] * Yo[0,1]) -(Xo[0,1] * Yo[0,0])])

                dcm = concatenate( (
                    multiply((3 - Xo * Xo.T)/2, Xo),
                    multiply((3 - Yo * Yo.T)/2, Yo),
                    multiply((3 - Zo * Zo.T)/2, Zo)
                        ))

                # extract Euler angles from DCM (indexing is from 0)
                euler[0] = -arctan2(dcm[2,1], dcm[2,2])    # roll
                euler[1] = -arcsin(dcm[2,0])        # pitch
                euler[2] = arctan2(dcm[1,0], dcm[0,0])    # yaw

                # Use gravity vector to correct gyro-drift
                # Roll
                target = arcsin( dot(accv, x_vec) / accl)
                if abs(target) < 1:
                    trim[0] = euler[0] - target
                else:
                    trim[0] = 0

                # Pitch
                t_vec = dot(y_vec, array([[cos(euler[0]), 0, sin(euler[0])],
                        [0, 1, 0],
                        [-sin(euler[0]), 0, cos(euler[0])]]))

                target = arcsin( dot(accv, t_vec) / accl)
                if abs(target) < 1:
                    trim[1] = euler[1] - target
                else:
                    trim[1] = 0

                '''
                # Yaw (unable to compensate with gravity)
                t_vec = x_vec
                target = arcsin( dot(acc, t_vec) / accl)
                if abs(target) < 1:
                    trim[2] = target - euler[2]
                else:
                    trim[2] = 0
                '''

                # PI controller
                error_d = error_d + (time_delta * trim)
                error = (trim * 0.8) + (0.3 * error_d)

                # DCM ends here -------------------------------------------

    if (self.input >= 3):
        # Compute magnitude of acceleration
        return accl
    else:
            return acc[self.input]


win = gtk.Window(gtk.WINDOW_TOPLEVEL)

# main split
vbox = gtk.VBox()
vbox.show()
win.add(vbox)

# linear plots
vbox2 = gtk.VBox()
vbox2.show()
vbox.add(vbox2)
win.set_border_width(5)

for name, rate, my_range, channels in [
    ("Drift Error", 200, (-pi,pi), [
    PlotError(0,(1,0,0)),
    PlotError(1,(0,1,0)),
    PlotError(2,(0,0,1))]),
    ("Rotation (by integration)", 200, (-180,180), [
    PlotRotation(0,(1,0,0)),
    PlotRotation(1,(0,1,0)),
    PlotRotation(2,(0,0,1))]),
    ("Gyros (raw)", 200, (-2000,2000), [
    PlotGyro(0,(1,0,0)),
    PlotGyro(1,(0,1,0)),
    PlotGyro(2,(0,0,1))]),
    ("Accs (raw)", 200, (-2,2), [
    PlotAcc(0,(1,0,0)),
    PlotAcc(1,(0,1,0)),
    PlotAcc(2,(0,0,1)),
    PlotAcc(3,(0,0,0))])
    ]:

    graph = rtgraph.HScrollLineGraph(scrollRate=rate, size=(600,80), range=my_range)
    graph.channels = channels
    graph.show()

    frame = gtk.Frame()
    frame.set_label(name)
    frame.add(graph)
    frame.show()

    vbox2.pack_end(frame)

# rotary dials
hbox = gtk.HBox()
hbox.show()
vbox.add(hbox)

for name, channels in [
    ("Yaw", [ PlotRotationVector(2,(0,0,1))]),
    ("Pitch", [ PlotRotationVector(1,(0,1,0))]),
    ("Roll", [ PlotRotationVector(0,(1,0,0))]),
    ]:

    graph = rtgraph.PolarVectorGraph(pollInterval=200)
    graph.channels = channels
    graph.show()

    frame = gtk.Frame()
    frame.set_label(name)
    frame.add(graph)
    frame.show()

    hbox.pack_end(frame)

# crude Isometeric plot of gravity
graph = rtgraph.IsometricVectorGraph(pollInterval=200, size=(150,150))
graph.channels = [PlotAccv(0,(0,0,0))]

graph.show()

frame = gtk.Frame()
frame.set_label("Gravity")
frame.add(graph)
frame.show()

hbox.pack_end(frame)
'''
'''

win.show()
win.connect("destroy", gtk.mainquit)
gtk.main()
