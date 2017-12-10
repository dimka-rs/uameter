#!/usr/bin/env python

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import serial
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import struct

PORT_FILE = '/dev/ttyACM0'
MAX_GRAPH_LEN = 1000	## max number of point in graph
AVG_SIZE = 10		## number of averaging points
RCV_TOUT = 0.5		## read serial tout in ms
UPD_TOUT = 100		## update graph tout in ms
MA_OFFSET = 0
MA_SCALE = 1
UA_OFFSET = 0
UA_SCALE = 1


print "start"
t_arr = np.array([])
i_arr = np.array([])
t_avg_arr = np.array([])
i_avg_arr = np.array([])
low_current = 0
t_sum = 0
i_sum = 0

## graph
#QtGui.QApplication.setGraphicsSystem('raster')
app = QtGui.QApplication([])
#mw = QtGui.QMainWindow()
#mw.resize(800,800)

win = pg.GraphicsWindow(title="uameter")
win.resize(1000,600)
win.setWindowTitle('uameter')
pg.setConfigOptions(antialias=True)
i_plot = win.addPlot(title="Measured current")
i_plot.showGrid(x=True, y=True, alpha=0.5)
#i_plot.setLogMode(x=None, y=True)
i_plot.setLabel("bottom", text="Time, ms")
i_plot.setLabel("left", text="Current, uA")
i_raw = i_plot.plot(pen='y')


def recv():
    global t_arr, i_arr, t_avg_arr, i_avg_arr, t_sum, i_sum
    t_tmp = ""
    t_tmp += ser.read(1)
    t_tmp += ser.read(1)
    t_tmp += ser.read(1)
    t_tmp += ser.read(1)
    t = struct.unpack("<I", t_tmp)[0]
    ## < - little endial
    ## I - unsigned int

    i_tmp = ""
    i_tmp += ser.read(1)
    i_tmp += ser.read(1)
    i_tmp += ser.read(1)
    i_tmp += ser.read(1)
    i = struct.unpack("<I", i_tmp)[0]
    #print("t="+str(t)+" i="+str(i))

    ## if 15th bit is set, then low current mode is active
    if i >= 32768:
        i = i - 32768
        low_current = 1
	i = i * UA_SCALE
	i = i + UA_OFFSET
    else:
        low_current = 0
	i = i * MA_SCALE
	I = i + MA_OFFSET

    t_avg_arr = np.append(t_avg_arr, t)
    i_avg_arr = np.append(i_avg_arr, i)

    if len(t_avg_arr) >= AVG_SIZE:
        #print("average")
        for a in range(0, len(t_avg_arr)):
            t_sum += t_avg_arr[a]
            i_sum += i_avg_arr[a]

        t = int(t_sum / AVG_SIZE)
        i = int(i_sum / AVG_SIZE)
        t_arr = np.append(t_arr, t)
        i_arr = np.append(i_arr, i)
        #print("t="+str(t)+", i="+str(i))
        t_avg_arr = []
        i_avg_arr = []
        t_sum = 0
        i_sum = 0

    if len(t_arr) > 0 and t < t_arr[-1]:
        print("reset data! MCU has been restarted!")
        t_arr = []
        i_arr = []


def upd():
    global t_arr, i_arr

    ## limit graph length
    while len(t_arr) > MAX_GRAPH_LEN:
        #print("truncate")
        t_arr = np.delete(t_arr, 0)
        i_arr = np.delete(i_arr, 0)

    ## update dataset
    #print("update")
    i_raw.setData(t_arr, i_arr)

## Open serial
ser = serial.Serial(
        port=PORT_FILE,
        baudrate=115200,
	timeout=None,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
ser.isOpen()

tmr = QtCore.QTimer()
tmr.timeout.connect(recv)
tmr.start(RCV_TOUT) ## period in ms

tmr2 = QtCore.QTimer()
tmr2.timeout.connect(upd)
tmr2.start(UPD_TOUT)

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()


