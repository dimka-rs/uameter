#!/usr/bin/env python

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import serial
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import struct

MAX_GRAPH_LEN = 1000
PORT_FILE = '/dev/ttyACM0'
RCV_TOUT = 0.5  ## read serial tout in ms
UPD_TOUT = 100  ## update graph tout in ms

print "start"
x = np.array([])
y = np.array([])
#mn = np.array([])
#av1 = np.array([])
#av2 = np.array([])
#mx = np.array([])
low_current = 0

## graph
#QtGui.QApplication.setGraphicsSystem('raster')
app = QtGui.QApplication([])
#mw = QtGui.QMainWindow()
#mw.resize(800,800)

win = pg.GraphicsWindow(title="uameter")
win.resize(1000,600)
win.setWindowTitle('uameter')
pg.setConfigOptions(antialias=True)
p = win.addPlot(title="Measured current")
p.showGrid(x=True, y=True, alpha=0.5)
wraw = p.plot(pen='b')
#wmin = p.plot(pen='g')
#wav1 = p.plot(pen='y')
#wav2 = p.plot(pen='w')
#wmax = p.plot(pen='r')


def recv():
    global x, y, mn, mx, av1, av2
    tm=""
    tm += ser.read(1)
    tm += ser.read(1)
    tm += ser.read(1)
    tm += ser.read(1)
    m = struct.unpack("<I", tm)[0]
    ## < - little endial
    ## I - unsigned int

    tw=""
    tw += ser.read(1)
    tw += ser.read(1)
    tw += ser.read(1)
    tw += ser.read(1)
    w = struct.unpack("<I", tw)[0]
    #print("m="+str(m)+" w="+str(w))
    if len(x) > 0 and m < x[-1]:
        print("reset data! MCU has been restarted!")
        x=[]
        y=[]
#        mn=[]
#        mx=[]
#        av1=[]
#        av2=[]

    ## if 15th bit is set, then low current mode is active
    if w > 32768:
        w = w - 32768
        low_current = 1
	p.setTitle("uA")
	## TODO: scale w for low current
    else:
        low_current = 0
	p.setTitle("mA")
	## TODO: scale w for high current

    x=np.append(x, m)
    y=np.append(y, w)
#    av1d=10
#    av2d=100
#    if len(y) < av1d:
#        mn=np.append(mn, w)
#        mx=np.append(mx, w)
#        av1=np.append(av1, w)
#    else:
#        av1t=0
#        mnt=y[-1]
#        mxt=y[-1]
#        for i in range(-1, -1*av1d-1, -1):
#            av1t=av1t+y[i]
#            if y[i] < mnt:
#                mnt=y[i]
#            if y[i] > mxt:
#                mxt=y[i]
#
#        av1t=round((av1t-mnt-mxt)/(av1d-2))
#        av1=np.append(av1, av1t)
#        mn=np.append(mn, mnt)
#        mx=np.append(mx, mxt)
#
#    if len(y) < av2d:
#        av2=np.append(av2, w)
#    else:
#        av2t=0
#        for i in range(-1, -1*av2d-1, -1):
#            av2t=av2t+y[i]
#        av2t=round(av2t/av2d)
#        av2=np.append(av2, av2t)

## limit graph length
    if len(y) > MAX_GRAPH_LEN:
       x=np.delete(x, 0)
       y=np.delete(y, 0)
#       mn=np.delete(mn, 0)
#       mx=np.delete(mx, 0)
#       av1=np.delete(av1, 0)
#       av2=np.delete(av2, 0)

def upd():
    wraw.setData(x, y)
#    wmin.setData(x, mn)
#    wmax.setData(x, mx)
#    wav1.setData(x, av1)
#    wav2.setData(x, av2)

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


