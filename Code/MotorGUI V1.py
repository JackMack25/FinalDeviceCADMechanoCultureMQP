# GUI to control motor
# 7 inputs
# Strain, Up Speed, Hold Up, Down Speed, Hold Down, Cycles, Step

import re
import tkinter as tk
import serial 
import datetime

import serial.serialutil

serialInst = serial.Serial()

strainToSteps = 3.1496
#1% strain = 0.1 mm = 3.1496 steps

def numbers(str):
    #removes letters and special characters other than '.'
    return re.sub('[^0-9.]', '', str)

def getNum(tx):
    return numbers(tx.get())

def getSteps(tx):
    return str(int(float(getNum(tx))*strainToSteps))

def getMilSec(tx):
    return str(int(float(getNum(tx))*1000))

def getPeriod():
    return float(getNum(StrainTx))/float(getNum(UpSpeedTx))+float(getNum(HoldUpTx))+float(getNum(StrainTx))/float(getNum(DownSpeedTx))+float(getNum(HoldDownTx))

def getTotalRunTime():
    secs = float(getNum(CyclesTx))*getPeriod()
    return datetime.timedelta(seconds=secs)

def startPress():
    msg = '<'+getSteps(StrainTx)+','+getSteps(UpSpeedTx)+','+getMilSec(HoldUpTx)+','+getSteps(DownSpeedTx)+','+getMilSec(HoldDownTx)+','+getNum(CyclesTx)+','+'0'+'>'
    serialInst.write(msg.encode())
    print(msg.encode())

def stepUpPress():
    msg = '<0,0,0,0,0,0,'+getNum(ManualTx)+'>'
    serialInst.write(msg.encode())
    print(msg.encode())

def stepDownPress():
    msg = '<0,0,0,0,0,0,-'+getNum(ManualTx)+'>'
    serialInst.write(msg.encode())
    print(msg.encode())

def stopPress():
    msg = "<!>"
    serialInst.write(msg.encode())
    print(msg)

def updatePress():
    distOutLb.config(text=float(getNum(StrainTx))/10)
    periodOutLb.config(text=round(getPeriod(),3))
    freqOutLb.config(text=round(1/getPeriod(),3))
    timeOutLb.config(text=getTotalRunTime())

def connectToSerial():
    try:
        serialInst.port = SerialTx.get()
        serialInst.baudrate = 9600
        serialInst.open()
        ConnectLb.config(text="Connected")
    except serial.serialutil.SerialException:
        ConnectLb.config(text="Failed Connection")


# Make the window
win = tk.Tk()
win.title("Strain Form Control")
win.minsize(200,60)

# Add Label and Text
SerialLb = tk.Label(win,text="Serial Port")
SerialLb.grid(column=1,row=0)
SerialTx = tk.Entry(win,bd=1)
SerialTx.grid(column=2,row=0)
ConnectBtn = tk.Button(win,text="Connect",command=connectToSerial)
ConnectBtn.grid(column=3,row=0)
ConnectLb = tk.Label(win,text="Not Connected")
ConnectLb.grid(column=4,row=0)

StrainLb = tk.Label(win,text="Strain (%)")
StrainLb.grid(column=1,row=1)
StrainTx = tk.Entry(win,bd=1)
StrainTx.grid(column=2,row=1)

UpSpeedLb = tk.Label(win,text="Up Speed (% / s)")
UpSpeedLb.grid(column=1,row=2)
UpSpeedTx = tk.Entry(win,bd=1)
UpSpeedTx.grid(column=2,row=2)

HoldUpLb = tk.Label(win,text="Hold Up (s)")
HoldUpLb.grid(column=1,row=3)
HoldUpTx = tk.Entry(win,bd=1)
HoldUpTx.grid(column=2,row=3)

DownSpeedLb = tk.Label(win,text="Down Speed (% / s)")
DownSpeedLb.grid(column=1,row=4)
DownSpeedTx = tk.Entry(win,bd=1)
DownSpeedTx.grid(column=2,row=4)

HoldDownLb = tk.Label(win,text="Hold Down (s)")
HoldDownLb.grid(column=1,row=5)
HoldDownTx = tk.Entry(win,bd=1)
HoldDownTx.grid(column=2,row=5)

CyclesLb = tk.Label(win,text="Cycles")
CyclesLb.grid(column=1,row=6)
CyclesTx = tk.Entry(win,bd=1)
CyclesTx.grid(column=2,row=6)

StartBtn = tk.Button(win,text="Start",command=startPress)
StartBtn.grid(column=1,row=7)
StopBtn = tk.Button(win,text="Stop",command=stopPress)
StopBtn.grid(column=2,row=7)
UpdateBtn = tk.Button(win,text="Calculate",command=updatePress)
UpdateBtn.grid(column=3,row=7)

distLb = tk.Label(win,text="Distance (mm)")
distLb.grid(column=3,row=1)
distOutLb = tk.Label(win,text="N/A")
distOutLb.grid(column=4,row=1)

periodLb = tk.Label(win,text="Period (s)")
periodLb.grid(column=3,row=2)
periodOutLb = tk.Label(win,text="N/A")
periodOutLb.grid(column=4,row=2)

freqLb = tk.Label(win,text="Frequency (Hz)")
freqLb.grid(column=3,row=3)
freqOutLb = tk.Label(win,text="N/A")
freqOutLb.grid(column=4,row=3)

timeLb = tk.Label(win,text="Time (H:MM:SS)")
timeLb.grid(column=3,row=6)
timeOutLb = tk.Label(win,text="N/A")
timeOutLb.grid(column=4,row=6)

SpacerLb = tk.Label(win,text="")
SpacerLb.grid(column=1,row=8)


ManualLb = tk.Label(win,text="Reposition Steps")
ManualLb.grid(column=1,row=9)
ManualTx = tk.Entry(win,bd=1)
ManualTx.grid(column=2,row=9)

StepUpBtn = tk.Button(win,text="Step Up",command=stepUpPress)
StepUpBtn.grid(column=3,row=9)
StepDownBtn = tk.Button(win,text="Step Down",command=stepDownPress)
StepDownBtn.grid(column=4,row=9)

win.mainloop()