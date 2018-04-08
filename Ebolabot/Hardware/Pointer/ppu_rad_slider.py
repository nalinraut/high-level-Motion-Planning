""" Simple GUI sliders that send configuration commands to slider """

from Tkinter import *
from ttk import Separator
from pointer import *
from math import pi
from pointer_client import PPUClient

# Set up pointer
p = PPUClient()

def doHalt():
    sensedConfig = p.setHalt(retval='rad/prct')
    s_pan.set(sensedConfig[0])
    s_tilt.set(sensedConfig[1])
    ext = sensedConfig[2]
    s_ext.set(ext)

# Set up GUI
root = Tk()
root.title("PPU Controller")

Label(root, text="Tilt").grid(row=0, column=0)
s_tilt = Scale(root, from_=-pi, to=pi, length=300, resolution=0.0015)
s_tilt.grid(row=1, column=0, pady=5,padx=10)

Label(root, text="Extension").grid(row=0, column=1)
s_ext = Scale(root, from_=1.0, to=.05, length=300, resolution=.001)
s_ext.grid(row=1, column=1, pady=5,padx=10)

Label(root, text="Pan").grid(row=2, columnspan=2)
s_pan = Scale(root, from_=-pi, to=pi, orient=HORIZONTAL, length=300, resolution=0.0015)
s_pan.grid(row=3,columnspan=2,padx=10)

Separator(root).grid(row=4,columnspan=2, pady=15, sticky="EW")

Label(root, text="Speed").grid(row=5, columnspan=2)
s_step = Scale(root, from_=1, to=30, orient=HORIZONTAL, length=300)
s_step.set(5)
s_step.grid(row=6,columnspan=2,padx=10)

btn_pnl = Frame(root)
btn_pnl.grid(row=7,columnspan=2,pady=10, padx=5)
Button(btn_pnl, text="Halt", fg="red", activeforeground="red", command=doHalt).grid()

# initalize to physical state
sensedConfig = p.getSnsConfig(mode='rad/prct')
s_pan.set(sensedConfig[0])
s_tilt.set(sensedConfig[1])
s_ext.set(sensedConfig[2])

while True:
    root.update()
    p.setCmdConfig([s_pan.get(), s_tilt.get(), s_ext.get()], mode='rad/prct')
    p.setStepSize(s_step.get())

