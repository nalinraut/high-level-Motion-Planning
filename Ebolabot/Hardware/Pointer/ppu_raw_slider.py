""" Simple GUI sliders that send configuration commands to slider """

from Tkinter import *
from ttk import Separator
from pointer import *

# Set up pointer
p = PPU(port="/dev/ttyUSB3")

def doHalt():
    sensedConfig = p.setHalt()
    s_pan.set(sensedConfig[0])
    s_tilt.set(sensedConfig[1])
    ext = sensedConfig[2]
    s_ext.set(ext)

# Set up GUI
root = Tk()
root.title("PPU Controller")

Label(root, text="Tilt").grid(row=0, column=0)
s_tilt = Scale(root, from_=1, to=4095, length=300)
s_tilt.grid(row=1, column=0, pady=5,padx=10)
s_tilt.set(1024)

Label(root, text="Extension").grid(row=0, column=1)
s_ext = Scale(root, from_=2000, to=1050, length=300)
s_ext.grid(row=1, column=1, pady=5,padx=10)

Label(root, text="Pan").grid(row=2, columnspan=2)
s_pan = Scale(root, from_=4095, to=1, orient=HORIZONTAL, length=300)
s_pan.set(2048)
s_pan.grid(row=3,columnspan=2,padx=10)

Separator(root).grid(row=4,columnspan=2, pady=15, sticky="EW")

Label(root, text="Speed").grid(row=5, columnspan=2)
s_step = Scale(root, from_=1, to=30, orient=HORIZONTAL, length=300)
s_step.set(1)
s_step.grid(row=6,columnspan=2,padx=10)

btn_pnl = Frame(root)
btn_pnl.grid(row=7,columnspan=2,pady=10, padx=5)
Button(btn_pnl, text="Halt", fg="red", activeforeground="red", command=doHalt).grid()

# initalize to physical state
sensedConfig = p.initToPhysicalConfig(timeout=5)
s_pan.set(sensedConfig[0])
s_tilt.set(sensedConfig[1])
s_ext.set(sensedConfig[2])

while True:
    root.update()
    p.setCmdConfig([s_pan.get(), s_tilt.get(), s_ext.get()], mode='raw')
    p.stepSize = s_step.get()
    p.update()

