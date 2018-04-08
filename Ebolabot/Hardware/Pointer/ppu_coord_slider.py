""" Simple GUI slider that sends coordinate commands to slider """

from Tkinter import *
from ttk import Separator
from pointer import *

# Set up pointer
p = PPU(port="/dev/ttyUSB0")

# Set up GUI
root = Tk()
root.title("PPU Controller")

Label(root, text="Y").grid(row=0, column=0)
s_y = Scale(root, from_=-4, to=4, length=300, resolution=0.01)
s_y.grid(row=1, column=0, pady=5,padx=10)
s_y.set(0)

Label(root, text="Z").grid(row=0, column=1)
s_z = Scale(root, from_=0, to=6, length=300, resolution=0.01)
s_z.grid(row=1, column=1, pady=5,padx=10)
s_z.set(3)

Label(root, text="X").grid(row=2, columnspan=2)
s_x = Scale(root, from_=7, to=9, length=300, resolution=0.002)
s_x.grid(row=3,columnspan=2,padx=10)
s_x.set(7.5)

Separator(root).grid(row=4,columnspan=2, pady=15, sticky="EW")

Label(root, text="Speed").grid(row=5, columnspan=2)
s_step = Scale(root, from_=1, to=30, orient=HORIZONTAL, length=300)
s_step.set(1)
s_step.grid(row=6,columnspan=2,padx=10)

# initalize to physical state
sensedConfig = p.initToPhysicalConfig(timeout=5)

while True:
    root.update()
    p.setCmdPos([s_x.get(), s_y.get(), s_z.get()])
    p.stepSize = s_step.get()
    p.update()
