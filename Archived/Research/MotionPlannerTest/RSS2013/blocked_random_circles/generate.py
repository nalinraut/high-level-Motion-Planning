import random
import math

prefix = """<?xml version="1.0" encoding="UTF-8"?>
<point2d_cspace>
  <domain bmin = "0 0" bmax = "1 1"/>
  <obstacles>
    <geometry2d>"""

suffix = """    </geometry2d>
  </obstacles>
</point2d_cspace>"""

number = [20,30,40,50,60,70,80,90,100]
densities = [1.0,2.0,3.0,4.0,5.0]
for density in densities:
    radius = [math.sqrt(density/(math.pi*float(n))) for n in number]
    for n,r in zip(number,radius):
        out = open('blocked_%d_%d.xml'%(n,int(density)),'w')
        out.write(prefix)
        out.write('\n')
        for i in xrange(n):
            x=random.random()
            y=random.random()
            out.write("""      <circle center="%g %g" radius="%g" />\n"""%(x,y,r))
        out.write(suffix)
        out.write('\n')
        out.close()

