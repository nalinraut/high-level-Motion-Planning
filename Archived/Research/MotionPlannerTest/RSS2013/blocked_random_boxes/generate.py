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
    for n in number:
        out = open('blocked_%d_%d.xml'%(n,int(density)),'w')
        out.write(prefix)
        out.write('\n')
        for i in xrange(n):
            area = density/n
            assert area <= 1.0
            u=random.uniform(area-1.0,1.0-area)
            h=0.5*(-u + math.sqrt(u*u+4.0*area))
            w=u+h
            x=random.uniform(w*0.25,1.0-w*0.25)
            y=random.uniform(h*0.25,1.0-h*0.25)
            x -= w*0.5
            y -= h*0.5
            out.write("""      <aabb bmin = "%g %g" bmax = "%g %g"/>\n"""%(x,y,x+w,y+h))

        out.write(suffix)
        out.write('\n')
        out.close()

