import math

prefix = """<?xml version="1.0" encoding="UTF-8"?>
<point2d_cspace>
  <domain bmin = "0 0" bmax = "1 1"/>
  <obstacles>
    <geometry2d>"""

suffix = """    </geometry2d>
  </obstacles>
</point2d_cspace>"""

count = [1,2,3,5,10,20]
passageWidths = [0.05,0.04,0.03,0.02,0.01]
for passageWidth in passageWidths:
    for n in count:
        out = open('free_%d_%g.xml'%(n,passageWidth),'w')
        out.write(prefix)
        out.write('\n')
        for i in xrange(n):
            for j in xrange(n):
                x=float(i)/float(n)
                y=float(j)/float(n)
                w = 1.0/float(n)
                h = 1.0/float(n)
                x += passageWidth*0.5
                y += passageWidth*0.5
                w -= passageWidth
                h -= passageWidth
                out.write("""      <aabb bmin = "%g %g" bmax = "%g %g"/>\n"""%(x,y,x+w,y+h))

        out.write(suffix)
        out.write('\n')
        out.close()

