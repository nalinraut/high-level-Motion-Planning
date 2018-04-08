
import homography
import numpy as np

fp = []
tp = []
f = open("corresponding_points.txt")
for l in f:
	if l.strip()=="":
		continue
	points = l.strip().split(",")
	points = map(int, points)
	fp.append([points[0], points[1], 1])
	tp.append([points[2], points[3], 1])

fp = np.array(fp).transpose()
tp = np.array(tp).transpose()

H = homography.H_from_points(fp,tp)

print H

for i in xrange(len(fp.transpose())):
	f = fp.transpose()[i]
	transformed = H*np.matrix(f).transpose()
	transformed = homography.normalize(transformed)
	print [int(j) for j in transformed.flat]
	print [int(j) for j in tp.transpose()[i].flat]
	print "==================="
