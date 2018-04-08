
import homography
import numpy as np
import scipy.optimize

dp = []
rgbp = []
f = open("correspondences3d.txt")
for l in f:
	if l.strip()=="":
		continue
	points = l.strip().split(",")
	points = map(float, points)
        if len(points)==3:
            #dp.append(map(lambda x:x*1000,points))
	    dp.append(points)
        else:
            assert(len(points)==2)
            rgbp.append([points[0],points[1],1])

#model (function of A (3x3),b (3), ox,oy)
#ei = A*rgbpi+b
#di.x = ei.x/ei.z + ox
#di.y = ei.y/ei.y + oy
def f(A,b,ox,oy):
	global dp,rgbp
	err = 0.0
	for d,rgb in zip(dp,rgbp):
		e = np.dot(A,d)+b
		err += (rgb[0]-e[0]/e[2]-ox)**2
		err += (rgb[1]-e[1]/e[2]-oy)**2
	return err

def df(A,b,ox,oy):
	global dp,rgbp
	dA = np.zeros((3,3))
	db = np.zeros(3)
	dox = 0
	doy = 0
	for d,rgb in zip(dp,rgbp):
		e = np.dot(A,d)+b
		ex = (rgb[0]-e[0]/e[2]-ox)
		ey = (rgb[1]-e[1]/e[2]-oy)
		#dx(u/v) = du/v - u dv/v^2
		#dex / dc = de[0]/dc / e[2] - de[2]/dc * e[0]/(e[2]*e[2])
		#de[0]/dA = [[d[0],d[1],d[2],[0,0,0],[0,0,0]]
		#de[1]/dA = [[0,0,0],[d[0],d[1],d[2],[0,0,0]]
		#de[2]/dA = [[0,0,0],[0,0,0],[d[0],d[1],d[2]]
		dA -= 2*ex*(np.array([d,[0,0,0],[0,0,0]])/e[2] - np.array([[0,0,0],[0,0,0],d])*(e[0]/(e[2]*e[2]))) + 2*ey*(np.array([[0,0,0],d,[0,0,0]])/e[2] - np.array([[0,0,0],[0,0,0],d])*(e[1]/(e[2]*e[2])))
		db -= 2*ex*np.array([1.0/e[2],0.0,-e[0]/(e[2]*e[2])]) + 2*ey*np.array([0.0,1.0/e[2],-e[1]/(e[2]*e[2])])
		dox -= 2*ex
		doy -= 2*ey
	return (dA,db,dox,doy)

def optimal_oxoy(A,b):
	global dp,rgbp
	sumex = 0
	sumey = 0
	for d,rgb in zip(dp,rgbp):
		e = np.dot(A,d)+b
		ex = (rgb[0]-e[0]/e[2])
		ey = (rgb[1]-e[1]/e[2])
		sumex += ex
		sumey += ey
	return (sumex/len(dp),sumey/len(dp))

A = np.eye(3)
b = np.zeros(3)
A[0,0] = 1000
A[1,1] = -1000
ox = 1920/2
oy = 1080/2

print "Predictions, actual"
for d,rgb in zip(dp,rgbp):
	e = np.dot(A,d)+b
	print e[0]/e[2]+ox,e[1]/e[2]+oy,",",rgb[0],rgb[1]

err = f(A,b,ox,oy)

def pack(A,b):
	res = np.zeros(12)
	res[0] = A[0,0]
	res[1] = A[0,1]
	res[2] = A[0,2]
	res[3] = A[1,0]
	res[4] = A[1,1]
	res[5] = A[1,2]
	res[6] = A[2,0]
	res[7] = A[2,1]
	res[8] = A[2,2]
	res[9:12] = b
	return res

def unpack(Ab):
	A = np.array([Ab[0:3],Ab[3:6],Ab[6:9]])
	b = Ab[9:12]
	ox,oy = optimal_oxoy(A,b)
	return A,b,ox,oy

def myf(Ab):
	A,b,ox,oy = unpack(Ab)
	res = f(A,b,ox,oy)
	return res

def mygrad(Ab):
	A = np.array([Ab[0:3],Ab[3:6],Ab[6:9]])
	b = Ab[9:12]
	ox,oy = optimal_oxoy(A,b)
	print A,b,ox,oy
	dA,db,dox,doy = df(A,b,ox,oy)
	return pack(dA,db)


print "Starting error:",err
res = scipy.optimize.fmin_bfgs(myf,pack(A,b),fprime = mygrad)
print "Final error",myf(res)
A,b,ox,oy = unpack(res)
"""
for iters in range(10000):
	dA,db,dox,doy = df(A,b,ox,oy)
	print "Gradient magnitude",np.linalg.norm(dA,1)+np.linalg.norm(db)
	dAdiff = np.zeros((3,3))
	for i in range(3):
		for j in range(3):
			h = np.zeros((3,3))
			h[i,j] = 1e-3
			f1 = f(A+h,b,ox,oy)
			f2 = f(A-h,b,ox,oy)
			dAdiff[i,j] = (f1-f2)/2e-3
	alpha = 1e-5
	testerr = err
	while testerr >= err:
		newA = A-alpha*dA
		newb = b-alpha*db
		newox,newoy = optimal_oxoy(newA,newb)
		testerr = f(newA,newb,newox,newoy)
		if testerr >= err:
			alpha *= 0.5
			if alpha < 1e-20:
				print "No step taken"
				break;
		else:
			print "Step size",alpha
			A,b,ox,oy=newA,newb,newox,newoy
	err = testerr
	print "Error:",err
"""
print A
print b
print ox,oy

print "C code:"
print "double H [3][3]= {{",A[0,0],",",A[0,1],",",A[0,2],"},"
print "                  {",A[1,0],",",A[1,1],",",A[1,2],"},"
print "                  {",A[2,0],",",A[2,1],",",A[2,2],"}};"
print "double J[3] = {",b[0],",",b[1],",",b[2],"};"
print "double ox =",ox,", oy =",oy,";"

