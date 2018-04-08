import random
import math

def sample_circle(r=1.0):
    u = random.uniform(0,math.pi*2)
    return (math.cos(u)*r,math.sin(u)*r)

def polar_coords(xy):
    x,y, = xy
    r2 = sum(v**2 for v in xy)
    r = math.sqrt(r2)
    theta = math.atan2(y,x)
    return (r,theta)

def spherical_coords(xyz):
    x,y,z = xyz
    r2 = sum(v**2 for v in xyz)
    r = math.sqrt(r2)
    theta = math.atan2(y,x)
    phi = math.atan2(z,math.sqrt(x**2+y**2))
    return (r,theta,phi)

#axis-aligned ellipsoid of width a, height b
#samples along the x axis, finds y through projection
def sample_ellipse_x(a,b):
    x = random.uniform(-a,a)
    y = random.choice([-b,b])*math.sqrt(1.0-(x/a)**2)
    return (x,y)

def implicit_function_jacobian(xy,dcdx,dcdy):
    x,y=xy
    Jy = dcdy(x,y)
    return np.linalg.inv(Jy)*dcdx(x,y)

#returns the determinant of the metric tensor of the
#sampling function used to sample xy
def sample_ellipse_x_metric(a,b,xy):
    #x,y = T(x) = (x,b*sqrt(1-(x/a)^2))
    #dT/dx = (1,-b*x/a^2/sqrt(1-(x/a)^2))
    #dT/dx = (1,-b^2*x/a^2/y)
    #
    #does implicit function work?
    #C(x,y) = (x/a)^2 + (y/b)^2 - 1 = 0
    #dC/dx = 2x/a^2, dC/dy = 2y/b^2
    #dC/dy(x,y)^-1 = b^2/2y
    #-dC/dy(x,y)^-1 dC/dx(x,y) = -xb^2/ya^2
    #yes
    x,y = xy
    deriv = (1,-(b**2/a**2)*x/y)
    return deriv[0]**2 + deriv[1]**2

def project(xy,a,b):
    x,y = xy
    u,v = x/a,y/b
    l = math.sqrt(u**2+v**2)
    return (a*u/l,b*v/l)

def sample_ellipse_project(a,b,eps=0.1):
    while True:
        x = random.uniform(a*(-1.0-eps),a*(1.0+eps))
        y = random.uniform(b*(-1.0-eps),b*(1.0+eps))
        if abs((x/a)**2 + (y/b)**2 - 1.0) <= eps**2:
            return project((x,y),a,b)

def sphere_gibbs_1(pt):
    s = sum(v**2 for v in pt)
    ptnew = pt[:]
    for i,v in enumerate(pt):
        v2 = 1.0 + v**2 - s
        v2c = min(max(v2,0.0),1.0)
        ptnew[i] = random.choice([-1,1])*math.sqrt(v2c)
        s += v2 - v**2
    return ptnew

def sphere_gibbs_2(pt):
    assert (len(pt) >= 2)
    s = sum(v**2 for v in pt)
    ptnew = pt[:]
    for i in xrange(len(pt)):
        v = ptnew[i]
        j = (i+1)%len(pt)
        w = ptnew[j]
        r2 = 1.0 + v**2 + w**2 - s
        r2c = min(max(r2,0.0),1.0)
        ptnew[i],ptnew[j] = sample_circle(math.sqrt(r2c))
        s += r2 -  v**2 - w**2
    return ptnew

"""
#TEST: sphere gibbs 2 works
numsamples = 10000
pt = [1,0,0]
print "sphere_gibbs_1"
print "r theta phi"
for i in xrange(numsamples):
    pt = sphere_gibbs_1(pt)
    for v in spherical_coords(pt):
        print v,
    print

print "sphere_gibbs_2"
print "r theta phi"
pt = [1,0,0]
for i in xrange(numsamples):
    pt = sphere_gibbs_2(pt)
    for v in spherical_coords(pt):
        print v,
    print
"""

a = 1.0
b = 10.0
numsamples = 10000
print "a=%g,b=%g"%(a,b)
print "ellipse_atlas_x"
print "r theta"
for i in xrange(numsamples):
    pt = sample_ellipse_x(a,b)
    detg = sample_ellipse_x_metric(a,b,pt)
    for v in polar_coords((pt[0]/a,pt[1]/b)):
        print v,
    print

print "ellipse_project"
print "r theta"
for i in xrange(numsamples):
    pt = sample_ellipse_project(a,b)
    for v in polar_coords((pt[0]/a,pt[1]/b)):
        print v,
    print
