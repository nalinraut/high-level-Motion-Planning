import math
import random
from gaussian import *
from kde import *


r = 1.0

mu = [0.1,0.0]
sigma = [0.3,0.3]
p = GaussianDistribution(mu,sigma)

def project(x):
    norm = math.sqrt(x[0]*x[0]+x[1]*x[1])
    if norm == 0:
        return [r,0]
    return [x[0]*r/norm,x[1]*r/norm]

def constraint(x):
    return x[0]*x[0]+x[1]*x[1] - r*r

def sample():
    return p.sample()

def probability(x):
    return p.probability(x)

def weightedSample(values):
    sumV = sum(values)
    v = random.random()*sumV
    for i in xrange(len(values)):
        if v < values[i]:
            return i
        v -= values[i]
    raise ValueError("Went past end of array for some reason")

def sampleproject(range=1e30):
    while 1:
        s=sample()
        if abs(constraint(s)) <= range:
            return project(s)
    return None

def sampleperturbproject(s0,delta):
    #TODO: sample on linear approx to constraint manifold
    s=[random.gauss(v,delta) for v in s0]
    return project(s)

def samplemetropolis(burnin,numsamples=1):
    x = sampleproject()
    samples = []
    for i in xrange(burnin+numsamples):
        xp = sampleproject()
        a1 = probability(xp)/probability(x)
        a2 = 1.0
        a = a1*a2
        if random.random() < a:
            #accept
            x = xp
        else:
            #reject
            pass
        if i >= burnin:
            samples.append(x)
    if numsamples==1:
        return samples[0]
    else:
        return samples

def samplemetropolis_perturb(delta,burnin,numsamples=1):
    x = sampleproject()
    samples = []
    for i in xrange(burnin+numsamples):
        xp = sampleperturbproject(x,delta)
        a1 = probability(xp)/probability(x)
        a2 = 1.0
        a = a1*a2
        if random.random() < a:
            #accept
            x = xp
        else:
            #reject
            pass
        if i >= burnin:
            samples.append(x)
    if numsamples==1:
        return samples[0]
    else:
        return samples


def sampleKernel(numsamples=10000,numkernel=1000,kernelWidth=0.1):
    newpts = []
    while(len(newpts) < numsamples):
        pts = [sampleproject() for i in xrange(numkernel)]
        kde = KernelDensityEstimator(pts,[kernelWidth,kernelWidth])
        w = [probability(x)/kde(x) for x in pts]
        #sample from the points according to w
        for i in xrange(numkernel/10):
            newpts.append(pts[weightedSample(w)][:])
        print len(newpts),"/",numsamples,"samples done"
    return newpts

def testProject(fn,range=1e30,iters=10000):
    pts = []
    nsampled = 0
    while nsampled < iters:
        s=sampleproject(range)
        pts.append(s)
        nsampled += 1
    f = open(fn,'w')
    f.write("#x y angle\n")
    for s in pts:
        f.write("%f %f %f\n"%(s[0],s[1],math.atan2(s[1],s[0])))
    f.close()

def testMetropolis(fn,burnin=0,iters=10000):
    pts = samplemetropolis(burnin,iters)
    f = open(fn,'w')
    f.write("#x y angle\n")
    for s in pts:
        f.write("%f %f %f\n"%(s[0],s[1],math.atan2(s[1],s[0])))
    f.close()

def testMetropolisPerturb(fn,delta,burnin=0,iters=10000):
    pts = samplemetropolis_perturb(delta,burnin,iters)
    f = open(fn,'w')
    f.write("#x y angle\n")
    for s in pts:
        f.write("%f %f %f\n"%(s[0],s[1],math.atan2(s[1],s[0])))
    f.close()

def testKernel(fn,iters=10000,kernelWidth=1.0):
    pts = sampleKernel(iters,1000,kernelWidth)
    f = open(fn,'w')
    f.write("#x y angle\n")
    for s in pts:
        f.write("%f %f %f\n"%(s[0],s[1],math.atan2(s[1],s[0])))
    f.close()


#testProject('project.txt')
#testProject('project0.001.txt',0.001)
#testProject('project0.01.txt',0.01)
#testProject('project0.1.txt',0.1)
#testProject('project0.5.txt',0.5)
#testProject('project1.txt',1.0)
#testMetropolis('metropolis.txt')
#testMetropolisPerturb('metropolisp_0.1.txt',0.1)

testKernel('kernel0.01.txt',kernelWidth=0.01)
testKernel('kernel0.1.txt',kernelWidth=0.1)
