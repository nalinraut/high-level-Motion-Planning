from gaussian import *

class KernelDensityEstimator:
    def __init__(self,c,h):
        self.c = c
        self.h = h

    def __call__(self,x):
        s = 0.
        for c in self.c:
            g = GaussianDistribution(c,self.h)
            s += g.probability(x)
        return s/len(self.c)
