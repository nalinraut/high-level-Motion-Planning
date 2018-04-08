import random
import gaussian

class Sampler:
    def __call__(self):
        return None
    def probability(self,x):
        return None

class BoxSampler(Sampler):
    def __init__(self,bounds):
        self.bounds = bounds
    def __call__(self):
        return [random.uniform(a,b) for a,b in self.bounds]
    def probability(self,x):
        vol = 1.0
        for a,b in self.bounds:
            vol *= (b-a)
        return 1.0/vol

class BoxPerturbSampler(Sampler):
    def __init__(self,center,radius):
        self.center = center
        self.radius = radius
    def __call__(self):
        if hasattr(self.radius,'__iter__'):
            return [random.uniform(c-r,c+r) for c,r in zip(self.center,self.radius)]
        else:
            return [random.uniform(c-self.radius,c+self.radius) for c in self.center]
    def probability(self,x):
        if hasattr(self.radius,'__iter__'):
            vol = 1.0
            for r in self.radius:
                vol *= 2.0*r
            return 1.0/vol
        else:
            return pow(2.0*self.radius,-len(self.center))

class GaussianSampler(Sampler):
    def __init__(self,center,stdev):
        self.gaussian = gaussian.GaussianDistribution(center,stdev)
    def __call__(self):
        return self.gaussian.sample()
    def probability(self,x):
        return self.gaussian.probability(x)

class GuardedSampler:
    def __init__(self,sampler,guard):
        self.sampler = sampler
        self.guard = guard
        self.guard.start()
    def __call__(self):
        if not self.guard():
            raise StopIteration()
        return self.sampler()
