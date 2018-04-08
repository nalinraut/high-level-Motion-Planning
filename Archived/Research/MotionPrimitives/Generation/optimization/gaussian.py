import math
import random

class GaussianDistribution:
    """A univariate or multivariate Gaussian distribution.  Only axis-aligned
    multivariate gaussians are currently supported."""
    def __init__(self,mean,std):
        if hasattr(mean,'__len__'):
            assert len(mean) == len(std),"mean and std dev must be of same dimension"
        self.mean = mean
        self.std = std

    def log_probability(self,x):
        if hasattr(self.mean,'__len__'):
            assert len(x) == len(self.mean),"Evaluation point must be same dimension as gaussian mean"
            xnorm = [(xi-mu)/std for (xi,mu,std) in zip(x,self.mean,self.std)]
            d = len(x)
            logdet = 0.0
            for i in xrange(d):
                logdet += 2.0*math.log(self.std[i])
            xnorm2 = sum(x*x for x in xnorm)
            return -0.5*xnorm2 - math.log(math.pi*2.0)*d/2-0.5*logdet
        else:
            return -0.5*(x-self.mean)**2/self.std**2 - math.log(math.pi*2.0)*0.5 - math.log(self.std)     
    def probability(self,x):
        if hasattr(self.mean,'__len__'):
            assert len(x) == len(self.mean),"Evaluation point must be same dimension as gaussian mean"
            xnorm = [(xi-mu)/std for (xi,mu,std) in zip(x,self.mean,self.std)]
            d = len(x)
            sqrtdet = 1.0
            for i in xrange(d):
                sqrtdet *= abs(self.std[i])
            xnorm2 = sum(x*x for x in xnorm)
            return math.exp(-0.5*xnorm2)/(math.pow(math.pi*2.0,d/2)*sqrtdet)
        else:
            return math.exp(-0.5*(x-self.mean)**2/self.std**2)/(math.sqrt(math.pi*2.0)*self.std)      

    def sample(self):
        if hasattr(self.mean,'__len__'):
            return [random.gauss(self.mean[i],self.std[i]) for i in xrange(len(self.mean))]
        else:
            return random.gauss(self.mean,self.std)
