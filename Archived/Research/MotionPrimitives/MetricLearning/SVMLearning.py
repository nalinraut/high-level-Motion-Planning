from mahalanobis import *
from training import *
import scipy as sp
from scipy import linalg as LA
import time

class SVMMetricLearner:
    def __init__(self,mu=0.1,alpha=None,startM=None,mask=None):
        """Input:
        - mu: regularization term
        - alpha: an initial step size
        - startM: an initial matrix (Default: identity)
        - mask: a set of matrix entries that will be optimized.  Either
          None (all entries), 'diagonal' (diagonal entries), or a list of
          pairs.
        """
    
        self.mu = mu
        self.alpha = alpha
        self.M = startM
        self.xtol = 1e-8
        self.mask = mask
        
    def run(self,center,close,far,maxiters):
        """Loss function:
        L(M) = sum(c in close)max((x-c)^T M (x-c) - 1,0)
               + sum(f in far)max(1-(x-f)^T M (x-f),0)
        """
        if self.M == None:
            self.M = sp.eye(len(center))

        print "SVM learning,",len(close),"near points",len(far),"far points"
        for c in close:
            assert(len(c)==len(center))
        for c in far:
            assert(len(c)==len(center))
        if len(close)+len(far)==0:
            return
        self.loss,grad = svmGradient(center,close,far,self.mu,self.M)
        grad = mask(grad,self.mask)
        if self.alpha == None:
            self.alpha  = 1.0/self.linalg.norm(grad)
        while maxiters > 0:
            #if maxiters % 20 == 0:
            if maxiters % 1 == 0:
                print "loss",self.loss,"step size",self.alpha
            maxiters -= 1
            Mnew,numNegative  = projectSemidef(self.M - self.alpha*grad)
            if LA.norm(self.M-Mnew) < self.xtol:
                return "converged"
            if sp.count_nonzero(Mnew)==0:
                self.alpha *= 0.5
            else:
                futureLoss,futureGrad = svmGradient(center,close,far,self.mu,Mnew)
                if futureLoss <= self.loss:
                    self.loss = futureLoss
                    self.M = Mnew
                    grad = mask(futureGrad,self.mask)
                    self.alpha *= 1.2
                else:
                    self.alpha *= 0.5
        return "maxiters reached"
    def solution(self):
        return (self.M,1)

def svmGradient(center,close,far,mu,M):
    """Loss function:
    L(M) = mu||M||^2 + (1-mu)sum(c in close)max(1-b+(x-c)^T M (x-c),0)
                     + (1-mu)sum(f in far)max(1+b-(x-f)^T M (x-f),0))
    """
    loss = LA.norm(M)**2
    grad = (2*mu) * M
    numSVs = 0
    for x in close:
        d = mahalanobis(center,x,M)
        if d > 1.0:
            numSVs += 1
            loss += (1.0-mu)*(d-1.0)
            diff = sp.subtract(center,x)
            grad += (1.0-mu)*sp.outer(diff,diff)
    for x in far:
        d = mahalanobis(center,x,M)
        if d < 1.0:
            numSVs += 1
            loss += (1.0-mu)*(1.0 - d)
            diff = sp.subtract(center,x)
            grad -= (1.0-mu)*sp.outer(diff,diff)
    scale = 1.0 / (len(close)+len(far))
    return loss*scale,grad*scale

class GlobalSVMLearning:
    """Learns a global metric via an SVM technique"""
    def __init__(self,mu=0.1,alpha=1,maxiter=100,startM=None,mask=None):
        self.mu = mu
        self.alpha=alpha
        self.M = startM
        self.maxiter = maxiter
        self.mask = mask
    def run(self,clusters):
        #gather all instances centered at their respective centers
        center = sp.array(clusters.problemFeatures[0])*0.0
        close = []
        far = []
        for i,p in enumerate(clusters.library.primitives):
            fclose,ffar = clusters.clusterFeatures(i)
            close += [sp.subtract(f,p.problemFeatures) for f in fclose]
            far += [sp.subtract(f,p.problemFeatures) for f in ffar]
        learner = SVMMetricLearner(self.mu,self.alpha,self.M,self.mask)
        learner.run(center,close,far,self.maxiter)
        Mb = learner.solution()
        self.M = Mb[0]/Mb[1]
        return True
    def solution(self):
        return self.M


class LocalSVMLearning:
    """Learns local metrics via an SVM technique"""
    def __init__(self,mu=0.1,alpha=1,maxiter=100,startMs=None,mask=None):
        self.mu = mu
        self.alpha=alpha
        self.Ms = startMs
        self.maxiter = maxiter
        self.mask = mask
    def run(self,clusters):
        if self.Ms == None:
            self.Ms = [None]*len(clusters.library.primitives)
        for i,p in enumerate(clusters.library.primitives):
            fclose,ffar = clusters.clusterFeatures(i)
            learner = SVMMetricLearner(self.mu,self.alpha,self.Ms[i],self.mask)
            learner.run(p.problemFeatures,fclose,ffar,self.maxiter)
            Mb = learner.solution()
            self.Ms[i] = Mb[0]/Mb[1]
        return True
    def solution(self):
        return self.Ms
