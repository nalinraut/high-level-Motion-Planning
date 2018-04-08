from mahalanobis import *
from training import *
import scipy as sp
from scipy import linalg as LA

class GlobalKISSLearning:
    """from Koestinger et al CVPR 2012
    """
    def __init__(self,regularization=1e-3,mask=None):
        self.regularization = regularization
        self.M = None
        self.mask = mask
        pass
    def run(self,clusters):
        """Input:
        - clusters: a TrainingClusters instance
        """
        n = len(clusters.problemFeatures[0])
        sigmaPos = sp.eye(n)*self.regularization
        sigmaNeg = sp.eye(n)*self.regularization
        numPos = 0
        numNeg = 0
        for i,c in enumerate(clusters.clusterClose):
            for j in c:
                v = sp.subtract(clusters.problemFeatures[j],clusters.library.primitives[i].problemFeatures)
                sigmaPos += sp.outer(v,v)
                numPos += 1
        for i,c in enumerate(clusters.clusterFar):
            for j in c:
                v = sp.subtract(clusters.problemFeatures[j],clusters.library.primitives[i].problemFeatures)
                sigmaNeg += sp.outer(v,v)
                numNeg += 1
        #Do we want to do covariance, or just E[xxt]?
        sigmaPos = mask(sigmaPos/numPos,self.mask)
        sigmaNeg = mask(sigmaNeg/numNeg,self.mask)
        M = LA.inv(sigmaPos)-LA.inv(sigmaNeg)
        self.M,numNegative = projectSemidef(M)
        print numNegative,"negative eigenvalues"
        return 

    def solution(self):
        """Returns the current solution."""
        return self.M


class LocalKISSLearning:
    """adaptation from Koestinger et al CVPR 2012
    """
    def __init__(self,regularization=1e-3,mask=None):
        self.regularization = regularization
        self.Ms = None
        self.mask = mask
        pass
    def run(self,clusters):
        """Input:
        - clusters: a TrainingClusters instance
        """
        n = clusters.problemSpaceDims()
        self.Ms = [None]*len(clusters.library.primitives)
        for i in xrange(len(clusters.library.primitives)):
            trainPos = []
            trainNeg = []
            close = clusters.clusterClose[i]
            far = clusters.clusterFar[i]
            for j in close:
                trainPos.append(sp.subtract(clusters.problemFeatures[j],clusters.library.primitives[i].problemFeatures))
            for j in far:
                trainNeg.append(sp.subtract(clusters.problemFeatures[j],clusters.library.primitives[i].problemFeatures))
            sigmaPos = sp.eye(n)*self.regularization
            sigmaNeg = sp.eye(n)*self.regularization
            for v in trainPos:
                sigmaPos += sp.outer(v,v)
            for v in trainNeg:
                sigmaNeg += sp.outer(v,v)
            sigmaPos = mask(sigmaPos,self.mask)
            sigmaNeg = mask(sigmaNeg,self.mask)
            M = LA.inv(sigmaPos)-LA.inv(sigmaNeg)
            self.Ms[i],numNegative = projectSemidef(M)
            print numNegative,"negative eigenvalues"
        return 

    def solution(self):
        """Returns the current solution."""
        return self.Ms



