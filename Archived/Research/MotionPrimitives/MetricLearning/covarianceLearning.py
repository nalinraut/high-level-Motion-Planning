from mahalanobis import *
from training import *
from scipy import linalg as LA

class GlobalCovarianceLearning:
    """Learns a global covariance matrix M.

    Parameters:
    - regularization: an L2 regularization term
    """
    def __init__(self,regularization=0.001,mask=None):
        self.regularization = regularization
        self.M = None
        self.mask = mask
    def run(self,clusters):
        dim = clusters.problemSpaceDims()
        thisM = clusters.problemFeatures
        diag = [self.regularization] * dim
        thisM = sp.cov(sp.array(thisM).T, bias = 1) + sp.diag(diag)
        thisM = mask(thisM,self.mask)
        self.M = LA.inv(thisM)
    def solution(self):
        return self.M

class LocalCovarianceLearning:
    """Learns local covariance matrices M[0],...,M[N-1].

    Parameters:
    - regularization: an L2 regularization term
    - mask: a mask for entries of the matrix to estimate (default: all)
    """
    def __init__(self,regularization=0.001,mask=None):
        self.regularization = regularization
        self.Ms = None
        self.mask = mask
    def run(self,clusters):
        self.Ms = []
        dim = clusters.problemSpaceDims()
        for i,p in enumerate(clusters.library.primitives):
            Cclose = clusters.clusterClose[i]
            thisM = [clusters.problemFeatures[problem] for problem in Cclose]
            diag = [self.regularization] * dim
            if len(thisM)==0:
                thisM = sp.diag(diag)
            else:
                thisM = sp.cov(sp.array(thisM).T, bias = 1) + sp.diag(diag)
            thisM = mask(thisM,self.mask)
            thisM = LA.inv(thisM)
            self.Ms.append(thisM)
    def solution(self):
        return self.Ms
