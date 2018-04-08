import scipy as sp
from scipy import linalg as LA
from primitivelibrary import *


def mahalanobis(p1, p2, metric):
    diff = sp.subtract(p1, p2)
    dist1 = diff.dot(metric)
    dist2 = dist1.dot(diff.transpose())
    return dist2.real

                        
                           
def mask(mat,items):
    """Returns a matrix with the given items retained (and 0's elsewhere).
    items can be:
    - None or 'all': all items (in this case, mat is returned, not a copy)
    - 'diagonal' or 'diag': only diagonal items
    - 'scalar': only a constant multiple times the identity
    - a list of pairs: a list of retained indices
    """
    if items==None or items=='all':
        return mat
    elif items=='diagonal' or items=='diag':
        res = sp.zeros(mat.shape)
        for i in xrange(min(mat.shape[0],mat.shape[1])):
            res[i,i] = mat[i,i].real
        return res
    elif items=='scalar':
        s = sum(v.real for v in sp.diagonal(mat))
        res = sp.eye(mat.shape)*s
        return res
    else:
        res = sp.zeros(mat.shape)
        for (i,j) in items:
            res[i,j] = mat[i,j].real
        return res

def projectSemidef(mat):
    """Returns a pair (projmat,numNegativeEigenvalues)"""
    eigenvalues, evectors  = LA.eig(mat)

    numNegativeEigenvalues = 0
    
    #print 'before projection ' + str(mat) + '\r\n'
    Dplus = []
    #holds positive eigenvalues
    
    
    for value in eigenvalues:
        if value < 0:
            numNegativeEigenvalues+=1
            #print "Negative eigenvalue",value.real
        Dplus.append(max(value, 0))
    
    Dcapped = []
    for num in Dplus:
        Dcapped.append(min(num, 1000))
    D = sp.diag((Dcapped))

    newMat = evectors.dot(D).dot(evectors.transpose())
    
    return newMat,numNegativeEigenvalues


class MahalanobisPrimitiveSelector(PrimitiveSelector):
    def __init__(self,M):
        """A primitive selection function that uses a mahalanobis distance
        matrix (given as a 2D scipy array)"""
        self.M = M
        self.scoringFunction = lambda(x,y):mahalanobis(x,y,M)
    def save(self,f):
        """Saves the matrix to disk"""
        raise NotImplementedError()
    def load(self,f):
        """Loads the matrix from disk"""
        raise NotImplementedError()

class LocalMahalanobisPrimitiveSelector(PrimitiveSelector):
    def __init__(self,Ms=[]):
        """A primitive selection function that uses a mahalanobis distance
        matrix (given as a 2D scipy array)"""
        self.Ms = Ms
    def select(self,library,newProblem):
        """Overrides PrimitiveSelector with individual metrics"""
        assert len(self.Ms)==len(self.library.primitives),"Invalid local metric size, %d vs %d"%(len(self.Ms),len(self.library.primitives))
        fnew = library.getProblemFeatures(newProblem)
        dmin = 1e300
        best = None
        for M,p,f in zip(self.Ms,library.primitives,library.problemFeatures):
            d = mahalanobis(f,fnew,M)
            if d < dmin:
                dmin = d
                best = p
        return best
    def save(self,f):
        """Saves the matrices to disk"""
        raise NotImplementedError()
    def load(self,f):
        """Loads the matrices from disk"""
        raise NotImplementedError()
