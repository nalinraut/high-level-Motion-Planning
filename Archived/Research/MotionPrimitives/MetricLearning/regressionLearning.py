from mahalanobis import *
from training import *
import scipy as sp
from scipy import linalg as LA
from LMNNLearning import projectSemidef

class RegressionLearning:
    """Learns via regression on costs, rather than equivalence classes.
    """
    def __init__(self,diag=True):
        self.diag = diag
        self.M = None
        pass
    def run(self,clusters):
        """Input:
        - clusters: a TrainingClusters instance
        """
        trainValues = []
        for i,c in enumerate(clusters.trainingMatrix):
            for j,v in c.iteritems():
                trainValues.append((sp.subtract(clusters.problemFeatures[j],clusters.library.primitives[i].problemFeatures),v))
        Arows = []
        b = []
        print len(trainValues)*(len(trainValues[0][0])**2)
        if len(trainValues)*len(trainValues[0][0])**2 > 50000000:
            print "Warning, very large entries, using diagonal learning"
            self.diag = True
        if self.diag:
            for (v,c) in trainValues:
                Arows.append([vi*vi for vi in v])
                b.append(c)
            x,resid,rank,sv = LA.lstsq(sp.array(Arows),sp.array(b))
            self.M = sp.diag(x)
        else:
            for (v,c) in trainValues:
                Arows.append(sum([[vi*vj for vi in v] for vj in v],[]))
                b.append(c)
            x,resid,rank,sv = LA.lstsq(sp.array(Arows),sp.array(b))
            n = len(trainValues[0][0])
            self.M = sp.array([x[i*n:(i+1)*n].tolist() for i in range(n)])
        return 

    def solution(self):
        """Returns the current solution."""
        return self.M


