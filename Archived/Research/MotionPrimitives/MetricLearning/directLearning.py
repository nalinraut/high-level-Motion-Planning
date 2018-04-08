from mahalanobis import *
from training import *
import scipy as sp
import math
from scipy import linalg as LA

class GlobalDirectLearning:
    """Directly minimizes loss of assignment rather than using clusters.
    loss(M) = sum{j=1,...,N} cost(primitive[arg min_i=1,..,M score(M,primitive[i],problem[j])])
    If we let z_ij be the indicator that the the minimizer of the interior term for
    value j is i (in range [0,1]).
    
    We have the following optimization:
    min_{M,z_ij} sum{j=1,...,N} sum{i=1,...,M} z_ij*cost(p[i],q[j])
      such that:
    sum_k z_kj * score(M,p[k],q[j]) <= score(M,p[i],q[j]) for all i,j
    sum_k z_kj = 1, for all j
    z_ij >= 0 for all i,j

    KKT conditions don't give anything useful, it is still enumerating the
    optimal assignment.

    Enumeration method.  TODO

    Soft-arg-min method:
    For a given weight w, 
    z_ij(w,M) = exp(-s_ij(M)*w)/sum_k exp(-s_kj(M)*w)
    with s_ij(M) = score(M,p[i],q[j])
    As w approaches infinity, this goes to the desired indicator function.

    For w = 1,2,4,..., solve a step of the optimization
    min_{M} sum{j=1,...,N} sum{i=1,...,M} z_ij(w,M)*cost(p[i],q[j])

    d/dM z_ij(w,M) = -s_ij'(M)*w*z_ij(w,M) + [sum_k s_kj'(M)*w*exp(s_kj(M)*w)]/[sum_k exp(s_kj(M)*w)]*z_ij(w,M)
        = [-s_ij'(M) + sum_k s_kj'(M)*z_kj(M)]*w*z_ij(w,M)
    s_ij'(M) is constant
    df(w,M)/dM = sum{j=1,...,N} sum{i=1,...,M} [s_ij' + sum_k s_kj'*z_kj(M)]*w*z_ij(w,M)*c_ij
    """
    def __init__(self,method='softmargin',mask=None):
        self.method=method
        self.M = None
        self.mask = mask
        pass
    def run(self,clusters):
        """Input:
        - clusters: a TrainingClusters instance
        """
        if self.method=='softmargin':
            self.runSoftMargin(clusters)
        elif self.method=='softargmin':
            self.runSoftArgMin(clusters)
        elif self.method=='enumerate':
            self.runEnumerate(clusters)
        else:
            raise ValueError("Invalid method "+self.method)

    def solution(self):
        """Returns the current solution."""
        return self.M

    def runSoftArgMin(self,clusters,alpha=None,maxiters=100):
        if self.M == None:
            n = clusters.problemSpaceDims()
            self.M = sp.eye(n)
            #whiten the training data along each axis
            for i in xrange(self.M.shape[0]):
                vi = sp.var([f[i] for f in clusters.problemFeatures])
                if vi != 0:
                    self.M[i,i] = 1.0/vi
        self.xtol = 1e-8
        self.softAssignments = [[0.0]*len(v) for v in clusters.trainingMatrixTranspose]
        w = 0.01
        loss,grad = self.softAssignmentGrad(clusters,self.M,w)
        self.alpha = 0.5 / LA.norm(grad)
        while maxiters > 0:
            if maxiters % 1 == 0:
                print "w",w,"optimization loss",loss,"true loss",clusters.evalLoss(lambda p,q:mahalanobis(p.problemFeatures,q,self.M)),"step size",self.alpha
            maxiters -= 1
            Mnew,numNegative  = projectSemidef(self.M - self.alpha*grad)
            print numNegative,"Negative eigenvalues"
            if LA.norm(self.M-Mnew) < self.xtol:
                return "converged"
            if sp.count_nonzero(Mnew)==0:
                self.alpha *= 0.5
            else:
                futureLoss,futureGrad = self.softAssignmentGrad(clusters,Mnew,w)
                if futureLoss <= loss:
                    print "Taking step"
                    self.alpha *= 1.1
                    loss = futureLoss
                    self.M = Mnew
                    grad = mask(futureGrad,self.mask)
                else:
                    print "Shrinking step size"
                    self.alpha *= 0.5
                print "Gradient norm",LA.norm(futureGrad)
                if maxiters % 10 == 0 or LA.norm(futureGrad) < 1e-2:
                    w *= 1.5
                    loss,grad = self.softAssignmentGrad(clusters,self.M,w)
    
        return "maxiters reached"
        
    def softAssignmentGrad(self,clusters,M,w):
        n = clusters.problemSpaceDims()
        
        #compute soft assignments
        assignments = self.softAssignments
        for (j,v) in enumerate(clusters.trainingMatrixTranspose):
            #compute distances
            assert len(assignments[j])==len(v)
            if len(assignments[j])==0: continue
            for (i,(primIndex,cost)) in enumerate(v.iteritems()):
                assignments[j][i] = mahalanobis(clusters.library.primitives[primIndex].problemFeatures,clusters.problemFeatures[j],M)
            #divide through by the sum of exp(-w*d) for assignments of problem j
            #avoid underflow if all d are large
            closest = min(assignments[j])
            for i,v in enumerate(assignments[j]):
                assignments[j][i] = math.exp(-w*(v-closest))
            scale = 1.0/sum(assignments[j])
            for i,v in enumerate(assignments[j]):
                assignments[j][i] *= scale
        #compute loss
        loss = 0
        grad = sp.zeros((n,n))
        for j,primList in enumerate(clusters.trainingMatrixTranspose):
            assert len(assignments[j])==len(primList)
            if len(assignments[j])==0: continue
            aveGrad = sp.zeros((n,n))
            for k,(prim,c) in enumerate(primList.iteritems()):
                diff = sp.subtract(clusters.problemFeatures[j],clusters.library.primitives[prim].problemFeatures)
                aveGrad += assignments[j][k]*sp.outer(diff,diff)
            for i,(prim,c) in enumerate(primList.iteritems()):
                loss += c*assignments[j][i]
                diff = sp.subtract(clusters.problemFeatures[j],clusters.library.primitives[prim].problemFeatures)
                grad += c*assignments[j][i]*w*(aveGrad-sp.outer(diff,diff))
        #scalefactor = len(clusters.problemFeatures)
        scalefactor = 1
        return loss/scalefactor,grad/scalefactor 


    def runSoftMargin(self,clusters,alpha=None,maxiters=100):
        if self.M == None:
            n = clusters.problemSpaceDims()
            self.M = sp.eye(n)
            #whiten the training data along each axis
            for i in xrange(self.M.shape[0]):
                vi = sp.var([f[i] for f in clusters.problemFeatures])
                if vi != 0:
                    self.M[i,i] = 1.0/vi
        self.xtol = 1e-8
        self.weights = [1.0]*len(clusters.trainingMatrixTranspose)
        for (i,v) in enumerate(clusters.trainingMatrixTranspose):
            if len(v)==0: continue
            self.weights[i] = sum(v.values())/len(v.values())
        loss,grad = self.softMarginGrad(clusters,self.M)
        grad = mask(grad,self.mask)
        if alpha == None:
            self.alpha = 0.5 / sp.linalg.norm(grad)
            #self.alpha = 1.0 / len(clusters.problemFeatures)
        else:
            self.alpha = alpha / len(clusters.problemFeatures)
            
        M = self.M
        self.loss = clusters.evalLoss(lambda p,q:mahalanobis(p.problemFeatures,q,M))
        while maxiters > 0:
            if maxiters % 1 == 0:
                print "optimization loss",loss,"true loss",clusters.evalLoss(lambda p,q:mahalanobis(p.problemFeatures,q,M))
            maxiters -= 1
            Mnew,numNegative  = projectSemidef(M - self.alpha*grad)
            #keep from getting degenerate
            Mnew = Mnew / LA.norm(Mnew)
            if LA.norm(self.M-Mnew) < self.xtol:
                return "converged"
            if sp.count_nonzero(Mnew)==0:
                self.alpha *= 0.5
            else:
                futureLoss,futureGrad = self.softMarginGrad(clusters,Mnew)
                if futureLoss <= loss:
                    self.alpha *= 1.1
                else:
                    self.alpha *= 0.5
                
                loss = futureLoss
                M = Mnew
                grad = mask(futureGrad,self.mask)
                trueloss = clusters.evalLoss(lambda p,q:mahalanobis(p.problemFeatures,q,M))
                if trueloss < self.loss:
                    self.loss = trueloss
                    self.M = M
        return "maxiters reached"

    def softMarginGrad(self,clusters,M):
        n = clusters.problemSpaceDims()
        loss = 0
        grad = sp.zeros((n,n))
        for j,primList in enumerate(clusters.trainingMatrixTranspose):
            best = clusters.bestPrimitiveIndex(j)
            scoreBest = mahalanobis(clusters.library.primitives[best].problemFeatures,clusters.problemFeatures[j],M)
            minScore = scoreBest
            minIndex = best
            for i in primList.iterkeys():
                score = mahalanobis(clusters.library.primitives[i].problemFeatures,clusters.problemFeatures[j],M)
                if score < minScore:
                    minScore = score
                    minIndex = i
            #this assert doesn't discriminate between primitives assigned the same score
            #assigned = clusters.evalAssignment(j,lambda p,q:mahalanobis(p.problemFeatures,q,M))
            #assert(minIndex == assigned),"Assignment for problem %d doesnt appear correct: %d vs %d, cost %g vs %g"%(j,minIndex,assigned,minScore,mahalanobis(clusters.library.primitives[assigned].problemFeatures,clusters.problemFeatures[j],M))
            if minIndex != best:
                loss += self.weights[j]*(scoreBest - minScore)
                diff = sp.subtract(clusters.library.primitives[best].problemFeatures,clusters.problemFeatures[j])
                grad += self.weights[j]*sp.outer(diff,diff)
                diff = sp.subtract(clusters.library.primitives[minIndex].problemFeatures,clusters.problemFeatures[j])
                grad -= self.weights[j]*sp.outer(diff,diff)
        #scalefactor = len(clusters.problemFeatures)
        scalefactor = 1
        return loss/scalefactor,grad/scalefactor 

    def runEnumerate(self,clusters):
        """Approximate method -- examines all differences as directions for
        optima.
        |E|^2 running time.
        """
        #gather all instances centered at their respective centers
        print "Enumerating..."
        candidateV = []
        for i,p in enumerate(clusters.library.primitives):
            for j,c in clusters.trainingMatrix[i].iteritems():
                candidateV.append((c,sp.subtract(p.problemFeatures,clusters.problemFeatures[j])))
        candidateV = sorted(candidateV)
        n = clusters.problemSpaceDims()
        self.M = sp.eye(n)
        bestLoss = self.branchedLoss(clusters,self.M,1e300)
        for c,v in candidateV:
            Mtemp = sp.outer(v,v)
            loss = self.branchedLoss(clusters,Mtemp,bestLoss)
            if loss < bestLoss:
                self.M = Mtemp
                bestLoss = loss
                print "Got improvement",loss
        return

    def branchedLoss(self,clusters,M,branch):
        c = 0.0
        for problem,primitiveDict in enumerate(clusters.trainingMatrixTranspose):
            if len(primitiveDict)==0: continue
            problemFeatures = clusters.problemFeatures[problem]
            scores = [(mahalanobis(clusters.library.primitives[p].problemFeatures,problemFeatures,M),p) for p in primitiveDict.iterkeys()]
            primitive = min(scores)[1]
            c += primitiveDict[primitive]
            if c >= branch:
                return c
        return c
