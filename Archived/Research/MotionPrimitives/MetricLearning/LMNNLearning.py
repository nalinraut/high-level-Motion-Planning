from mahalanobis import *
from training import *
import scipy as sp
import pickle
from datetime import datetime
from collections import defaultdict
import time

class GlobalLMNNLearning:
    """Solves the LMNN problem for a global metric M.

    Parameters:
    - mu: a regularization term
    - alpha: the start step size (None for autodetect)
    - xtol: the step size threshold for determining convergence
    - logPrefix: a logging prefix prefix,
    - startM: an optional start matrix startM.  If startM is not
      given, an identity matrix is used
    - maxiter: a maximum number of iterations
    """
    def __init__(self,mu=0.5,alpha=None,startM=None,maxiter=100,logPrefix='output',mask=None):
        self.mu = mu
        self.alpha = alpha
        self.xtol = 1e-8
        self.maxiter = maxiter
        self.M = startM
        self.Mbest = startM
        self.loss = None
        self.bestLoss = None
        self.logPrefix = logPrefix
        self.mask = mask
    def run(self,clusters):
        """Runs to local convergence or maxiter iterations.
        - clusters: a TrainingClusters instance
        """
        maxiter = self.maxiter
        if self.M == None:
            self.M = sp.eye(clusters.problemSpaceDims())
            #whiten the training data along each axis
            """
            for i in xrange(self.M.shape[0]):
                vi = sp.var([f[i] for f in clusters.problemFeatures])
                if vi != 0:
                    self.M[i,i] = 1.0/vi
            """
        #impostorupdatefreq = 10
        impostorupdatefreq = 1
        with open(self.logPrefix+'/logs/globaliter.txt', 'w') as stop:
            stop.write('global ' + self.logPrefix + ' beginning ' + str(datetime.now()) + '\r\n')

        #sortallneighborsglobal(clusters, Mprev)
        zeromat = sp.zeros(self.M.shape)
        t0 = time.time()    
        currLoss, currGrad, currimpostors = lossGradientFull(clusters, self.mu, self.M, self.logPrefix)
        currGrad = mask(currGrad,self.mask)
        if self.alpha == None:
            self.alpha = 0.5/sp.linalg.norm(currGrad)
        print "Full gradient time",time.time()-t0
        self.Mbest = self.M
        self.bestLoss = self.loss = currLoss
        lastimpostorupdate = maxiter
        while maxiter > 0:
            print "Loss:",self.loss,"gradient norm",sp.linalg.norm(currGrad),"step size",self.alpha
            #print gradient diagonal?
            #print [currGrad[i,i] for i in xrange(currGrad.shape[0])]
            t0 = time.time()
            maxiter -=1
            Mnew,numNegative = projectSemidef(self.M - self.alpha*currGrad)
            #line search
            while sp.count_nonzero(Mnew)==0:
                maxiter -= 1
                self.alpha = self.alpha*.7
                Mnew,numNegative = projectSemidef(self.M - self.alpha*currGrad)
                #with open(prefix+'/logs/globaliter.txt', 'a') as stop:
                    #stop.write(str(maxiter) + ' nonzeroing at ' + str(datetime.now()) + '\r\n')
            print numNegative,"Negative eigenvalues"
            if maxiter <= lastimpostorupdate - impostorupdatefreq:
                lastimpostorupdate = maxiter
                with open(self.logPrefix+'/logs/globaliter.txt', 'a') as stop:
                    stop.write(str(maxiter) + ' full at ' + str(datetime.now()) + '\r\n')
                with open(self.logPrefix+'/dumps/M.txt', 'wb') as M:
                    pickle.dump(self.M, M)
                futureLoss, futureGrad, currimpostors = lossGradientFull(clusters, self.mu, Mnew, self.logPrefix)
                if futureLoss < self.bestLoss:
                    self.Mbest = Mnew
                    self.bestLoss = futureLoss
            else:
                    futureLoss, futureGrad = lossGradientEstimate(clusters, self.mu, Mnew, currimpostors, self.logPrefix)

            if futureLoss < self.loss:
                self.alpha *= 1.2
            else:
                self.alpha *= 0.5
            #print "new alpha:",alpha
            Mchange = sp.linalg.norm(self.M-Mnew)
            self.M = Mnew
            currGrad = mask(futureGrad,self.mask)
            self.loss = futureLoss
            if Mchange < self.xtol:
                return "converged"
        return "maxiters reached"

    def solution(self):
        """Returns the current solution."""
        return self.Mbest


def lossGradientFull(clusters, mu, M, prefix):
    #print 'full global update beginning for '+ prefix + ' at ' + str(datetime.now()) + '\r\n'
    knownimpostors = []
    impostorCount = 0
    #compute impostor lists
    for i,(close,far) in enumerate(zip(clusters.clusterClose,clusters.clusterFar)):
        primitiveFeatures = clusters.library.primitives[i].problemFeatures
        fclose,ffar = clusters.clusterFeatures(i)
        dclose = [mahalanobis(primitiveFeatures,f,M) for f in fclose]
        dfar = [mahalanobis(primitiveFeatures,f,M)-1.0 for f in ffar]
        allSorted = sorted(zip(dclose,close,[True]*len(close)) + zip(dfar,far,[False]*len(far)))
        #count the number of times a close point is farther than a far point,
        #and how many times a far point is closer than a close point
        ccnt = dict()
        fcnt = dict()
        numFar = 0
        for d,idx,isclose in allSorted:
            if not isclose:
                numFar += 1
            elif numFar > 0:
                ccnt[idx] = numFar
                impostorCount += numFar
        numClose = 0
        for d,idx,isclose in reversed(allSorted):
            if isclose:
                numClose += 1
            elif numClose > 0:
                fcnt[idx] = numClose
        knownimpostors.append((ccnt,fcnt))
    print impostorCount,"Impostors"
    totalloss, gradient = lossGradientEstimate(clusters,mu,M,knownimpostors,prefix)
    return totalloss, gradient, knownimpostors

def lossGradientEstimate(clusters, mu, M, currimpostors, prefix):
    #print 'estimated global update beginning for ' + prefix + ' at '+ str(datetime.now()) + '\r\n'
    
    totalloss = 0
    gradient = sp.zeros(M.shape)
    closecount = 0
    for i,(close,far) in enumerate(zip(clusters.clusterClose,clusters.clusterFar)):
        primitiveFeatures = clusters.library.primitives[i].problemFeatures
        fclose,ffar = clusters.clusterFeatures(i)
        myimpostors = currimpostors[i]
        
        for c in fclose:
            diffn = sp.subtract(primitiveFeatures, c)
            totalloss += (1.0-mu)*mahalanobis(primitiveFeatures, c, M)
            gradient += (1.0-mu)*sp.outer(diffn,diffn)

        #assumes impostors are always updated
        ccnt, fcnt = myimpostors
        for (nbr,cnt) in ccnt.iteritems():
            ndist = mahalanobis(primitiveFeatures,clusters.problemFeatures[nbr], M)
            diffn = sp.subtract(primitiveFeatures,clusters.problemFeatures[nbr])
            totalloss += mu * (ndist + 1) * cnt
            gradient += mu* sp.outer(diffn,diffn) * cnt
            closecount += cnt
        for (imp,cnt) in fcnt.iteritems():
            sdist = mahalanobis(primitiveFeatures,clusters.problemFeatures[imp], M)
            diffi = sp.subtract(primitiveFeatures,clusters.problemFeatures[imp])
            totalloss -= mu * sdist * cnt
            gradient -= mu* sp.outer(diffi,diffi) * cnt
    scalefactor = sum(len(v) for v in clusters.clusterClose)
    print closecount,"close impostors","scale factor",scalefactor
    #print numnonimpostors,"Not impostors anymore"
    #raw_input()
    #print 'estimated global update ending for '+ prefix + ' at ' + str(datetime.now()) + '\r\n'
    return totalloss/scalefactor, gradient/scalefactor

                        
            

