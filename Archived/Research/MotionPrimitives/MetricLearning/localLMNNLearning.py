from mahalanobis import *
from training import *
import scipy as sp
from scipy import linalg as LA
import pickle
from datetime import datetime
from collections import defaultdict
import time


class LocalLMNNLearning:
    """Solves the LMNN problem for local metrics Ms.

    Parameters:
    - mu: a regularization term
    - alpha: the start step size (None for autodetect)
    - xtol: the step size threshold for determining convergence
    - logPrefix: a logging prefix,
    - maxiter: a maximum number of iterations
    - startM: an optional start matrix startM.  If startM is not
      given, an identity matrix is used
    """
    def __init__(self,mu=0.5,alpha=None,maxiter=100,startMs=None,logPrefix='output',mask=None):
        self.mu = mu
        self.alpha = alpha
        self.xtol = 1e-8
        self.Ms = startMs
        self.Mbests = startMs
        self.maxiter = maxiter
        self.loss = None
        self.bestLoss = None
        self.logPrefix = logPrefix
        self.mask = mask
    def run(self,clusters):
        """Runs to local convergence or maxiters iterations.
        - clusters: a TrainingClusters instance
        """
        maxiter = self.maxiter
        if self.Ms == None:
            M = sp.eye(clusters.problemSpaceDims())
            #whiten the training data along each axis
            for i in xrange(M.shape[0]):
                vi = sp.var([f[i] for f in clusters.problemFeatures])
                if vi != 0:
                    M[i,i] = 1.0/vi
            self.Ms = [M for i in clusters.library.primitives]
        #impostorupdatefreq = 10
        impostorupdatefreq = 1
        with open(self.logPrefix+'/logs/localiter.txt', 'w') as stop:
            stop.write('local LMNN ' + self.logPrefix + ' beginning ' + str(datetime.now()) + '\r\n')

        #sortallneighborsglobal(clusters, Mprev)
        zeromat = sp.zeros(self.Ms[0].shape)
        t0 = time.time()    
        self.loss, currGrad, currimpostors = lossGradientFull(clusters, self.mu, self.Ms, self.logPrefix)
        for i,g in enumerate(currGrad):
            currGrad[i] = mask(g,self.mask)
        self.bestLoss = self.loss
        self.Mbests = self.Ms
        if self.alpha == None:
            self.alpha = 1.0/sum(sp.linalg.norm(g) for g in currGrad)
        print "Full gradient time",time.time()-t0
        lastimpostorupdate = maxiter
        while maxiter > 0:
            print "Loss:",self.loss,"gradient norm",sum(sp.linalg.norm(g) for g in currGrad),"step size",self.alpha
            t0 = time.time()
            maxiter -=1
            Mnew = []
            for M,g in zip(self.Ms,currGrad):
                Mproj,numNegative = projectSemidef(M - self.alpha*g)
                Mnew.append(Mproj)
            #line search
            while all(sp.count_nonzero(M)==0 for M in Mnew):
                maxiter -= 1
                self.alpha = self.alpha*.7
                Mnew = []
                for M,g in zip(self.Ms,currGrad):
                    Mproj,numNegative = projectSemidef(M - self.alpha*g)
                    Mnew.append(Mproj)
                #with open(prefix+'/logs/globaliter.txt', 'a') as stop:
                    #stop.write(str(maxiter) + ' nonzeroing at ' + str(datetime.now()) + '\r\n')
            if maxiter <= lastimpostorupdate - impostorupdatefreq:
                lastimpostorupdate = maxiter
                with open(self.logPrefix+'/logs/localiter.txt', 'a') as stop:
                    stop.write(str(maxiter) + ' full at ' + str(datetime.now()) + '\r\n')
                #with open(self.logPrefix+'/dumps/Ms.txt', 'wb') as M:
                #    pickle.dump(self.Ms, M)
                futureLoss, futureGrad, currimpostors = lossGradientFull(clusters, self.mu, Mnew, self.logPrefix)
                if futureLoss < self.bestLoss:
                    self.Mbests = Mnew
                    self.bestLoss = futureLoss
            else:
                futureLoss, futureGrad = lossGradientEstimate(clusters, self.mu, Mnew, currimpostors, self.logPrefix)

            if futureLoss < self.loss:
                self.alpha *= 1.2
            else:
                self.alpha *= 0.5
            #print "new alpha:",alpha
            Mchange = sum(sp.linalg.norm(M-Mi) for (M,Mi) in zip(self.Ms,Mnew))
            self.Ms = Mnew
            currGrad = [mask(g,self.mask) for g in futureGrad]
            self.loss = futureLoss
            if Mchange < self.xtol:
                return "converged"
        return "maxiters reached"

    def solution(self):
        """Returns the current solution."""
        return self.Mbests


def lossGradientFull(clusters, mu, Ms, prefix):
    #print 'full global update beginning for '+ prefix + ' at ' + str(datetime.now()) + '\r\n'
    knownimpostors = []
    #compute impostor lists
    for i,(close,far) in enumerate(zip(clusters.clusterClose,clusters.clusterFar)):
        primitiveFeatures = clusters.library.primitives[i].problemFeatures
        fclose,ffar = clusters.clusterFeatures(i)
        Mi = Ms[i]
        dclose = [mahalanobis(primitiveFeatures,f,Mi) for f in fclose]
        dfar = [mahalanobis(primitiveFeatures,f,Ms[clusters.bestPrimitiveIndex(findex)])-1 for f,findex in zip(ffar,far)]
        allSorted = sorted(zip(dclose,close,[True]*len(close)) + zip(dfar,far,[False]*len(far)))
        #count the number of times a close point is farther than a far point,
        #and how many times a far point is closer than a close point
        ccnt = defaultdict(int)
        fcnt = defaultdict(int)
        numFar = 0
        for d,idx,isclose in allSorted:
            if not isclose:
                numFar += 1
            elif numFar > 0:
                ccnt[idx] = numFar
        numClose = 0
        for d,idx,isclose in reversed(allSorted):
            if isclose:
                numClose += 1
            elif numClose > 0:
                fcnt[idx] = numClose
        knownimpostors.append((ccnt,fcnt))
    totalloss, gradient = lossGradientEstimate(clusters,mu,Ms,knownimpostors,prefix)
    return totalloss, gradient, knownimpostors

def lossGradientEstimate(clusters, mu, Ms, currimpostors, prefix):
    #print 'estimated global update beginning for ' + prefix + ' at '+ str(datetime.now()) + '\r\n'
    
    totalloss = 0
    gradient = [sp.zeros(M.shape) for M in Ms]
    numnonimpostors = 0
    for i,(close,far) in enumerate(zip(clusters.clusterClose,clusters.clusterFar)):
        primitiveFeatures = clusters.library.primitives[i].problemFeatures
        fclose,ffar = clusters.clusterFeatures(i)
        myimpostors = currimpostors[i]
        Mi = Ms[i]
        
        for c in fclose:
            ndist = mahalanobis(primitiveFeatures, c, Mi)
            diffn = sp.subtract(primitiveFeatures, c)
            ngrad = sp.outer(diffn,diffn)
            totalloss += (1.0-mu)*ndist
            gradient[i] += (1.0-mu)*ngrad

        #assumes impostors are always updated
        ccnt, icnt = myimpostors
        for (nbr,cnt) in ccnt.iteritems():
            ndist = mahalanobis(primitiveFeatures,clusters.problemFeatures[nbr], Mi)
            diffn = sp.subtract(primitiveFeatures,clusters.problemFeatures[nbr])
            ngrad = sp.outer(diffn,diffn)
            totalloss += mu * (ndist + 1) * cnt
            gradient[i] += mu* ngrad * cnt
        for (imp,cnt) in icnt.iteritems():
            k = clusters.bestPrimitiveIndex(imp)
            M = Ms[k]
            sdist = mahalanobis(primitiveFeatures,clusters.problemFeatures[imp], M)
            diffi = sp.subtract(primitiveFeatures,clusters.problemFeatures[imp])
            igrad = sp.outer(diffi,diffi)
            totalloss -= mu * sdist * cnt
            gradient[k] -= mu* igrad * cnt
    scalefactor = sum(len(v) for v in clusters.clusterClose)
    #print numnonimpostors,"Not impostors anymore"
    #raw_input()
    #print 'estimated global update ending for '+ prefix + ' at ' + str(datetime.now()) + '\r\n'
    return totalloss/scalefactor, [g/scalefactor for g in gradient]

                        
            
                        
                           
