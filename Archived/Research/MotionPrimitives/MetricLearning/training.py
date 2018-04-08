from primitivelibrary import Primitive,PrimitiveLibrary
import random
import scipy as sp
import scipy.linalg as LA
import time
from collections import defaultdict

class TrainingData:
    """
    Raw training data.
    
    Members:
    - problems: a list of training problems
    - costs: a dict from (primitiveIndex,problemIndex) pairs to
    lists of (adaptSuccessful,adaptTime,solutionCost).
    - failWeight, timeWeight, and costWeight: weights for mapping adaptation
      cost tuples to numeric values.  Penalize adapation failure, computation
      time, and solution cost.
    """
    def __init__(self,library,problems=None):
        """If the problems argument is not specified, it uses the problems in
        the library."""
        self.library = library
        if problems==None:
            self.problems = [p.problem for p in library.primitives]
            self.problemFeatures = [p.problemFeatures for p in library.primitives]
        else:
            self.problems = problems
            self.problemFeatures = [library.getProblemFeatures(p) for p in problems]
        self.costs = defaultdict(list)
        self.failWeight=10
        self.timeWeight=1
        self.costWeight=1
    def testAll(self):
        """Tests all pairs of primitives and problems"""
        self.costs = defaultdict(list)
        self.testAllMore()
    def testAllMore(self):
        """Adds a new test for each primitive,problem pair."""
        for i,p in enumerate(self.library.primitives):
            for j,q in enumerate(self.problems):
                self.test(i,j)
    def testRandom(self,number):
        """Tests a random subset of problems."""
        self.costs = defaultdict(list)
        self.testRandomMore(number)
    def testRandomMore(self,number=1):
        """Tests a new set of random problems."""
        for n in xrange(number):
            i = random.randrange(len(self.library.primitives))
            j = random.randrange(len(self.problems))
            self.test(i,j)
    def testQuasiRandom(self,numberPerPrimitive):
        """Tests a random subset of problems for each primitive."""
        self.costs = defaultdict(list)
        self.testQuasiRandomMore(numberPerPrimitive)
    def testQuasiRandomMore(self,numberPerPrimitive=1):
        """Tests a new set of random problems for each primitive."""
        for i in xrange(len(self.library.primitives)):
            for n in xrange(numberPerPrimitive):
                j = random.randrange(len(self.problems))
                self.test(i,j)
    def test(self,primitiveIndex,problemIndex):
        """Adds a new test for the indexed primitive and problem"""
        i,j = primitiveIndex,problemIndex
        p = self.library.primitives[primitiveIndex]
        q = self.problems[problemIndex]
        t0 = time.time()
        soln = self.library.adapt(p,q)
        t = time.time()-t0
        if soln == None:
            self.costs[(i,j)].append((False,t,0))
        else:
            cost = self.library.cost(soln,q)
            self.costs[(i,j)].append((True,t,cost))
    def numericCost(self,costTuple):
        """Maps an adaptation cost tuple to a numeric value. The numeric
        costs is assessed by penalizing failure by failWeight, time by
        timeWeight,and solution cost by costWeight. """
        return (self.failWeight if costTuple[0]==False else 0.0) + self.timeWeight*costTuple[1] + self.costWeight*costTuple[2] 
    def flattenedCosts(self):
        """Returns a sparse matrix mapping (i,j) entries to numeric costs.
        If multiple trials are run on the same (primitive,problem) pair,
        the numeric costs of each trial are averaged to give a single number."""
        flattened = dict()
        for (i,j),v in self.costs.iteritems():
            flatcosts = [self.numericCost(vi) for vi in v]
            flattened[(i,j)] = sum(flatcosts)/len(flatcosts)
        return flattened
    def save(self,f):
        """Given a file object f, writes out 5-item whitespace-separated lines
        consisting of elements
        
          primitive, problem, success (1 or 0), adapt time, solution cost

        """
        for (i,j),v in self.costs.iteritems():
            for vi in v:
                f.write(' '.join([str(i),str(j),str(int(vi[0])),str(vi[1]),str(vi[2])]))
                f.write('\n')
        return
    def load(self,f):
        """Given a file object f, reads in 5-item whitespace-separated lines
        consisting of elements
        
          primitive, problem, success (1 or 0), adapt time, solution cost

        and returns True on success.  An empty line signifies the end of the
        data.
        """
        self.costs = defaultdict(list)
        for line in f.readlines():
            items = line.split()
            if len(items)==0:
                #done
                return True
            assert len(items)==5,"Training data line must have 5 elements, got %d"%(len(items),)
            i = int(items[0])
            j = int(items[1])
            success = int(items[2]) != 0
            time = float(items[3])
            cost = float(items[4])
            self.costs[(i,j)].append((success,time,cost))            
        return True

class TrainingClusters:
    """Assigns each primitive in the library a set of close problems and
    far problems, in clusterClose and clusterFar, respectively.  Also stores
    which primitive each problem is is closest to, as well as row-major sparse
    matrix representations of training data."""
    def __init__(self,trainingData):
        self.trainingData = trainingData
        self.library = trainingData.library
        self.problems = trainingData.problems
        self.problemFeatures = trainingData.problemFeatures
        self.trainingMatrix = [dict() for p in self.library.primitives]
        self.trainingMatrixTranspose = [dict() for p in self.problems]
        self.clusterClose = [[] for p in self.library.primitives]
        self.clusterFar = [[] for p in self.library.primitives]
        self.problemToBestPrimitive = []
    def problemSpaceDims(self):
        """Returns the number of problem space dimensions"""
        return len(self.problemFeatures[0])
    def numClose(self):
        return sum(len(c) for c in self.clusterClose)
    def saveCosts(self,f):
        """Given a file object f, writes out 3-item whitespace-separated lines
        consisting of elements
        
          primitive, problem, cost

        """
        for i,row in enumerate(self.trainingMatrix):
            for j,c in row.iteritems():
                f.write(' '.join([str(i),str(j),str(c)]))
                f.write('\n')
        return
    def loadCosts(self,f):
        """Given a file object f, reads in 3-item whitespace-separated lines
        consisting of elements
        
          primitive, problem, cost

        and returns the matrix upon success.  (note that you will need to
        call assignBest or assignThreshold afterwards to set up the internal
        structures of this object instance)
        """
        costs = dict()
        for line in f.readlines():
            items = line.split()
            if len(items)==0:
                #done
                return costs
            assert len(items)==3,"Training data line must have 3 elements, got %d"%(len(items),)
            i = int(items[0])
            j = int(items[1])
            cost = float(items[2])
            costs[(i,j)] = cost
        return costs
    def assignBest(self,trainingCosts):
        """Given a training matrix trainingCosts (e.g., given by
        TrainingData.flattenedCosts), assigns problems to the one
        primitive they are closest to."""
        self.problemToBestPrimitive = [-1]*len(self.problems)
        for (i,j),c in trainingCosts.iteritems():
            self.trainingMatrix[i][j]=c
            self.trainingMatrixTranspose[j][i]=c
        for problem,primList in enumerate(self.trainingMatrixTranspose):
            if len(primList)==0: continue
            sortedCosts = sorted([(c,i) for (i,c) in primList.iteritems()])
            primitive = sortedCosts[0][1]
            self.clusterClose[primitive].append(problem)
            for (c,i) in sortedCosts[1:]:
                self.clusterFar[i].append(problem)
            self.problemToBestPrimitive[problem] = primitive
        return
    def assignThreshold(self,trainingCosts,epsAbs=0.0,epsRel=0.1):
        """Given a training matrix trainingCosts (e.g., given by
        TrainingData.flattenedCosts), assigns problems to clusters
        if their cost is no more than
             cmin + epsAbs + epsRel*(cmax-cmin)
        where [cmin,cmax] is the range
        of training costs for that problem. Default assigns top 10% of
        problems to a cluster."""
        self.problemToBestPrimitive = [-1]*len(self.problems)
        for (i,j),c in trainingCosts.iteritems():
            self.trainingMatrix[i][j]=c
            self.trainingMatrixTranspose[j][i]=c
        for problem,primList in enumerate(self.trainingMatrixTranspose):
            if len(primList)==0: continue
            sortedCosts = sorted([(c,i) for (i,c) in primList.iteritems()])
            primitive = sortedCosts[0][1]
            cmin,cmax = sortedCosts[0][0],sortedCosts[-1][0]
            cthresh = cmin + epsAbs+epsRel*(cmax-cmin)
            for (i,c) in primList.iteritems():
                if c <= cthresh:
                    self.clusterClose[i].append(problem)
                else:
                    self.clusterFar[i].append(problem)
            self.problemToBestPrimitive[problem] = primitive
        return
    def bestPrimitiveIndex(self,problemIndex):
        """Returns the best primitive for the given problem"""
        return self.problemToBestPrimitive[problemIndex]
    def candidateProblems(self,primitiveIndex):
        """Returns the set of problems for which there's ground truth cost
        data for the given primitive"""
        return self.trainingMatrix[primitiveIndex].keys()
    def candidatePrimitives(self,problemIndex):
        """Returns the set of primitives for which there's ground truth cost
        data for the given problem"""
        return self.trainingMatrixTranspose[problemIndex].keys()
    def clusterFeatures(self,primitiveIndex):
        """Returns the primitive's cluster's positive and negative training
        examples (Dpos,Dneg), where Dpos is a list of feature points for
        the positive examples and Dneg is a list of feature points for the
        negative examples"""
        Cclose = self.clusterClose[primitiveIndex]
        Cfar = self.clusterFar[primitiveIndex]
        Dclose = [self.problemFeatures[p] for p in Cclose]
        Dfar = [self.problemFeatures[p] for p in Cfar]
        return (Dclose,Dfar)
    def assignmentLoss(self,assignments):
        """For the given list of assignments from problems to primitives
        (a list of integers, of length len(self.problem)), computes the
        total loss."""
        c = 0.0
        for problem,primitive in enumerate(assignments):
            if primitive < 0:
                if len(self.trainingMatrixTranspose[problem])==0:
                    continue
                else:
                    raise ValueError("Problem %d not assigned anything"%(problem,))
            try:
                c += self.trainingMatrix[primitive][problem]
            except KeyError:
                print self.trainingMatrix[primitive]
                raise ValueError("Assignment %d->%d not in training data"%(primitive,problem))
        return c
    def assignmentError(self,assignments):
        """For the given list of assignments from problems to primitives
        (a list of integers, of length len(self.problem)), computes the
        total number of non-optimal primitives predicted."""
        c = 0
        for problem,primitive in enumerate(assignments):
            if self.bestPrimitiveIndex(problem)!=primitive:
                c += 1
        return c
    def assignmentClusterError(self,assignments):
        """For the given list of assignments from problems to primitives
        (a list of integers, of length len(self.problem)), computes the
        total number of non-optimal primitives predicted."""
        c = 0
        for problem,primitive in enumerate(assignments):
            if primitive < 0:
                if len(self.trainingMatrixTranspose[problem])==0:
                    continue
                else:
                    raise ValueError("Problem %d not assigned anything"%(problem,))
            if problem not in self.clusterClose[primitive]:
                c += 1
        return c
    
    def evalAssignment(self,problem,scoringFunction):
        """For a given scoring function g(primitive,fproblem) that maps candidate
        adaptations to a score (lower is better), returns the index of the
        assigned primitive chosen by minimizing the scoring function
        """
        primitiveDict = self.trainingMatrixTranspose[problem]
        if len(primitiveDict)==0: return -1
        problemFeatures = self.problemFeatures[problem]
        scores = [(scoringFunction(self.library.primitives[p],problemFeatures),p) for p in primitiveDict.iterkeys()]
        return min(scores)[1]
    
    def evalAssignments(self,scoringFunction):
        """For a given scoring function g(primitive,fproblem) that maps candidate
        adaptations to a score (lower is better), returns the assignment vector
        that assigns problems to best primitives.

        The return value is a list of length len(self.problem) containing integers.
        """
        assignment = [-1]*len(self.problems)
        for problem,primitiveDict in enumerate(self.trainingMatrixTranspose):
            if len(primitiveDict)==0: continue
            problemFeatures = self.problemFeatures[problem]
            scores = [(scoringFunction(self.library.primitives[p],problemFeatures),p) for p in primitiveDict.iterkeys()]
            assignment[problem] = min(scores)[1]
        return assignment
    def evalLoss(self,scoringFunction):
        """For a given scoring function g(fproblem,fprimitive) that maps features
        to a score (lower is better), returns the loss assuming the best-scoring
        primitive for each problem is selected."""
        return self.assignmentLoss(self.evalAssignments(scoringFunction))
    def evalError(self,scoringFunction):
        """For a given scoring function g(fproblem,fprimitive) that maps features
        to a score (lower is better), returns the error assuming the best-scoring
        primitive for each problem is selected."""
        return self.assignmentError(self.evalAssignments(scoringFunction))
    def evalClusterError(self,scoringFunction):
        """For a given scoring function g(fproblem,fprimitive) that maps features
        to a score (lower is better), returns the error assuming the best-scoring
        primitive for each problem is selected."""
        return self.assignmentClusterError(self.evalAssignments(scoringFunction))
    def minimumLoss(self):
        """Returns the minimum possible loss"""
        c = 0.0
        for problem,primitiveDict in enumerate(self.trainingMatrixTranspose):
            if len(primitiveDict)==0: continue
            c += min(primitiveDict.values())
        return c

def makeSyntheticData(dims,f,Nprim,Ntrain,Ntest,suboptimalityFactor=0.0,Nedges=None):
    """Returns a PrimitiveLibrary and two TrainingClusters instances
    from synthetic, randomly generated data.  Data is generated by taking
    randomly sampled point in a hypercube of dimension dims, and applying a
    warping function f(x) that produces the new feature vector for that point.
    The costs are simply the euclidean distances in the original space.
    Nprim is the number of primitives, Ntrain is the number of training points, and Ntest is the
    number of testing points."""
    #generate primitives
    xprim = [sp.random.random(dims) for i in xrange(Nprim)]
    fxprim = [Primitive(f(x),None) for x in xprim]
    P = PrimitiveLibrary(fxprim)

    #generate training data
    if Ntrain == 0:
        #self-training
        Ntrain = Nprim
        xtrain = xprim
        fxtrain = [p.problemFeatures for p in fxprim]
    else:
        xtrain = [sp.random.random(dims) for i in xrange(Ntrain)]
        fxtrain = [f(x) for x in xtrain]
    print "Generating training edges"
    trainMat = dict()
    for i,x in enumerate(xprim):
        if Nedges==None or Nedges >= Ntrain:
            for j,y in enumerate(xtrain):
                trainMat[(i,j)] = LA.norm(x-y)
        else:
            for j in random.sample(xrange(Ntrain),Nedges):
                trainMat[(i,j)] = LA.norm(x-xtrain[j])
    print "Assigning clusters"
    Dtrain = TrainingData(P,fxtrain)
    clustTrain = TrainingClusters(Dtrain)
    if suboptimalityFactor == 0.0:
        clustTrain.assignBest(trainMat)
    else:
        clustTrain.assignThreshold(trainMat,epsRel=suboptimalityFactor)

    #generate testing data
    xtest = [sp.random.random(dims) for i in xrange(Ntest)]
    fxtest = [f(x) for x in xtest]
    testMat = dict()
    print "Generating testing edges"
    for i,x in enumerate(xprim):
        for j,y in enumerate(xtest):
            testMat[(i,j)] = LA.norm(x-y)
    print "Assigning clusters"
    Dtest = TrainingData(P,fxtest)
    clustTest = TrainingClusters(Dtest)
    if suboptimalityFactor == 0.0:
        clustTest.assignBest(testMat)
    else:
        clustTest.assignThreshold(testMat,epsRel=suboptimalityFactor)
    return (P,clustTrain,clustTest)
