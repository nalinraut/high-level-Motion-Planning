def distanceSquared(a,b):
    return sum((ai-bi)**2 for (ai,bi) in zip(a,b))

class Primitive:
    def __init__(self,problem,solution,data=None,problemFeatures=None,solutionFeatures=None):
        self.problem = problem
        self.solution = solution
        self.data = data
        self.problemFeatures = problemFeatures
        self.solutionFeatures = solutionFeatures

class PrimitiveLibrary:
    """A primitive library is a collection of primitives, which are each
    (problem,solution) pairs.  The default implementation treats problems
    as points in a Euclidean vector space, i.e., problems are identical to
    their feature vectors."""
    def __init__(self,primitives=None):
        if primitives==None:
            self.primitives = []
        else:
            self.set(primitives)
    def set(self,primitives):
        self.primitives = primitives
        for p in self.primitives:
            if p.problemFeatures == None:
                p.problemFeatures = self.getProblemFeatures(p.problem)
            if p.solutionFeatures == None:
                p.solutionFeatures = self.getSolutionFeatures(p.problem)
    def append(self,p):
        if p.problemFeatures == None:
            p.problemFeatures = self.getProblemFeatures(p.problem)
        if p.solutionFeatures == None:
            p.solutionFeatures = self.getSolutionFeatures(p.problem)        
        self.primitives.append(p)
    def getProblemFeatures(self,problem):
        """Returns a fixed-length feature vector for a problem. Subclasses
        should override this if they wish to store structured problem
        definitions."""
        return problem
    def getSolutionFeatures(self,solution):
        """Returns a fixed-length feature vector for a solution. Subclasses
        should override this if they wish to include solution features."""
        return []
    def adapt(self,primitive,newProblem):
        """Returns a solution that adapts the existing primitive to the given
        newProblem.  Subclasses should override this for a nontrivial
        adaptation.  Should return None if adaptation failed"""
        return primitive.solution
    def cost(self,solution,problem):
        """Returns the cost of the given solution for the given problem"""
        return 0

class PrimitiveSelector:
    """Base class for selection routines.  Default implementation selects
    primitives using Nearest Neighbors, measuring closeness using
    Euclidean distance.  Can modify self.selectionScore to produce a different
    nearest neighbors metric."""
    def __init__(self):
        self.selectionScore = lambda p,probFeatures:distanceSquared(p.problemFeatures,probFeatures)
        return
    def select(self,library,newProblem):
        """Returns a primitive that is a good candidate for adaptation to the
        new problem.  Subclasses should override this for non-NN selection.
        New scoring functions can be added by changing the selectionScore
        member to another function.

        Returns a pair (i,p) containing both the index and the primitive
        """
        fnew = library.getProblemFeatures(newProblem)
        best = None
        dmin = 1e300
        for i,p in enumerate(library.primitives):
            d = self.selectionScore(p,fnew)
            if d < dmin:
                dmin = d
                best = i,p
        return best
