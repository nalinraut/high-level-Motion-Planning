import sys
import math
import csv
import os
import cPickle
import itertools
import random
from optparse import OptionParser

class PrimitiveLibrary:
    """A class to help with the loading, saving, generating, and labeling
    of motion primitives.

    The library is initialized with a problemSpace object (could be a class
    or a module) that generates problem objects and optimizers.
    A problem object p is required to store its parameters in p.params.
    The problemSpace is required to implement the functions:
        - sampleProblem(): samples a problem object at random from the space
          of problems.
        - genProblem(params): returns a problem object given its parameters
        - globalOptimizer(problem): returns an object that will generate a
          globally optimal or approximately optimal solution for the given
          problem when its run() method is called.
        - adaptOptimizer(problem,seedpparams,seedprimitive): returns an object
          that will generate a locally optimal solution for the given
          problem starting from the seed primitive when its run(seed) method
          is called.  The problem parameters generating the seed primitive
          are given in seedpparams.
        - features(params): return a feature list corresponding to params
        - featureNames(params): returns a list of feature names corresponding
          to the list of features returned by features()
    
    File naming is problemX.dat, primitiveX.dat where X is the primitve
    index+1.  The cPickle module is used to save problems and primitives
    as needed."""
    def __init__(self,problemSpace):
        self.problemSpace = problemSpace
        self.primitives = []
    
    def plan_from_scratch(self,problem):
        opt = self.problemSpace.globalOptimizer(problem)
        opt.run()
        return opt.getSolution()

    def adapt(self,srcproblem,srcprimitive,destproblem,log=False):
        opt = self.problemSpace.adaptOptimizer(self.problemSpace.genProblem(destproblem),srcproblem,srcprimitive)
        if log: opt.beginLogValues()
        opt.run()
        return (opt.getSolution(),opt.getLog())

    def generate(self,n,savedir='primitives'):
        """Generates up to n primitives from scratch given the problem space's
        problem sampler. If savedir is provided, then each primitive is saved
        to savedir right after it is generated."""
        for iters in xrange(n):
            p = self.problemSpace.sampleProblem()
            soln = self.plan_from_scratch(p)
            if soln != None:
                self.primitives.append((p.params,soln))
                if savedir:
                    problem,primitive = self.primitives[-1]
                    problemfn = os.path.join(savedir,'problem%d.dat'%(len(self.primitives),))
                    primitivefn = os.path.join(savedir,'primitive%d.dat'%(len(self.primitives),))
                    print "Saving primitive to",problemfn,",",primitivefn
                    fproblem = open(problemfn,'w')
                    fprimitive = open(primitivefn,'w')
                    cPickle.dump(problem,fproblem)
                    cPickle.dump(primitive,fprimitive)
                    fproblem.close()
                    fprimitive.close()

    def reoptimize(self):
        newprimitives = []
        for (problem,primitive) in self.primitives:
            np,costs = self.adapt(problem,primitive,problem,False)
            newprimitives.append((problem,np))
        self.primitives = newprimitives

    def save_primitives(self,dir='primitives'):
        for i,(problem,primitive) in enumerate(self.primitives):
            problemfn = os.path.join(dir,'problem%d.dat'%(i+1,))
            primitivefn = os.path.join(dir,'primitive%d.dat'%(i+1,))
            print "Saving primitive to",problemfn,",",primitivefn
            fproblem = open(problemfn,'w')
            fprimitive = open(primitivefn,'w')
            cPickle.dump(problem,fproblem)
            cPickle.dump(primitive,fprimitive)
            fproblem.close()
            fprimitive.close()

    def load_primitives(self,dir='primitives'):
        self.primitives = []
        dirList = os.listdir(dir)
        for fname in dirList:
            if fname.startswith('problem'):
                fproblem = open(os.path.join(dir,fname),'r')
                primitivefn = 'primitive'+fname[7:]
                fprimitive = open(os.path.join(dir,primitivefn),'r')
                problem = cPickle.load(fproblem)
                primitive = cPickle.load(fprimitive)
                self.primitives.append((problem,primitive))
                fproblem.close()
                fprimitive.close()
        print "Loaded",len(self.primitives),"primitives from",dir

    def label(self,pairs=None):
        """Computes the adaptation curves for all pairs (s,d) in the
        indices and targets lists, respectively.  Saves them to the
        adaptcosts member."""
        if pairs==None:
            n = len(self.primitives)
            pairs = itertools.product(range(n),range(n))
        elif isinstance(pairs,int):
            #pick random
            n = len(self.primitives)
            pairs = [(random.randint(0,n-1),random.randint(0,n-1)) for k in xrange(pairs)]
        if not hasattr(self,'adaptcosts'):
            self.adaptcosts = dict()
        for (i,t) in pairs:
            src, srcsoln = self.primitives[i]
            print "Adapt",i,t
            dest = self.primitives[t][0]
            destp = self.problemSpace.genProblem(dest)
            
            #call this version for control problems
            print "Optimized cost",destp.evalCost(*self.primitives[t][1])
            #call this version for regular optimization problems
            #print "Optimized cost",destp.evalCost(self.primitives[t][1])
            
            destsoln,costs = self.adapt(src,srcsoln,dest,True)
            #print costs
            print "Adapted cost",costs[0],'->',costs[-1]
            self.adaptcosts[(i,t)] = costs

    def save_costs(self,dir):
        if not hasattr(self,'adaptcosts'):
            return
        f = open(os.path.join(dir,'costs.dat'),'w')
        cPickle.dump(self.adaptcosts,f)
        f.close()

    def load_costs(self,dir):
        f = open(os.path.join(dir,'costs.dat'),'r')
        self.adaptcosts = cPickle.load(f)
        f.close()

    def merge(self,libs):
        """Merges a list of PrimitiveLibrary's"""
        ofs = [0]
        for lib in libs:
            ofs.append(obs[-1]+len(libs.primitives))
        self.primitives = []
        self.adaptcosts = dict()
        for of,lib in zip(ofs[:-1],libs):
            self.primitives += lib.primitives
            if hasattr(lib,'adaptcosts'):
                for key,value in lib.adaptcosts:
                    self.adaptcosts[(key[0]+of,key[1]+of)] = value

    def problem_params(self):
        """Returns a list of all problem parameters"""
        return [p[0] for p in self.primitives]

    def problems(self):
        """Returns a list of all problems"""
        return [genProblem(p[0]) for p in self.primitives]

    def motions(self):
        """Returns a list of all motions (x trajectories)"""
        return [p[1][0] for p in self.primitives]
        
    def controls(self):
        """Returns a list of all controls (u trajectories)"""
        return [p[1][1] for p in self.primitives]

    def best_primitive(self,dest,iteration_cost_weight=0.0):
        """Returns the best primitive to adapt to the primitive indexed by
        dest.  Searches through the adaptcosts structure.  Returns
        the best index and its expected cost"""
        best = None
        best_cost = 1e300
        for (s,d),costs in self.adaptcosts.iteritems():
            if d!=dest: continue
            c = self.opt_cost(s,d,iteration_cost_weight)
            if c[0] < best_cost:
                best_cost = c[0]
                best = s
        return (best,best_cost)

    def opt_cost(self,s,d,iteration_cost_weight=0.0):
        """Returns the optimal value and iteration count (c,k) for the
        adaptation from source s to destination d.  Uses a satisficing
        strategy that minimizes the time-sensitive cost function
        c(k) = cost[k]+k*iteration_cost_weight"""
        cost = self.adaptcosts[(s,d)]
        return min([(c+k*iteration_cost_weight,k) for (k,c) in enumerate(cost) if c != None])        

    def costs_to_array(self,iteration_cost_weight=0.0):
        """Returns a pair of arrays (X,Y) where each entry corresponds to the
        pair (s,d) in the arraycosts member, with X[i]=[f[s],f[d]] and
        Y[i]=opt_cost(s,d)."""
        X = []
        Y = []
        for (s,d) in sorted(self.adaptcosts.keys()):
            X.append([s]+self.problemSpace.features(self.primitives[s][0])+[d]+self.problemSpace.features(self.primitives[d][0]))
            Y.append(self.opt_cost(s,d,iteration_cost_weight))
            #c_from_scratch = self.problemSpace.genProblem(self.primitives[d][0]).evalCost(*self.primitives[d][1])
            c_from_scratch = self.problemSpace.genProblem(self.primitives[d][0]).evalCost(self.primitives[d][1])
            Y[-1] = Y[-1] + (c_from_scratch,)
        return (X,Y)
    
    def costs_to_array_labels(self):
        """Returns labels for the arrays provided by costs_to_array"""
        return (['s']+self.problemSpace.featureNames(self.primitives[0][0],'s')+['d']+self.problemSpace.featureNames(self.primitives[0][0],'d'),['adapt_cost','adapt_iters','scratch_cost'])

if __name__=='__main__':
    usage = """Usage: %prog [options] {gen,reopt,label,dump}
    
Commands:
  - gen: generates N primitives using the problem's global optimizer.
  - reopt: reoptimizes the primitives in the library using the
    problem's local optimizer.
  - label: generates pairwise adaptation costs using the local optimizer
  - dump: outputs adapted costs to costs.csv."""

    parser = OptionParser(usage=usage)
    parser.add_option('-o','--output',dest='fdir',help='read/write to directory DIR',metavar='DIR',default='primitives')
    parser.add_option('-p','--problem',dest='problem',help='set the problem space module to P',metavar='P',default='optimization.control.doubleintegrator')
    parser.add_option('-n','--number',dest='nPrimitives',help='generate N primitives',metavar='N',type="int",default=None)
    parser.add_option('-c','--timecost',dest='timecost',help='set the satisficing weight to C',metavar='C',type="float",default=0.01)
                      
    (options,args) = parser.parse_args()
    fdir = options.fdir
    nPrimitives = options.nPrimitives
    stepCostWeight = options.timecost
    from optimization.control import doubleintegrator
    try:
        mod,fn = options.problem.rsplit('.',1)
        print "from",mod,"import",fn
        __import__(options.problem)
        problemSpace = sys.modules[options.problem]
    except ImportError as e:
        print "Invalid problem specified"
        print e
        exit(1)
    
    if len(args) >= 1:
        for arg in args:
            library = PrimitiveLibrary(problemSpace)
            if arg=='gen':
                library.load_primitives(fdir)
                print "Saving primitives to",fdir
                if nPrimitives == None:
                    nPrimitives = 10
                library.generate(nPrimitives,fdir)
            elif arg=='reopt':
                library.load_primitives(fdir)
                library.reoptimize()
                print "Saving primitives to",fdir
                library.save_primitives(fdir)
            elif arg=='label':
                library.load_primitives(fdir)
                library.label(nPrimitives)
                print "Saving costs to",fdir
                library.save_costs(fdir)
            elif arg=='dump':
                library.load_primitives(fdir)
                library.load_costs(fdir)
                X,Y = library.costs_to_array(stepCostWeight)
                Xlab,Ylab = library.costs_to_array_labels()
                with open(os.path.join(fdir,"costs.csv"),'w') as f:
                    writer = csv.writer(f)
                    writer.writerow(Xlab+Ylab)
                    for Xi,Yi in zip(X,Y):
                        writer.writerow(Xi+list(Yi))
                print "Saved to",os.path.join(fdir,"costs.csv")
            else:
                print "Valid commands are gen, reopt, label, and dump"
    else:
        parser.print_help()
