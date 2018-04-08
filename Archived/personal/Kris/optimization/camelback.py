import random
import optimize
from utils import *
from sympy_function import *
import sympy
import function
import sample

def features(params):
    return flatten(params)

def featureNames(params,prefix=''):
    return flatten_keys(params,prefix)

def genProblem(params):
    p = optimize.OptimizationProblem()
    p.params = params
    x = sympy.Symbol('x')
    y = sympy.Symbol('y')
    c1 = 4
    c2 = -2.1
    c3 = 1.0/3.0
    c4 = -4
    c5 = 4
    if 'c1' in params: c1 = params['c1']
    if 'c2' in params: c2 = params['c2']
    if 'c3' in params: c3 = params['c3']
    if 'c4' in params: c4 = params['c4']
    if 'c5' in params: c5 = params['c5']
    f = ( c1 + c2*x**2 +  c3*x**4)*x**2 + x*y + ( c4  +  c5*y**2)*y**2
    p.f = SympyFunction(f,[x,y])
    #print p.f.diff(1).expr
    bounds = [(-3,3),(-2,2)]
    p.addConstraint(function.BoxContains(bounds))
    assert p.isBoxBounded()
    p.sample = sample.BoxSampler(bounds)
    return p
    
def sampleProblem(coeffbounds=[(3,5),(-2.5,-2.0),(1.0/3.0,1.0/3.0),(-4.5,-3.5),(3.5,4.5)]):
    params = dict()
    for i,c in enumerate(coeffbounds):
        params['c'+str(i+1)] = random.uniform(c[0],c[1])
    return genProblem(params)

def globalOptimizer(problem):
    """Return the global optimizer used by the primitive library"""
    #local optimizer parameters
    localparams = {'numIters':100,'tol':1e-4}
    opt = optimize.globalOptimizer(problem,'random','gradient',localOptParams=localparams,numIters=50)
    opt.beginPrint()
    return opt

def adaptOptimizer(problem,seedProblemParams,seedPrimitive):
    """Return the adaptation optimizer used by the primitive library"""
    opt = optimize.localOptimizer(problem,'gradient',tol=1e-4,x=seedPrimitive,numIters=100)
    opt.beginPrint()
    return opt
    
