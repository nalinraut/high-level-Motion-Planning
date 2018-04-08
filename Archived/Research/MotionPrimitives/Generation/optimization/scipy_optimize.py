from scipy.optimize import minimize
from optimize import Optimizer

class ScipyOptimizer(Optimizer):
    def __init__(self,problem,method='BFGS',tol=None,options=dict()):
        Optimizer.__init__(self,problem)
        self.method = method
        self.tol = tol
        self.options = options.copy()

    def run(self,**kwparams):
        params = self.runparams
        params.update(kwparams)
        self.init(**params)
        nIters = 0
        if 'numIters' in params:
            nIters = params['numIters']
        else:
            raise ValueError("run() method needs numIters to be set")

        def myCallback(x):
            self.solution = (x,self.problem.evalCost(x))
            self.inspector(self,'step')
        myCallback.self = self

        jac = None
        try:
            if self.problem.f.deriv(1,self.solution[0]) != None:
                jac = lambda x:self.problem.f.deriv(1,x)
        except NotImplementedError:
            pass
        hess = None
        try:
            if self.problem.f.deriv(2,self.solution[0]) != None:
                hess = lambda x:self.problem.f.deriv(2,x)
        except NotImplementedError:
            pass                
        self.options['numIters']=nIters
        res = minimize(self.problem.f,self.solution[0],method=self.method,
                       jac=jac,hess=None,
                       options = self.options,
                       tol = self.tol,
                       callback = myCallback)
        self.solution = (res.x,self.problem.f.evalCost(res.x))
        return self.solution

