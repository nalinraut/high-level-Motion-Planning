import sympy
import function
import numpy as np

def Sym2NumArray(F):
    """Function to convert symbolic expression with numerical data to
    numpy array"""
    if type(F)==sympy.Float:
        return float(F)
    shapeF=F.shape
    B=np.zeros(shapeF)
    for i in range(0,shapeF[0]):
        for j in range(0,shapeF[1]):
            B[i,j]=sympy.N(F[i,j])
    return B

class SympyFunction(function.Numeric):
    """Evaluates a sympy expression with arguments mapped to the given list
    of Symbols.  Each element of symbols can itself be a list, in which case
    the input argument is assumed to be a numpy array."""
    def __init__(self,expr,*symbols):
        self.expr = expr
        self.symbols = symbols
    def arity(self):
        return len(self.symbols)
    def __call__(self,*args):
        expr = self.expr
        subs = dict()
        for s,a in zip(self.symbols,args):
            if hasattr(s,'__iter__'):
                #its an array
                for si,ai in zip(s,a):
                    subs[si] = ai
            else:
                subs[s] = a
        return Sym2NumArray(expr.evalf(subs=subs))
    def deriv(self,order,*args):
        #memoize the diff function
        if hasattr(self,'df'):
            if order not in self.df:
                self.df[order] = self.diff(order)
        else:
            self.df = dict()
            self.df[order] = self.diff(order)
        return self.df[order](*args)
    def diff(self,order):
        expr = self.expr
        make_matrix = (order > 1 or len(self.symbols) > 0)
        while order > 0:
            dedx = []
            for s in self.symbols:
                deds = []
                if hasattr(s,'__iter__'):
                    for si in s:
                        #need to check whether the result is a matrix, and if so, group into columns
                        dedsi = expr.diff(si)
                        if isinstance(dedsi,sympy.Matrix):
                            deds.append(dedsi)
                        else:
                            deds.append([dedsi])
                    #join the elements into rows
                    dedsm = sympy.Matrix(deds[0])
                    for d in deds[1:]:
                        dedsm = dedsm.row_join(sympy.Matrix(d))
                    deds = dedsm
                else:
                    deds = expr.diff(s)
                dedx.append(deds)
            expr = dedx[0] if len(self.symbols)==1 else (sympy.Matrix([dedx]) if make_matrix else dedx)
            order -= 1
        return SympyFunction(expr,*self.symbols)

class SympyCondition(function.Boolean):
    def __init__(self,expr,*symbols):
        self.expr = expr
        self.symbols = symbols
    def arity(self):
        return len(self.symbols)
    def __call__(self,*args):
        expr = self.expr
        for s,a in zip(self.symbols,args):
            if hasattr(s,'__iter__'):
                #its an array
                for si,ai in zip(s,a):
                    expr = expr.subs(si,ai)
            else:
                expr = expr.subs(s,a)
        return expr.evalf()

if __name__== '__main__':
    x,y,u1,u2 = sympy.symbols('x y u1 u2')
    expr = x**2+y**3
    f = SympyFunction(expr,x,y)
    print f(2.0,2.0),'==',12
    print "derivative 1 = [2x,3y^2]:",f.diff(1).expr
    print "derivative 2 = [[2,0],[0,6y]]:",f.diff(2).expr
    print "derivative 3 = const",f.diff(3).expr
    expr2 = sympy.Matrix([x**2+y**3+u1,y**2+0.5*u2])
    g = SympyFunction(expr2,[x,y],[u1,u2])
    print "derivative 1 = [[2x+1,3y^2],[0,2y+0.5]]:"
    print g.diff(1).expr[0]
    print g.diff(1).expr[1]
