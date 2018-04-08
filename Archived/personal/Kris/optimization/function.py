import numpy as np
import math

def isscalar(x):
    return not hasattr(x,'__len__')

class Function:
    """This is the abstract base class for parseable functions.
    
    Subclasses should implement __call__, arity, and returnType.
    Typically you will subclass one of Boolean, Integer, or Numeric
    which automatically override returnType.
    """
    def __call__(self,*args):
        raise NotImplementedError()
    def arity(self):
        raise NotImplementedError()
    def returnType(self):
        raise NotImplementedError()
    def __add__(self,other):
        return Composite(Plus(),self,other)
    def __radd__(self,other):
        return Composite(Plus(),other,self)
    def __sub__(self,other):
        return Composite(Minus(),self,other)
    def __mul__(self,other):
        return Composite(Times(),self,other)
    def __rmul__(self,other):
        return Composite(Times(),other,self)
    def __neg__(self):
        return Composite(Negative(),self)
    def and_(self,other):
        return Composite(And(),self,other)
    def or_(self,other):
        return Composite(Or(),self,other)
    def not_(self):
        return Composite(Not(),self)
    def __eq__(self,other):
        return Composite(EQ(),self,other)
    def __ge__(self,other):
        return Composite(GE(),self,other)
    def __le__(self,other):
        return Composite(LE(),self,other)
    def __getitem__(self,index):
        if isinstance(index,int):
            return Composite(Index(index),self)
        else:
            return Composite(Slice(range(*index.indices(1000000))),self)

class Boolean(Function):
    """This is the abstract base class for boolean hooks"""
    def returnType(self):
        return bool
    def numeric(self):
        """Subclass: return a Numeric representation (g,cmp) of the function
        such that f(x)=I{g(x) cmp 0}.  cmp can be any of 'ge', 'le', 'eq',
        'neq'.
        
        If no such representation exists, return None."""
        return None

class Integer(Function):
    """This is the abstract base class for integer hooks"""
    def returnType(self):
        return int
    def numeric(self):
        """Subclass: return a Numeric representation (g,op) of the function
        such that f(x)=op(g(x)).  op can be any of 'floor', 'ceil', 'round'.
        
        If no such representation exists, return None."""
        return None


class Numeric(Function):
    """This is the abstract base class for numerical hooks that may include
    derivative information.  Derivative at a point is evaluated with deriv().
    Differentation of the function is produced by diff(), which should
    return a subclass of Function.  Zero's are often denoted as None.
    
    See scipy.misc.derivative for numerical differentiation methods.

    1st derivatives for multiple arguments should be a list corresponding to
    the argument order.

    1st derivatives for scalar valued functions with vector valued arguments
    should return a row vector.

    1st derivatives for vector valued functions with vector valued arguments
    should return a numpy array or a scipy sparse matrix with shape (m,n)
    where f(x):R^n -> R^m, i.e. the Jacobian matrix.

    2nd derivatives for multiple arguments should be a list-of-lists
    so that the i,j'th entry corresponds to (d^2 f) / (dxi dxj)

    2nd derivatives for scalar valued functions with vector valued arguments
    should be a numpy array or scipy sparse matrix with shape (n,n), i.e.
    the Hessian matrix.
    """
    def __call__(self,*args):
        raise NotImplementedError()
    def arity(self):
        raise NotImplementedError()
    def deriv(self,order,*args):
        raise NotImplementedError()
    def diff(self,order):
        raise NotImplementedError()
    def returnType(self):
        return np.array
    def numeric(self):
        return self

class Value(Function):
    """Defines a constant value that can be of bool, int, float, or np.ndarray
    types."""
    def __init__(self,val,arity=0):
        self.order = 0
        self.val = val
        self.arity = arity
    def __call__(self,*args):
        return self.val
    def arity(self):
        return self.arity
    def deriv(self,order,*args):
        if self.arity <= 1:
            return None
        else:
            return [None]*self.arity
    def diff(self,order):
        return None
    def returnType(self):
        return type(self.val)
    def numeric(self):
        return self
    def __add__(self,other):
        if isinstance(other,Value): return Value(self.val+other.val,max(self.arity,other.arity))
        return Function.__add__(self,other)
    def __radd__(self,other):
        return Value(self.val+other)
    def __sub__(self,other):
        if isinstance(other,Value): return Value(self.val-other.val,max(self.arity,other.arity))
        return Function.__sub__(self,other)
    def __mul__(self,other):
        if isinstance(other,Value): return Value(self.val*other.val,max(self.arity,other.arity))
        return Function.__mul__(self,other)
    def __rmul__(self,other):
        return Value(self.val*other)
    def __neg__(self):
        return Value(-self.val)
    def and_(self,other):
        if isinstance(other,Value): return Value(self.val and other.val,max(self.arity,other.arity))
        return Function.and_(self,other)
    def or_(self,other):
        if isinstance(other,Value): return Value(self.val or other.val,max(self.arity,other.arity))
        return Function.or_(self,other)
    def not_(self):
        return Value(not self.val,self.arity)
    def __eq__(self,other):
        if isinstance(other,Value): return Value(self.val==other.val,max(self.arity,other.arity))
        return Function.__eq__(self,other)
    def __ge__(self,other):
        if isinstance(other,Value): return Value(self.val>=other.val,max(self.arity,other.arity))
        return Function.__ge__(self,other)
    def __getitem__(self,index):
        return Value(self.val[index],self.arity)

class Composite(Function):
    """Creates a function that passes arguments to g's and then passes results
    to f.  g's can also be constants (anything that doesn't have the __call__
    attribute)

    h(x,y,z) = f(g0(x,y,z),...,gk(x,y,z))
    """
    def __init__(self,f,*gs):
        self.f = f
        self.gs = gs

    def __call__(self,*args):
        gres = []
        for g in self.gs:
            if hasattr(g,'__call__'):
                gres.append(g(*args))
            else:
                gres.append(g)
        return self.f(*gres)

    def arity(self):
        return max((g.arity() if hasattr(g,'arity') else 0) for g in self.gs)

    def deriv(self,order,*args):
        if order > 1: raise NotImplementedError()
        gres = []
        for g in self.gs:
            if not hasattr(g,'__call__'):
                gres.append(g)
            else:
                gres.append(g(*args))
        partials = self.f.deriv(order,*gres)
        if partials == None:
            return None
        derivs = None
        if len(args) > 1:
            derivs = [None]*len(args)
        if len(self.gs)==1:
            partials = [partials]
        assert len(partials)==len(self.gs)
        for dfdg,g in zip(partials,self.gs):
            if dfdg==None:
                continue
            elif hasattr(g,'__call__'):
                dg = g.deriv(1,*args)
                if len(args) > 1:
                    for i in xrange(len(args)):
                        if dg[i] != None:
                            if derivs[i] != None:
                                derivs[i] += np.dot(dfdg,dg[i])
                            else:
                                derivs[i] = np.dot(dfdg,dg[i])
                else:
                    if dg == None: continue
                    if derivs != None:
                        derivs += np.dot(dfdg,dg)
                    else:
                        derivs = np.dot(dfdg,dg)
        return derivs
    
    def returnType(self):
        return self.f.returnType()
    
    def __add__(self,other):
        if isinstance(self.f,(Plus,Sum)):
            return Composite(Sum(),*(self.gs+[other]))
        return Function.__add__(self,other)
    def __radd__(self,other):
        if isinstance(self.f,(Plus,Sum)):
            return Composite(Sum(),other,*self.gs)
        return Function.__radd__(self,other)
    def __neg__(self):
        if isinstance(self.f,Negative):
            assert len(self.gs)==1
            return self.gs[0]        
        return Function.__neg__(self)
    def and_(self,other):
        if isinstance(self.f,And):
            return Composite(And(),*(self.gs+[other]))
        return Function.and_(self,other)
    def or_(self,other):
        if isinstance(self.f,Or):
            return Composite(Or(),*(self.gs+[other]))
        return Function.or_(self,other)
    def not_(self):
        if isinstance(self.f,Not):
            assert len(self.gs)==1
            return self.gs[0]
        return Function.not_(self)
    def numeric(self):
        raise NotImplementedError("TODO: composite numerical conditions")
        gn = [g.numeric() for g in self.gs]
        if any(g == None for g in gn): return None
        if isinstance(self.f,Not):
            return Composite(Negative(),*gn)
        if isinstance(self.f,And):
            return Composite(Max(),*gn)
        fn = self.f.numeric()
        if fn == None: return None
        return Composite(fn,*self.gs)

class Not(Boolean):
    """Condition not x """
    def arity(self):
        return 1
    def __call__(self,x):
        return not x

class And(Boolean):
    """Condition x and y """
    def arity(self):
        return 2
    def __call__(self,x,y):
        return x and y

class Or(Boolean):
    """Condition x or y """
    def arity(self):
        return 2
    def __call__(self,x,y):
        return x or y

class All(Boolean):
    """Condition all(xi) """
    def __init__(self,numargs=-1):
        self.numargs = numargs
    def arity(self):
        return self.numargs
    def __call__(self,*args):
        return all(args)

class Any(Boolean):
    """Condition any(xi) """
    def __init__(self,numargs=-1):
        self.numargs = numargs
    def arity(self):
        return self.numargs
    def __call__(self,*args):
        return any(args)

class EQ(Boolean):
    """Equality condition x == y """
    def arity():
        return 2
    def __call__(self,x,y):
        if isscalar(x):
            return x==y
        else:
            return all(x == y)
    def numeric(self):
        return (Minus(),'eq')

class GE(Boolean):
    """Inequality condition x >= y """
    def arity():
        return 2
    def __call__(self,x,y):
        if isscalar(x):
            return x>=y
        else:
            return all(x >= y)
    def numeric(self):
        return (Minus(),'ge')


class LE(Boolean):
    """Inequality condition x <= y """
    def arity():
        return 2
    def __call__(self,x,y):
        if isscalar(x):
            return x<=y
        else:
            return all(x <= y)
    def numeric(self):
        return (Minus(),'le')

class EQZ(Boolean):
    """Equality condition x == 0 """
    def arity():
        return 1
    def __call__(self,x):
        if isscalar(x):
            return x==0
        else:
            return all(x == 0)
    def numeric(self):
        return (Identity(),'eq')

class GEQZ(Boolean):
    """Inequality condition x >= 0  """
    def arity():
        return 1
    def __call__(self,*args):
        return all(a >= 0 for a in args)
    def numeric(self):
        return (Identity(),'geq')

class LEQZ(Boolean):
    """Inequality condition x <= 0
    """
    def arity():
        return 1
    def __call__(self,*args):
        return all(a <= 0 for a in args)
    def numeric(self):
        return (Negative(),'geq')

class IndexDiff(Numeric):
    """Derivative of the index function"""
    def __init__(self,index):
        self.order = 0
        self.index = index
    def arity(self):
        return 1
    def __call__(self,x):
        res = np.zeros((1,len(x)))
        res[(0,self.index)] = 1
        return res
    def deriv(self,order,x):
        return None
    def diff(self,order):
        return None

class Index(Numeric):
    """Index function f(x) = x[index]"""
    def __init__(self,index):
        self.order = 1
        self.index = index

    def arity(self):
        return 1
        
    def __call__(self,x):
        return x[self.index]

    def deriv(self,order,x):
        if order > 1: return None
        res = np.zeros((1,len(x)))
        res[(0,self.index)] = 1.0
        return res

    def diff(self,order):
        if order > 1: return None
        return IndexDiff(self.index)

class SliceDiff(Numeric):
    """Derivative of the slice function"""
    def __init__(self,range):
        self.order = 0
        self.range = range
    def arity(self):
        return 1
    def __call__(self,x):
        res = np.zeros((len(self.range),) + x.shape)
        for i,r in enumerate(self.range):
            res[(i,r)] = 1.0
        return res
    def deriv(self,order,x):
        return None
    def diff(self,order):
        return None

class Slice(Numeric):
    """Slice function f(x) = [x[i] for i in range]"""
    def __init__(self,range):
        self.order = 1
        self.range = range

    def arity(self):
        return 1
        
    def __call__(self,x):
        return np.array([x[i] for i in self.range])

    def diff(self,order):
        if order > 1: return None
        return SliceDiff(self.range)

    def deriv(self,order,x):
        if order > 1: return None
        res = np.zeros((len(self.range),) + x.shape)
        for i,r in enumerate(self.range):
            res[(i,r)] = 1.0
        return res


class Stack(Numeric):
    """Stack function f(x1,...,xn) = [x1^T, ..., xn^T]^T"""
    def __init__(self,arity=-1):
        self.order = 1
        self.arity = arity

    def arity(self):
        return self.arity
        
    def __call__(self,*args):
        return np.stack(args)


class Identity(Numeric):
    """Identity function f(x) = x"""
    def arity():
        return 1
    def __call__(self,x):
        return x
    def diff(self,order):
        if order > 1: return None
        return Value(1,arity=1)
    def deriv(self,order,x):
        if order > 1: return None
        if isscalar(x):
            return 1
        else:
            return np.eye(len(x))

class Negative(Numeric):
    """Negative function f(x) = -x"""
    def __init__(self):
        self.order = 1
    def __call__(self,x):
        return -x
    def arity(self):
        return 1
    def diff(self,order):
        if order > 1: return None
        return Value(-1,arity=1)
    def deriv(self,order,*args):
        if order > 1: return None
        x=args[0]
        if isscalar(x):
            return -1
        else:
            return -np.eye(len(x))

class AbsDiff(Numeric):
    def __init__(self):
        self.order = 0
    def arity(self):
        return 1
    def __call__(self,x):
        if isscalar(x):
            return -1 if x < 0 else (1 if x > 0 else 0)
        else:
            m = np.eye(len(x))
            for i in len(x):
                if x[i] < 0:
                    m[i,i]=-1
                elif x[i] == 0:
                    m[i,i]=0
            return m

class Abs(Numeric):
    """Absolute value f(x) = |x|"""
    def __init__(self):
        self.order = 1
    def __call__(self,x):
        return abs(x)
    def arity(self):
        return 1
    def diff(self,order):
        if order > 1: return None
        return AbsDiff()
    def deriv(self,order,x):
        if order > 1: return None
        return self.diff(order)(x)

class Sum(Numeric):
    def __init__(self,numargs = -1):
        self.order = 1
        self.numargs = numargs
        
    def __call__(self,*args):
        return sum(args)

    def arity(self):
        return numargs

    def deriv(self,order,*args):
        if order > 1: return None
        if isscalar(args[0]):
            return [1]*len(args)
        else:
            return [np.eye(len(a)) for a in args]

class Plus(Numeric):
    def __init__(self):
        self.order = 1
    def __call__(self,x,y):
        return x+y
    def arity(self):
        return 2
    def deriv(self,order,*args):
        if order > 1: return None
        if isscalar(x):
            return [1,1]
        else:
            return [np.eye(len(x)),np.eye(len(y))]

class Minus(Numeric):
    def __init__(self):
        self.order = 1
    def __call__(self,x,y):
        return x-y
    def arity(self):
        return 2
    def deriv(self,order,*args):
        if order > 1: return None
        x,y=args
        if isscalar(x):
            return [1,-1]
        else:
            return [np.eye(len(x)),-np.eye(len(y))]

class Times(Numeric):
    def __init__(self):
        self.order = 2
    def __call__(self,x,y):
        return x*y
    def arity(self):
        return 2
    def deriv(self,order,*args):
        if order > 2: return None
        x,y=args
        if order == 1:
            if isscalar(x):
                return [y,x]
            else:
                return [np.diag(y),np.diag(x)]
        else:
            if isscalar(x) and isscalar(y):
                return [[None,1.0],[1.0,None]]
            elif isscalar(x):
                return [[None,np.zeros(y.shape)+1.0],[np.zeros(y.shape).T+1.0,None]]
            elif isscalar(y):
                return [[None,np.zeros(x.shape).T+1.0],[np.zeros(x.shape)+1.0,None]]
            else:
                return [[None,np.eye(len(x))],[np.eye(len(x)),None]]

class Pow(Numeric):
    """Returns f(x) = x^power """
    def __init__(self,power):
        self.power = power
    def arity(self):
        return 1
    def __call__(self,x):
        return pow(x,self.power)
    def deriv(self,order,x):
        return self.diff(order)(x)
    def diff(self,order):
        if order==1:
            return Pow(self.power-1)*self.power
        else:
            return self.diff(1).diff(order-1)

class Exp(Numeric):
    """Returns f(x) = base^x """
    def __init__(self,base=math.e):
        self.base = base
        self.logbase = math.log(base)
    def arity(self):
        return 1
    def __call__(self,x):
        return pow(self.base,x)
    def deriv(self,order,x):
        return self.diff(order)(x)
    def diff(self,order):
        if order==1:
            return self.logbase*Exp(self.base)
        else:
            return pow(self.logbase,order)*Exp(self.base)

class Log(Numeric):
    """Returns f(x) = log_base(x) """
    def __init__(self,base=math.e):
        self.base = base
        self.logbase = math.log(base)
    def arity(self):
        return 1
    def __call__(self,x):
        return math.log(x)/self.logbase
    def deriv(self,order,x):
        if order == 1:
            return 1.0/(self.logbase*x)
        else:
            return self.diff(order)(x)
    def diff(self,order):
        if order==1:
            return (1.0/self.logbase)*Pow(-1)
        else:
            return self.diff(1).diff(order-1)

class PowXY(Numeric):
    """Returns f(x,y) = x^y"""
    def arity(self):
        return 2
    def __call__(self,x,y):
        return pow(x,y)
    def deriv(self,order,x,y):
        if order == 1:
            return [y*pow(x,y-1),math.log(x)*pow(x,y)]
        else:
            raise NotImplementedError()
    def diff(self,order):
        if order == 1:
            return [Index(1)*Composite(PowXY(),Index(0),Index(1)-1),Composite(Log(),Index(0))*PowXY]
        else:
            return [dx.diff(order-1) for dx in self.diff(1)]

class Dot(Numeric):
    def arity(self):
        return 2
    def __call__(self,x,y):
        return np.dot(x,y)
    def deriv(self,order,x,y):
        if order > 2: return None
        if order == 1:
            return [y,x]
        else:
            if isscalar(x) and isscalar(y):
                return [[None,1.0],[1.0,None]]
            else:
                return [[None,np.eye(len(x))],[np.eye(len(x)),None]]

class DotSelf(Numeric):
    def arity(self):
        return 1
    def __call__(self,x):
        return np.dot(x,x)
    def deriv(self,order,x):
        if order > 2: return None
        if order == 1:
            return 2.0*x
        else:
            if isscalar(x):
                return 2.0
            else:
                return 2.0*np.eye(len(x))

class Linear(Numeric):
    def __init__(self,A,b):
        self.A,self.b = A,b
        self.order = 1
        
    def arity(self):
        return 1
    
    def __call__(self,x):
        if isscalar(x):
            return self.A*x+self.b
        return np.dot(self.A,x)+self.b

    def deriv(self,order,x):
        if order > 1: return None
        return self.A

    def diff(self,order):
        if order > 1: return None
        return Constant(self.A)

class Quadratic(Numeric):
    def __init__(self,P,q,r):
        self.P,self.q,self.r = P,q,r
        self.order = 2

    def arity(self):
        return 1

    def __call__(self,x):
        if isscalar(x):
            return self.P*x**2+self.q*x+self.r
        return np.dot(x,np.dot(self.P,x)+self.q)+self.r

    def deriv(self,order,x):
        if order > 2: return None
        if order == 2: return 2.0*self.P
        if isscalar(x):
            return 2*self.P*x+self.q
        return 2.0*np.dot(self.P,x)+self.q

    def diff(self,order):
        if order > 2: return None
        if order == 2: return Constant(2.0*self.P)
        return Linear(2*self.P,self.q)

class LinearBivariate(Numeric):
    def __init__(self,A,B,c):
        self.A,self.B,self.c = A,B,c

    def arity(self):
        return 2

    def __call__(self,x,y):
        return np.dot(self.A,x)+np.dot(self.B,y)+self.c

    def deriv(self,order,x,y):
        if order > 1:
            return (None,None)
        return (self.A,self.B)

    def diff(self,order):
        if order > 1: return None
        return Constant((self.A,self.B))

class QuadraticBivariate(Numeric):
    def __init__(self,Pxx,Pxy,Pyy,qx,qy,r):
        self.Pxx,self.Pxy,self.Pyy = Pxx,Pxy,Pyy
        self.qx,self.qy = qx,qy
        self.r = r
        self.order = 2
        
    def arity(self):
        return 2
        
    def __call__(self,x,y):
        return np.dot(x,np.dot(self.Pxx,x)+self.qx)+np.dot(y,np.dot(self.Pyy,y)+self.qy)+2.0*np.dot(x,np.dot(self.Pxy,y))+self.r

    def deriv(self,order,x,y):
        if order > 2:
            return (None,None)
        if order == 2:
            return [[self.Pxx,self.Pxy],[self.Pxy.T,self.Pyy]]
        else:
            return (2.0*np.dot(self.Pxx,x)+np.dot(self.Pxy,y)+self.qx,
                    2.0*np.dot(self.Pyy,y)+np.dot(self.Pxy.T,x)+self.qy)

class BoxContains(Boolean):
    def __init__(self,bounds):
        self.bounds = bounds
    def arity(self):
        return 1
    def __call__(self,x):
        return all(a <= xi and xi <= b for xi,(a,b) in zip(x,self.bounds))

def and_(*args):
    if len(args) == 0: return And()
    return Composite(And(len(args)),*args)

def or_(*args):
    if len(args) == 0: return Or()
    return Composite(Or(len(args)),*args)

def not_(*args):
    if len(args) == 0: return Not()
    return Composite(Not(),*args)

def any_(*args):
    if len(args) == 0: return Any()
    return Composite(Any(len(args)),*args)

def all_(*args):
    if len(args) == 0: return All()
    return Composite(All(len(args)),*args)

def plus(*args):
    if len(args) == 0: return Plus()
    return Composite(Plus(),*args)

def minus(*args):
    if len(args) == 0: return Minus()
    return Composite(Minus(),*args)

def neg(*args):
    if len(args) == 0: return Negative()
    return Composite(Negative(),*args)

def times(*args):
    if len(args) == 0: return Times()
    return Composite(Times(),*args)

def dot(*args):
    if len(args) == 0: return Dot()
    return Composite(Dot(),*args)

def sum_(*args):
    if len(args) == 0: return Sum()
    return Composite(Sum(len(args)),*args)

def abs_(*args):
    if len(args) == 0: return Abs()
    return Composite(Abs(),*args)

def exp(arg=None,base=math.e):
    if arg == None: return Exp()
    return Composite(Exp(base),arg)

def log(arg=None,base=math.e):
    if arg == None: return Log()
    return Composite(Log(base),arg)

def pow_(*args):
    if len(args) == 0: return PowXY()
    else:
        assert len(args) == 2
        if not isinstance(args[0],Function):
            return Composite(Exp(args[0]),args[1])
        if not isinstance(args[1],Function):
            return Composite(Pow(args[1]),args[0])
        return Composite(PowXY,*args)

def dot_self(*args):
    if len(args) == 0: return DotSelf()
    else: return Composite(DotSelf(),*args)

def norm(*args):
    if len(args) == 0: return sqrt(DotSelf())
    else: return sqrt(dot_self(*args))

def distance(*args):
    if len(args) == 0: return sqrt(dot_self(minus()))
    else: return sqrt(dot_self(minus(*args)))

def sqrt(*args):
    if len(args) == 0: return Pow(0.5)
    else: return Composite(Pow(0.5),*args)

def eq(*args):
    if len(args) == 0: return EQ()
    return Composite(EQ(),*args)

def eqz(*args):
    if len(args) == 0: return EQZ()
    return Composite(EQZ(),*args)

def geqz(*args):
    if len(args) == 0: return GEQZ()
    return Composite(GEQZ(),*args)

def leqz(*args):
    if len(args) == 0: return LEQZ()
    return Composite(LEQZ(),*args)

zero_ = Value(0.0)

one_ = Value(1.0)

pi_ = Value(math.pi)

e_ = Value(math.e)

if __name__ == '__main__':
    print "Begin self test..."
    x = Identity()
    y = Negative()
    print x.deriv(1,3.0),'==',1
    print x.diff(1)(3.0),'== 1'
    x2 = x*x
    print x2(2.0),'==',4
    print x2(3.0),'==',9
    print x2.deriv(1,2.0),'==',4
    xy = x*y
    print xy.deriv(1,2.0),'==',-4
    #test indexing
    i0 = Index(0)
    i1 = Index(1)
    v = np.array([3.0,4.0])
    print i0(v),'==',3
    print i1(v),'==',4
    print i0.deriv(1,v),'== [1,0]'
    print i1.deriv(1,v),'== [0,1]'
    #test slicing
    s01 = Slice(range(2))
    print s01(v),'== [3,4]'
    print s01.deriv(1,v),'== ident'
    #test Pythonic indexing
    i0 = Identity()[0]
    i1 = Identity()[1]
    print i0(v),'== 3'
    print i1(v),'==',4
    print i0.deriv(1,v),'== [1,0]'
    print i1.deriv(1,v),'== [0,1]'
    #test Pythonic slicing
    s01 = Identity()[0:2]
    print s01(v),'== [3,4]'
    print s01.deriv(1,v),'== ident'
    print "Done."
