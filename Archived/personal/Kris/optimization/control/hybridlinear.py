import numpy as np
from linear import LinearController

class LinearCondition:
    """Any condition c^T x  >= threshold """
    def __init__(self,c,thresh):
        self.c = c
        self.thresh = thresh
        
    def __eval__(self,x):
        return np.dot(self.c,x) >= self.thresh

class MultiLinearCondition:
    """Any condition A x  >= threshold for all entries """
    def __init__(self,A,thresh):
        self.A = A
        self.thresh = thresh
        
    def __eval__(self,x):
        return all(np.dot(self.A,x) >= self.thresh)


class HybridLinearController:
    """Consists of N discrete states containing a matrix Ki and offset ji
    such that u=Ki*x+ji.  Each state may transition to another depending
    if the transition condition transitions[i][j](x) returns True."""
    def __init__(self):
        self.currentState = None
        self.states = []
        self.transitions = dict()

    def setParameters(self,params):
        """Given a set of parameters in the dict params, sets the linear
        controllers accordingly.  params is set up so a singleton key i
        gives the (Ki,ji) pair of the state, and a pair key (i,j) gives the
        (c_ij,thresh_ij) pair of the condition"""
        self.states = []
        self.transitions = dict()
        for (key,val) in params.iteritems():
            if hasattr(key,'__iter__'):
                self.addLinearTransition(key[0],key[1],val[0],val[1])
            else:
                self.addLinearState(key,val[0],val[1])

    def addState(self,controller):
        self.states.append(controller)
        self.transitions[len(self.states)-1] = dict()

    def addLinearState(self,K,j):
        self.addState(LinearController(K,j))

    def addTransition(self,i,j,condition):
        self.transitions[i][j] = condition

    def addLinearTransition(self,i,j,c,thresh):
        self.addTransition(i,j,LinearCondition(c,thresh))

    def init(self,initState=0):
        """Sets the initial controller's state"""
        self.currentState = initState

    def update(self,x):
        """Computes the control and advances the controller's state"""
        u = self.states[self.currentState](x)
        for (j,cond) in self.transitions[self.currentState].iteritems():
            if cond(x):
                self.currentState = j
                break
        return u
