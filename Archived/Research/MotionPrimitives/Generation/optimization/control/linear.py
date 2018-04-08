import numpy as np

class LinearController:
    """A linear controller u = A x + b (b is optional)"""
    def __init__(self,A,b=None):
        self.A = A
        self.b = b

    def __eval__(self,x):
        res = np.dot(self.A,x)
        if self.b:
            res += self.b
        return res
        
