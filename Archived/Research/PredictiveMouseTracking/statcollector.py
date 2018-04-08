class StatCollector:
    def __init__(self):
        self.count = 0.0
        self.mean = 0.0
        self.var = 0.0
        self.vmin = 0.0
        self.vmax = 0.0

    def collect(self,value,weight=1.0):
        assert weight > 0.0,"Weight is negative"
        if self.count == 0.0:
            self.vmin = self.vmax = value
        self.vmin = min(self.vmin,value)
        self.vmax = max(self.vmax,value)
        vsum = self.count*self.mean
        vsum2 = (self.var + self.mean*self.mean)*self.count
        newcount = self.count + weight
        newvsum = vsum + value*weight
        newvsum2 = vsum2 + value*value*weight*weight
        self.mean = newvsum/newcount
        self.var = newvsum2/newcount - self.mean*self.mean
        self.count = newcount
