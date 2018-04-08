def parameterShift(f,dt):
    """Returns the function g(t)=f(t+dt)"""
    if hasattr(f,"_func") and hasattr(f,"_dt"):
        g = f.copy()
        g.dt += dt
        return f
    def shifted(t):
        return _func(t+_dt)
    g = shifted
    g._func = func;
    g._dt = dt
    return g

def constantFunction(x):
    def constantFunc(t):
        return x
    f = constantFunc
    f.x = x
    return f

def vectorize(*args):
    """Given a list of component functions f1(t),...,fn(t), return a function
    f(t) = [f1(t),...,fn(t)]"""
    def vectorFunc(t):
        return [f(t) for f in funcs]
    f = vectorFunc
    f.funcs = args
    return f

def compose(f,g):
    """Returns a function h(t) = f(g(t))"""
    def composeFunc(t):
        return f(g(t))
    h = composeFunc
    h.f = f
    h.g = g
    return h

class PiecewiseTrajectory:
    """This class produces a trajectory y(t) consisting of a set of trajectory
    segments segments[], split among times times[].  segment[i] is defined over
    the interval [times[i],times[i+1]).
    The value of y(t)=segment[i](t) iff segment[i] is defined over a range
    that contains t."""
    def __init__(self,segments,times,relative=False):
        """Initializes the PiecewiseTrajectory with the given segments and
        times.  If relative = True, then each of the times is interpreted to
        be the duration of the segment's interval, and segment[i] is
        interpreted to have the domain [0,times[i]].  These will then be
        time-shifted to be defined on the correct domain"""
        if relative:
            if len(segments) != len(times):
                raise ValueError("Invalid sizes")
            if any(t<0 for t in times):
                raise ValueError("Durations are not positive")
            self.segments = []
            self.times = [0]
            for i in range(len(segments)):
                self.segments += [parameterShift(segments[i],-self.times[i])]
                self.times += [self.times[i]+times[i]]
        else:
            if len(segments)+1 != len(times):
                raise ValueError("Invalid sizes")
            if any(times[i] > times[i+1] for i in range(len(times)-1)):
                raise ValueError("Times are not sorted")
            self.segments = segments
            self.times = times

    @staticmethod
    def constant(x,a=0,b=0):
        return PiecewiseTrajectory([constantFunction(x)],[a,b])

    def findSegment(self,t):
        if t < self.times[0]:
            return -1
        # TODO: do a binary search
        for i in range(len(self.segments)):
            if t < self.times[i+1]:
                return i
        return len(self.segments)

    def __call__(self,t):
        if len(self.segments)==0:
            raise ValueError("Invalid sizes")
        i = self.findSegment(t)
        if i < 0:
            return self.start()
        elif i >= len(self.segments):
            return self.end()
        else:
            return self.segments[i](t)

    def deriv(self):
        return PiecewiseTrajectory([s.deriv() for s in self.segments],self.times)
    
    def startTime(self):
        return self.times[0]

    def start(self):
        return self.segments[0](0)
    
    def end(self):
        return self.segments[-1](self.times[-1])
    
    def endTime(self):
        return self.times[-1]

    def domain(self):
        return (self.startTime(),self.endTime())

    def append(self,segment,T,relative=False):
        """Appends the segment to the trajectory following the final
        segment in the trajectory.

        If relative is true, then T is interpreted as the total length
        of the segment, and segment(t) is defined on [0,T].

        If relative is false, then T is interpreted as the final time
        of the segment, and segment(t) is defined on [self.endTime(),T]"""
        if relative:
            if T < 0:
                raise ValueError("Segment cannot have negative duration")
            et = self.endTime()
            self.segments += [parameterShift(segment,-et)]
            self.times += [et+T]
        else:
            if T < endTime():
                raise ValueError("Segment end time must be >= current end time")
            self.segments += [segment]
            self.times += [T]

    def concat(self,traj,relative=False):
        """Appends traj to the current trajectory.  If relative=true,
        the domain of traj is shifted forward in time by self.endTime()"""
        if relative:
            et = self.endTime()
            if traj.times[0] > 0:
                #add a constant segment of the trajectory
                self.segments += [constantFunction(self.end())]
                self.times += et+traj.times[0]
            self.segments += [parameterShift(s,-et) for s in traj.segments]
            self.times += [t+et for t in traj.times[1:]]
        else:
            if traj.times[0] < self.endTime():
                raise ValueError("Concatenated trajectory starts before the end of the current trajectory")
            if traj.times[0] > self.endTime():
                #add a constant segment of the trajectory
                self.segments += [constantFunction(self.end())]
                self.times += [traj.times[0]]
            self.segments += traj.segments
            self.times += traj.times[1:]

    def timeShift(self,dt):
        for i in range(len(times)):
            self.times[i] = self.times[i]+dt
        for i in range(len(segments)):
            self.segments[i] = parameterShift(self.segments[i],-dt)

    def split(self,t):
        i = self.findSegment(t)
        if i < 0:
            t2 = constant(self.start(),t,self.startTime())
            t2.concat(self)
            return (constant(self.start(),t,t),t2)
        elif i >= len(self.segments):
            t1 = self.copy()
            t1.concat(constant(self.end(),self.endTime(),t))
            return (t1,constant(self.end(),t,t))
        else:
            t1 = self.times[:i+1]
            t1[-1] = t
            t2 = self.times[i:]
            t2[0] = t
            return (PiecewiseTrajectory(self.segments[:i],t1),PiecewiseTrajectory(self.segments[i:],t2))

    def trimFront(self,tstart):
        """Sets a new start time tStart"""
        i = self.findSegment(tstart)
        if i < 0:
            #add a new constant function
            s = self.copy()
            self.segments = [constantFunction(s.start())]
            self.times = [tstart,s.startTime()]
            self.concat(s)
        elif i >= len(self.segments):
            q=self.end()
            self.segments = [constantFunction(q)]
            self.times = [tstart,tstart]
        else:
            t2 = self.times[i:]
            t2[0] = tstart
            self.segments = self.segments[i:]
            self.times = t2

    def trimBack(self,tend):
        """Sets a new end time tend"""
        i = self.findSegment(tend)
        if i < 0:
            q=self.start()
            self.segments = [constantFunction(q)]
            self.times = [tend,tend] 
        elif i >= len(self.segments):
            self.segments += [constantFunction(self.end())]
            self.times += [tend]
        else:
            t1 = self.times[:i+1]
            t1[-1] = tend
            self.segments = self.segments[:i]
            self.times = t1

    def select(self,a,b):
        temp = self.copy()
        temp.trimFront(a)
        temp.trimBack(b)
        return temp

    @staticmethod
    def stack(*args):
        return PiecewiseTrajectory([vectorize(*args)],[min(f.startTime() for f in funcs),min(f.endTime() for f in args)])

