from trajclient import *

class MultiTrajClient:
    """A class that connects to multiple trajectory clients at once and that
    can interleave RPC calls for better motion synchronization.

    The standard convention is that all methods in TrajClient are duplicated
    here, except with an additional argument 'h' that defines the host index.
    
    For several methods, h=-1 returns a specially synch'ed version of the call.
    For example, MultiTrajClient.getConfig(-1) will return configurations that
    are more likely to have occurred at the same time than calling getConfig(h)
    for h=0...N (where N is the maximum index of the trajClients).
    """
    def __init__(self,hosts=None,port=1000):
        self.s = None
        self.flog = None
        if hosts != None:
            self.connect(hosts,port)

    def logBegin(self,fn='trajclient.log'):
        self.flog = open(fn,"a")
        for s in self.s:
            s.flog = self.flog

    def logEnd(self):
        if self.flog != None:
            self.flog.close()
            self.flog = None
            for s in self.s:
                s.flog = None
    
    def connect(self,hosts,port=1000):
        self.s = [None]*len(hosts)
        for i in xrange(len(hosts)):
            self.s[i] = TrajClient(hosts[i],port)

    def sendMessage(self,h,cmds,wantReply=True):
        """Sends cmds to only server h.  See TrajClient.sendMessage"""
        return self.s[h].sendMessage(cmds,wantReply)
        
    def sendMessageAll(self,cmds,wantReply=True):
        """ Sends the commands in cmds[i] to controller i.  This is essentially
        the same as:

        for s in range(len(cmds)):
             self.sendMessage(s,cmds[s])

        except all messages are sent at once, and then all return values are
        received at once.
        """
        assert len(cmds) == len(self.s),"Command list must be same size as servers"
        for i in xrange(len(cmds)):
            if cmds[i]==None:
                continue
            # Send the message
            self.s[i].sendMessageRaw(cmds[i])
        
        # Receive the replies
        if wantReply:
            msgs = [None]*len(cmds)
            for i in xrange(len(cmds)):
                if cmds[i]==None:
                    continue
                msgs[i] = self.s[i].receiveReply()
            return msgs
        else:
            return None

    def call(self,h,calls,wantReply=True):
        return self.s[h].call(calls,wantReply)

    def callAll(self,calls,wantReply=True):
        assert len(calls) == len(self.s),"Call list must be same size as clientss"
        def callToMsg(call):
            if type(call) is str:
                return call+"()"
            return call[0]+"("+" ".join(str(a) for a in call[1:])+")"

        #assemble command list
        cmds = [None]*len(funcs)
        for i in xrange(len(funcs)):
            if calls[i] == None:
                continue
            if type(calls[i]) is tuple or type(calls[i]) is str:
                cmds[i] = callToMsg(calls[i])
            else:
                cmds[i] = [callToMsg(c) for c in calls[i]]
        
        return self.sendMessageAll(cmds,wantReply)

    def broadcastCall(self,calls,args):
        """Sends the command(s) func(args) to all servers, returns the
        list of return values."""
        return self.callAll([calls]*len(self.s))

    def __methmap(self,h,method,*methodargs):
        """Default method mapping from MultiTrajClient to TrajClient.
        If h >= 0, this calls the given method in the h'th TrajServer object.
        If h < 0, this calls the given method in all TrajServer objects,
        returning the list of return values."""
        if h >= 0:
            return getattr(self.s[h],method)(self.s[h],*methodargs)
        else:
            return [getattr(sh,method)(sh,*methodargs) for sh in self.s]

    def echo(self,h,message):
        return self.__methmap(h,'echo',message)

    def version(self,h):
        return self.__methmap(h,'version')

    def rate(self,h):
        return self.__methmap(h,'rate')

    def getConfig(self,h=-1):
        """Supports synchronized broadcast calling with h<0."""
        if h >= 0:
            return self.s[h].getConfig()
        else:
            #assemble commands and return values
            res = self.broadcastCall("gj")
            return [[float(x) for x in resi.split()] for resi in res]
    
    def getVelocity(self,h=-1):
        """Supports synchronized broadcast calling with h<0."""
        if h >= 0:
            return self.s[h].getVelocity()
        else:
            #assemble commands and return values
            res = self.broadcastCall("gv")
            return [[float(x) for x in resi.split()] for resi in res]

    def getTransform(self,h=-1):
        """Supports synchronized broadcast calling with h<0."""
        if h >= 0:
            return self.s[h].getTransform()
        else:
            res = self.broadcastCall("gx")
            return [[float(x) for x in resi.split()] for resi in res]

    def getJointLimits(self,h):
        return self.__methmap(h,'getJointLimits')

    def getVelocityLimits(self,h):
        return self.__methmap(h,'getVelocityLimits')

    def getAccelerationLimits(self,h):
        return self.__methmap(h,'getAccelerationLimits')

    def getDecelerationLimits(self,h):
        return self.__methmap(h,'getAccelerationLimits')

    def setJointLimits(self,h,qmin,qmax):
        if h < 0:
            raise ValueError("Cannot set joint limits using h<0")
        return self.s[h].setJointLimits(qmin,qmax)

    def setVelocityLimits(self,h,vmax):
        if h < 0:
            raise ValueError("Cannot set velocity limits using h<0")
        return self.s[h].setVelocityLimits(vmax)

    def setAccelerationLimits(self,h,amax):
        if h < 0:
            raise ValueError("Cannot set acceleration limits using h<0")
        return self.s[h].setAccelerationLimits(amax)

    def setDecelerationLimits(self,h,amax):
        if h < 0:
            raise ValueError("Cannot set deceleration limits using h<0")
        return self.s[h].setDecelerationLimits(amax)

    def getMaxSegments(self,h):
        return self.__methmap(h,'getMaxSegments')

    def getCurSegments(self,h):
        return self.__methmap(h,'getCurSegments')
    
    def getRemainingSegments(self,h):
        return self.__methmap(h,'getRemainingSegments')

    def getCurrentTime(self,h=-1):
        """Supports synchronized broadcast calling with h<0."""
        if h >= 0:
            return self.s[h].getCurrentTime()
        else:
            res = self.broadcastCall("gct")
            return [float(resi) for resi in res]

    def getTrajEndTime(self,h):
        return self.__methmap(h,'getTrajEndTime')

    def getTrajDuration(self,h):
        """Supports synchronized broadcast calling with h<0."""
        if h >= 0:
            return self.s[h].getTrajDuration()
        else:
            res = self.broadcastCall("gd")
            return [float(resi) for resi in res]

    def getEndConfig(self,h=-1):
        return self.__methmap(h,'getEndConfig')
    
    def getEndVelocity(self,h=-1):
        return self.__methmap(h,'getEndVelocity')

    def addMilestone(self,h,dt,q):
        if h < 0:
            raise ValueError("Cannot addMilestone using h<0, use addMilestoneAll instead.")
        return self.s[h].addMilestone(dt,q)

    def addMilestoneQuiet(self,h,dt,q):
        if h < 0:
            raise ValueError("Cannot addMilestone using h<0, use addMilestoneAllQuiet instead.")
        return self.s[h].addMilestoneQuiet(h,dt,q)

    def addMilestoneAll(self,dts,qs):
        assert(len(dts) == len(self.s)),"Times and clients much be same size"
        assert(len(qs) == len(self.s)),"Milestones and clients much be same size"
        calls = []
        for i in xrange(len(dts)):
            calls.append(("am",dts[i])+tuple(qs[i]))
        return self.callAll(calls*len(self.s),args)

    def appendMilestones(self,h,dts,qs):
        if h < 0:
            raise ValueError("Cannot appendMilestones using h<0.  Use appendMilestonesAll.")
        return self.s[h].appendMilestones(dts,qs)

    def appendMilestonesQuiet(self,dts,qs):
        if h < 0:
            raise ValueError("Cannot appendMilestonesQuiet using h<0.  Use appendMilestonesAllQuiet.")
        return self.s[h].appendMilestonesQuiet(dts,qs)

    def appendMilestonesAll(self,dts,qs):
        assert(len(dts) == len(self.s)),"Times and clients much be same size"
        assert(len(qs) == len(self.s)),"Milestones and clients much be same size"
        calls = []
        for k in range(len(dts)):
            callk = [()]*len(dts[k])
            for i in xrange(len(dts[k])):
                callk[i] = ("am",dts[k][i])+tuple(qs[k][i])
            calls.append(callk)
        return callAll(calls)

    def resetTrajectoryAbs(self,h,t):
        if h < 0:
            raise ValueError("Cannot resetTrajectoryAbs using h<0.")        
        return self.s[h].resetTrajectoryAbs(t)
        
    def resetTrajectoryRel(self,h,dt):
        if h >= 0:
            return self.s[h].resetTrajectoryRel(dt)
        else:
            return self.broadcastCall("rtrel",str(dt))

    def checkTrajectory(self,h):
        return self.__methmap('checkTrajectory',h)


class UnifiedTrajClient:
    """A multi-trajectory-client that acts in the combined state space of
    all servers."""
    def __init__(self,hosts,port=1000):
        self.s = MultiTrajClient(hosts,port)
        self.toffsets = [0.]*len(hosts)

    def slice(self,index,q):
        return q[index*6:(index+1)*6]
                  
    def echo(self,message):
        return self.s.echo(0,message)

    def version(self):
        return self.s.version(0)

    def rate(self):
        return min(self.s.rate(-1))

    def getConfig(self):
        return sum(self.s.getConfig(),[])

    def getVelocity(self):
        return sum(self.s.getVelocity(),[])

    def getTransforms(self):
        return self.s.getTransform()

    def getJointLimits(self):
        return sum(self.s.getJointLimits(),[])

    def getVelocityLimits(self):
        return sum(self.s.getVelocityLimits(),[])

    def getAccelerationLimits(self):
        return sum(self.s.getAccelerationLimits(),[])

    def getDecelerationLimits(self):
        return sum(self.s.getDecelerationLimits(),[])

    def setJointLimits(self,qmin,qmax):
        for i in xrange(len(self.s.s)):
            self.s.setJointLimits(i,self.slice(i,qmin),self.slice(i,qmax))

    def setVelocityLimits(self,vmax):
        for i in xrange(len(self.s.s)):
            self.s.setVelocityLimits(i,self.slice(i,vmax))

    def setAccelerationLimits(self,amax):
        for i in xrange(len(self.s.s)):
            self.s.setAccelerationLimits(i,self.slice(i,amax))

    def setDecelerationLimits(self,amax):
        for i in xrange(len(self.s.s)):
            self.s.setDecelerationLimits(i,self.slice(i,amax))

    def getMaxSegments(self):
        return min(self.s.getMaxSegments(-1))

    def getCurSegments(self,h):
        return max(self.s.getCurSegments(-1))
    
    def getRemainingSegments(self):
        return min(self.s.getRemainingSegments(-1))

    def getCurrentTime(self):
        ts =  self.s.getCurrentTime(-1)
        self.toffsets = [t-ts[0] for t in ts]
        return ts[0]

    def getTrajEndTime(self,h):
        return self.s.getCurrentTime(0)

    def getTrajDuration(self,h):
        return max(self.s.getTrajDuration(-1))

    def getEndConfig(self):
        return sum(self.s.getEndConfig(),[])
    
    def getEndVelocity(self):
        return sum(self.s.getEndVelocity(),[])

    def addMilestone(self,dt,q):
        n = len(self.s.s)
        return self.s.addMilestoneAll([dt]*n,[self.slice(i,q) for i in xrange(n)])

    def appendMilestones(self,dts,qs):
        n = len(self.s.s)
        return self.s.appendMilestonesAll([dts]*n,[[self.slice(i,q) for q in qs] for i in xrange(n)])
