import time
from trajclient import *
from safetrajclient import *
from multitrajclient import *

#This tests the read-only trajectory client
def testReadOnly(s):
    print s.getConfig()
    #The following call should generate an error
    s.addMilestone(1,[0,0,0,0,0,0])
    return

#This tests a triangle wave on joint 2, starting at the zero position
def testTriangleWave(s,joint=2,deg=10,period=4):
    q = s.getEndConfig()
    q2 = q[:]
    q2[joint] += deg
    #triangle wave
    s.addMilestone(period*0.5,q2)
    q2[joint] -= 2*deg
    s.addMilestone(period,q2)
    s.addMilestone(period*0.5,q)
    return

def testQueries(s):
    print "Joint limits:"
    print (s.getJointLimits())
    print "Velocity limits:"
    print (s.getVelocityLimits())
    print "Acceleration limits:"
    print (s.getAccelerationLimits())
    print "Deceleration limits:"
    print (s.getAccelerationLimits())


def testSafe(host="127.0.0.1"):
    #This tests the safe trajectory client
    print "Starting safe trajectory test..."
    s = SafeTrajClient(host)
    print "Calibrating latency..."
    s.calibrateLatency()
    print "Connection latency: ",s.latency
    q = s.client().getEndConfig()
    q2 = q[:]
    q2[2] += 10
    s.moveto(q2)
    s.moveto(q)

def testPing(s,iters=1000):
    index = 0
    while index < iters:
        t1 = time.time()
        res = s.echo(str(index));
        if res != str(index):
            print "Warning, echo ID incorrect, '%s' instead of %s"%(res,str(index))
        t2 = time.time()
        print('Round trip time %g'%(t2-t1))
        index+=1

def testTiming(s,iters=10,batchsize=99):
    """This function tests the communication latency/bandwidth by sending
    batched vs nonbatched gx calls."""

    qstart = s.getEndConfig();
    gxcall = "gj"
    amcall = ("am",0.004)+tuple(qstart)

    index = 0
    while index < iters:
        t0=time.time()
        res = s.call([gxcall]*batchsize)
        assert not 'Error' in res,'Error on batch '+str(gxcall)
        print("Time for %d gx's (batched): %f"%(batchsize,time.time()-t0))
        
        t0=time.time()
        for i in xrange(batchsize):
            res = s.call(gxcall)
            assert res != 'Error','Error on '+str(gxcall)
        print("Time for %d gx's (unbatched): %f"%(batchsize,time.time()-t0))

        t0=time.time()
        res = s.call([amcall]*batchsize)
        assert not 'Error' in res,'Error on batch '+str(amcall)
        print("Time for %d am's (batched): %f"%(batchsize,time.time()-t0))
        t0=time.time()

        t0=time.time()
        s.call([amcall]*batchsize,wantReply=False)
        print("Time for %d am's (batched,noreply): %f"%(batchsize,time.time()-t0))
        ret = s.echo("blah")
        assert ret == "blah"
        print("     Responsive after time %f"%(time.time()-t0))

        t0=time.time()
        for i in xrange(batchsize):
            res=s.call(amcall)
            assert 'Error' != res,'Error on '+str(amcall)
        print("Time for %d am's (unbatched): %f"%(batchsize,time.time()-t0))

        t0=time.time()
        for i in xrange(batchsize):
            s.call(amcall,wantReply=False)
        print("Time for %d am's (unbatched,noreply): %f"%(batchsize,time.time()-t0))
        ret = s.echo("blah")
        assert ret == "blah"
        print("     Responsive after time %f"%(time.time()-t0))
       
        index += 1

leftarm = '10.10.1.202'
rightarm = '10.10.1.203'
localarm = 'localhost'
#host = rightarm
host = localarm
#testSafe(host)
#exit()

#right arm
c=TrajClient(host)
testTriangleWave(c,deg=50)
#testPing(c,100)
#testTiming(c)
#testSafe("localhost")

