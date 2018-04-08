#!/usr/bin/python
import sys
import csv
import math

history = 1

if len(sys.argv) < 5:
    print "Usage: %s dt keyfile datafile [files]"%(sys.argv[0],)
    print """Help:
    \tConcatenates all of the time series into one large data file.
    \tEach time series is given as a CSV file with fixed parameter keys/
    \tvalues given on the first/second line, with state keys/values given
    \ton the 3rd/3+k'th lines.
    
    \tThe parameter and state keys are written out to keyfile.

    \tIf dt is nonzero, then the time series are uniformly sampled in time
    \tusing linear interpolation.  The time variable is assumed to have key
    \t'time'.
    """
    exit(0)

historyskip = 1
dt = float(sys.argv[1])
keyfile = sys.argv[2]
datafile = sys.argv[3]

paramkeys = []
statekeys = []
rawdata = []
timeindex = -1
for i in range(4,len(sys.argv)):
    f = open(sys.argv[i],'r')
    reader = csv.reader(f)
    params = []
    trajectory = []
    for lineno,entries in enumerate(reader):
        if lineno==0:
            if paramkeys == []:
                paramkeys = entries[:]
            else:
                assert(paramkeys == entries)
        elif lineno==1:
            params = entries[:]
        elif lineno==2:
            if statekeys == []:
                statekeys = entries[:]
                timeindex = statekeys.index('time')
            else:
                assert(statekeys == entries)
        else:
            if len(trajectory) >  0 and entries[timeindex] == trajectory[-1][timeindex]:
                trajectory[-1] = entries[:]
            else:
                if len(trajectory) > 0:
                    assert(float(entries[timeindex]) > float(trajectory[-1][timeindex])),"Badly timed trajectory in file "+sys.argv[i]
                trajectory.append(entries[:])
    f.close()
    rawdata.append((params,trajectory))

#convert times to floats
for (p,t) in rawdata:
    for pt in t:
        pt[timeindex] = float(pt[timeindex])

#convert asynchronous trajectories into uniformly sampled ones starting at 0
for i in xrange(len(rawdata)):
    oldtraj = rawdata[i][1]
    t0 = float(oldtraj[0][timeindex])
    t1 = float(oldtraj[-1][timeindex])
    t1 += min(dt*5,0.5)
    n = int(math.ceil((t1-t0)/dt))
    newtraj = [None]*n
    index = 0
    for j in xrange(n):
        t = t0 + float(j)*dt
        while index+1 < len(oldtraj) and t > oldtraj[index+1][timeindex]:
            index += 1
        if index+1 < len(oldtraj):
            #interpolate
            u = (t-oldtraj[index][timeindex])/(oldtraj[index+1][timeindex]-oldtraj[index][timeindex])
            newtraj[j] = [None]*len(oldtraj[index])
            for k in xrange(len(oldtraj[index])):
                newtraj[j][k] = (1.0-u)*float(oldtraj[index][k])+u*float(oldtraj[index+1][k])-t0
        else:
            newtraj[j] = oldtraj[-1][:]
            newtraj[j][timeindex] = j*dt
    rawdata[i] = (rawdata[i][0],newtraj)



print "Writing data to",datafile
f = open(datafile,'w')
writer = csv.writer(f)
datakeys = ['trial']+paramkeys[:]+['iteration']+statekeys[:]
writer.writerow(datakeys)
for j,pt in enumerate(rawdata):
    for i in xrange(len(pt[1])):
        datum = [str(j)]+pt[0][:]
        datum += [str(i)]+pt[1][i]
        writer.writerow(datum)
f.close()

print "Writing keys to",keyfile
f = open(keyfile,'w')
writer = csv.writer(f)
writer.writerow(['trial']+paramkeys)
writer.writerow(['iteration']+statekeys)
f.close()
