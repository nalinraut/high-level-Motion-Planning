import sys
import math
from database import Database
from statcollector import StatCollector

if len(sys.argv) < 3:
    print "Usage: %s in.csv out.csv"%(sys.argv[0],)
    exit(0)

infile = sys.argv[1]
outfile = sys.argv[2]
db = Database()
db.readCSV(infile)
print len(db.entries),"entries read"
db.cast()

trials = db.split("trial")
print len(trials),"trials"

#find average length
avglen = sum(len(t.entries) for t in trials)/len(trials)*2
print "Discretizing time into",avglen,"steps"

cursorfwd = [StatCollector() for i in range(avglen)]
cursorrgt = [StatCollector() for i in range(avglen)]
cursord = [StatCollector() for i in range(avglen)]
predfwd = [StatCollector() for i in range(avglen)]
predrgt = [StatCollector() for i in range(avglen)]
predd = [StatCollector() for i in range(avglen)]
for trial in trials:
    gxind = trial.keys.index('goal x')
    gyind = trial.keys.index('goal y')
    mxind = trial.keys.index('mouse x')
    myind = trial.keys.index('mouse y')
    mdind = trial.keys.index('mouse dist')
    pxind = trial.keys.index('pred x')
    pyind = trial.keys.index('pred y')
    pdind = trial.keys.index('pred dist')
    gdir = (trial.entries[0][gxind],trial.entries[0][gyind])
    glen = math.sqrt(gdir[0]*gdir[0]+gdir[1]*gdir[1])
    gfwd = (gdir[0]/(glen*glen),gdir[1]/(glen*glen))
    grgt = (gfwd[1],-gfwd[0])
    #split t into avglen parts
    for i in range(avglen):
        t = float(i)/float(avglen-1)*len(trial.entries)
        index = int(math.floor(t))
        u = t-index
        if index+1 >= len(trial.entries):
            index=len(trial.entries)-2
            u = 1.0
        m0 = [trial.entries[index][j] for j in [mxind,myind,mdind]]
        m1 = [trial.entries[index+1][j] for j in [mxind,myind,mdind]]
        p0 = [trial.entries[index][j] for j in [pxind,pyind,pdind]]
        p1 = [trial.entries[index+1][j] for j in [pxind,pyind,pdind]]
        mx = m0[0]+u*(m1[0]-m0[0])
        my = m0[1]+u*(m1[1]-m0[1])
        md = m0[2]+u*(m1[2]-m0[2])
        px = p0[0]+u*(p1[0]-p0[0])
        py = p0[1]+u*(p1[1]-p0[1])
        pd = p0[2]+u*(p1[2]-p0[2])
        cursorfwd[i].collect(mx*gfwd[0]+my*gfwd[1])
        cursorrgt[i].collect(mx*grgt[0]+my*grgt[1])
        cursord[i].collect(md/glen)
        predfwd[i].collect(px*gfwd[0]+py*gfwd[1])
        predrgt[i].collect(px*grgt[0]+py*grgt[1])
        predd[i].collect(pd/glen)

dbnext = Database()
dbnext.keys = ["cursor fwd","cursor fwd std","cursor rgt","cursor rgt std","cursor dist","cursor dist std",
               "pred fwd","pred fwd std","pred rgt","pred rgt std","pred dist","pred dist std"]
dbnext.entries = zip([s.mean for s in cursorfwd],
                     [math.sqrt(s.var) for s in cursorfwd],
                     [s.mean for s in cursorrgt],
                     [math.sqrt(s.var) for s in cursorrgt],
                     [s.mean for s in cursord],
                     [math.sqrt(s.var) for s in cursord],
                     [s.mean for s in predfwd],
                     [math.sqrt(s.var) for s in predfwd],
                     [s.mean for s in predrgt],
                     [math.sqrt(s.var) for s in predrgt],
                     [s.mean for s in predd],
                     [math.sqrt(s.var) for s in predd])
dbnext.writeCSV(outfile)
