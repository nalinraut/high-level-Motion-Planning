import sys
import math
from database import Database
from statcollector import StatCollector

def distance(a,b):
    return math.sqrt(sum((ai-bi)*(ai-bi)  for (ai,bi) in zip(a,b)))

if len(sys.argv) < 2:
    print "Usage: %s in.csv"%(sys.argv[0],)
    exit(0)

infile = sys.argv[1]
db = Database()
db.readCSV(infile)
print len(db.entries),"entries read"
db.cast()

trials = db.split("trial")
print len(trials),"trials"

cur_dist_stats = StatCollector()
pred_dist_stats = StatCollector()
extrap_dist_stats = StatCollector()
horizon = 40
#smooth_val = 10
smooth_val = 0
smooth_disc = 0.95
for (no,trial) in enumerate(trials):
    gxind = trial.keys.index('goal x')
    gyind = trial.keys.index('goal y')
    mxind = trial.keys.index('mouse x')
    myind = trial.keys.index('mouse y')
    pxind = trial.keys.index('pred x 40')
    pyind = trial.keys.index('pred y 40')
    cur_dist_stats_trial = StatCollector()
    pred_dist_stats_trial = StatCollector()
    extrap_dist_stats_trial = StatCollector()
    smooth_v = (0.0,0.0)
    for (i,e) in enumerate(trial.entries):
        p = max(i-smooth_val,0)
        n = min(i+horizon-1,len(trial.entries)-1)
        #if i+horizon  >= len(trial.entries):
        #    break
        g_future = (trial.entries[n][gxind],trial.entries[n][gyind])
        if i == 0:
            smooth_v = (0.0,0.0)
        elif p != i:
            #plain smoothed
            smooth_v = ((e[mxind] - trial.entries[p][mxind])/(i-p),(e[myind] - trial.entries[p][myind])/(i-p))
        else:
            #discounted
            v = ((e[mxind] - trial.entries[i-1][mxind]),(e[myind] - trial.entries[i-1][myind]))
            smooth_v = (smooth_v[0]+(1.0-smooth_disc)*(v[0]-smooth_v[0]),smooth_v[1]+(1.0-smooth_disc)*(v[1]-smooth_v[1]))

            
        cur_dist = distance(g_future,(e[mxind],e[myind]))
        pred_dist = distance(g_future,(e[pxind],e[pyind]))
        extrap_dist = distance(g_future,(e[mxind]+horizon*smooth_v[0]*1.0,e[myind]+horizon*smooth_v[1]*1.0))
        
        cur_dist_stats.collect(cur_dist)
        pred_dist_stats.collect(pred_dist)
        extrap_dist_stats.collect(extrap_dist)
        cur_dist_stats_trial.collect(cur_dist)
        pred_dist_stats_trial.collect(pred_dist)
        extrap_dist_stats_trial.collect(extrap_dist)

    print "Trial",no
    print "Current mean error:",cur_dist_stats_trial.mean,"MSE",(pow(cur_dist_stats_trial.mean,2)+cur_dist_stats_trial.var)
    print "Pred mean error:",pred_dist_stats_trial.mean,"MSE",(pow(pred_dist_stats_trial.mean,2)+pred_dist_stats_trial.var)
    print "Extrap mean error:",extrap_dist_stats_trial.mean,"MSE",(pow(extrap_dist_stats_trial.mean,2)+extrap_dist_stats_trial.var)


print "1s distances:"
print "Current mean error:",cur_dist_stats.mean,"RMSE",math.sqrt(pow(cur_dist_stats.mean,2)+cur_dist_stats.var)
print "Pred mean error:",pred_dist_stats.mean,"RMSE",math.sqrt(pow(pred_dist_stats.mean,2)+pred_dist_stats.var)
print "Extrap mean error:",extrap_dist_stats.mean,"RMSE",math.sqrt(pow(extrap_dist_stats.mean,2)+extrap_dist_stats.var)
