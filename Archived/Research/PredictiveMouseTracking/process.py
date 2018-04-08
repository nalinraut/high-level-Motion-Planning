import sys
import csv
import re
import copy
import math
import random
from database import Database
from timeseriesdb import TimeSeriesDatabase


def do_reach_default(db,obsfile,transfile,initfile,scalefile):
    drop = ['iteration','time','target x','target y','widget x','widget y']
    process = ["target rel x = {target x}-{widget x}",
       "target rel y = {target y}-{widget y}",
       "widget dx = {widget x}-{widget x}[-1]",
       "widget dy = {widget y}-{widget y}[-1]",
       ]
    pflipx = ["target rel x = -{target rel x}",
              "widget dx = -{widget dx}"]
    pflipy = ["target rel y = -{target rel y}",
              "widget dy = -{widget dy}"]
    historyvars = ["widget dx","widget dy"]

    db.process(process)
    trials = db.split("trial")
    print len(trials),"trials"
    newkeys = db.keys[:]
    newkeys = newkeys[:db.keys.index("widget dx")]+["speed"]+newkeys[db.keys.index("widget dx"):]
    for trial in trials:
        dt = float(trial.get(-1,"time"))-float(trial.get(0,"time"))
        dx = float(trial.get(0,"target rel x"))
        dy = float(trial.get(0,"target rel y"))
        dist = max(math.sqrt(dx*dx+dy*dy)-float(trial.get(0,"target r")),0.0)
        trial.process("speed = %g"%(dist / dt,))
        trial.shuffle_keys(newkeys)
        trial.delete_key(*drop)

    #construct new trials by flipping x and y
    newtrials = trials[:]
    for trial in trials:
        flipx = Database(trial)
        flipx.process(pflipx)
        newtrials.append(flipx)
        flipy = Database(trial)
        flipy.process(pflipy)
        newtrials.append(flipy)
        flipxy = Database(flipy)
        flipxy.process(pflipx)
        newtrials.append(flipxy)
    trials = newtrials

    db.keys = []
    db.entries = []
    db.addTrials(trials)

    mean = db.mean()
    stdev = db.stdev()
    dbscale = Database()
    dbscale.keys = ["key","mean","stdev"]
    dbscale.entries = [[k,m,s] for (k,m,s) in zip(db.keys,mean,stdev)]
    print "Writing scale to %s"%(scalefile,)
    dbscale.writeCSV(scalefile)

    #shift and scale the db
    for i in xrange(1,len(db.keys)):
        for e in db.entries:
            if stdev[i] != 0.0:
                e[i] = (float(e[i])-mean[i])/stdev[i]
    #shift and scale the trials
    for trial in trials:
        for i in xrange(1,len(trial.keys)):
            for e in trial.entries:
                if stdev[i] != 0.0:
                    e[i] = (float(e[i])-mean[i])/stdev[i]
    print len(db.entries),"entries processed"
    print "Writing observations to",obsfile
    db.writeCSV(obsfile)

    print "Writing initial conditions to",initfile
    initdb = Database()
    initdb.keys = db.keys
    initdb.entries = [trial.entries[0] for trial in trials]
    initdb.writeCSV(initfile)

    duplicateStart = 10
    duplicateEnd = 20
    #numbetween = len(trials)/2
    dbtrans = TimeSeriesDatabase()
    #print "Adding",len(db.entries)-len(trials)+(duplicateEnd+duplicateStart)*len(trials),"within-trial transitions"
    if duplicateStart+duplicateEnd > 0: print "Duplicating",duplicateStart,"start steps and",duplicateEnd,"terminal steps"
    for trial in trials:
        prefix = [trial.entries[0][:] for i in xrange(duplicateStart)]
        suffix = [trial.entries[-1][:] for i in xrange(duplicateEnd)]
        trial.entries = prefix + trial.entries + suffix
        trialnext = Database()
        trialnext.keys = trial.keys[:]
        trialnext.entries = copy.deepcopy(trial.entries[1:])
        trialnext.delete_key(*historyvars)
        trialnext.keys = [k+" next" for k in trialnext.keys if k !="trial"]
        trialtrans = Database(trial)
        trialtrans.entries.pop(-1)
        trialtrans.join(trialnext)
        dbtrans.addTrial(trialtrans)

    """
    print "Adding",numbetween,"between-trial transitions"
    
    for iter in xrange(numbetween):
        trial1 = random.choice(trials)
        trial2 = random.choice(trials)
        
        dbtrans.entries.append(trial1.entries[-1][:])
        next = Database()
        next.keys = trial1.keys[:]
        next.entries = [trial2.entries[0][:]]
        next.delete_key(*historyvars)
        dbtrans.entries[-1].extend(next.entries[0])
    dbtrans.delete_key('trial')
    """
    print "Writing transitions to %s"%(transfile)
    dbtrans.writeCSV(transfile)

def do_traj_default(db,obsfile,transfile,initfile,meanscalefile):
    drop = ['iteration','time','slider x','slider y','pattern','height','width','speed','slider param','slider velocity x','slider velocity y','widget x','widget y']
    process = ["target rel x = {slider x}-{widget x}",
       "target rel y = {slider y}-{widget y}",
       "target dx  = {slider velocity x}",
       "target dy  = {slider velocity y}",
       "widget dx = {widget x}-{widget x}[-1]",
       "widget dy = {widget y}-{widget y}[-1]",
       ]
    pflipx = ["target rel x = -{target rel x}",
              "target dx = -{target dx}",
              "widget dx = -{widget dx}"]
    pflipy = ["target rel y = -{target rel y}",
              "target dy = -{target dy}",
              "widget dy = -{widget dy}"]
    exchxy = ["target rel y = {target rel x}",
              "target rel x = {target rel y}",
              "target dx = {target dy}",
              "target dy = {target dx}",
              "widget dy = {widget dx}",
              "widget dx = {widget dy}"]
    historyvars = ["widget dx","widget dy"]

    db.process(process)
    trials = db.split("trial")
    print len(trials),"trials"

    for trial in trials:
        trial.delete_key(*drop)

    #construct new trials by flipping x and y
    newtrials = trials[:]
    for trial in trials:
        flipx = Database(trial)
        flipx.process(pflipx)
        newtrials.append(flipx)
        flipy = Database(trial)
        flipy.process(pflipy)
        newtrials.append(flipy)
        flipxy = Database(flipy)
        flipxy.process(pflipx)
        newtrials.append(flipxy)

        #swap the x-y coordinates too to be isometric
        exch = Database(trial)
        exch.process(exchxy)
        newtrials.append(exch)
        eflipx = Database(flipx)
        eflipx.process(exchxy)
        newtrials.append(eflipx)
        eflipy = Database(flipy)
        eflipy.process(exchxy)
        newtrials.append(eflipy)
        eflipxy = Database(flipxy)
        eflipxy.process(exchxy)
        newtrials.append(eflipxy)
    trials = newtrials

    print trials[6].keys
    print trials[6].entries[0]

    db.keys = []
    db.entries = []
    db.addTrials(trials)

    mean = db.mean()
    stdev = db.stdev()
    dbscale = Database()
    dbscale.keys = ["key","mean","stdev"]
    dbscale.entries = [[k,m,s] for (k,m,s) in zip(db.keys,mean,stdev)]
    print "Writing scale to",meanscalefile
    dbscale.writeCSV(meanscalefile)
    #shift and scale the db
    for i in xrange(1,len(db.keys)):
        for e in db.entries:
            if stdev[i] != 0.0:
                e[i] = (float(e[i])-mean[i])/stdev[i]
    #shift and scale the trials
    for trial in trials:
        for i in xrange(1,len(trial.keys)):
            for e in trial.entries:
                if stdev[i] != 0.0:
                    e[i] = (float(e[i])-mean[i])/stdev[i]
    print len(db.entries),"entries processed"

    print "After scaling"
    print trials[6].keys
    print trials[6].entries[0]
    
    print "Writing observations to",obsfile
    db.writeCSV(obsfile)

    print "Writing initial conditions to",initfile
    initdb = Database()
    initdb.keys = db.keys
    initdb.entries = [trial.entries[0] for trial in trials]
    initdb.writeCSV(initfile)

    duplicate = 0
    numbetween = len(trials)*2
    dbtrans = TimeSeriesDatabase()
    print "Adding",len(db.entries)-len(trials)+duplicate*len(trials),"within-trial transitions"
    if duplicate > 0: print "Duplicating",duplicate,"terminal steps"
    for trial in trials:
        for i in xrange(duplicate):
            trial.entries.append(trial.entries[-1][:])
        trialnext = Database()
        trialnext.keys = trial.keys[:]
        trialnext.entries = copy.deepcopy(trial.entries[1:])
        trialnext.delete_key(*historyvars)
        trialnext.keys = [k+" next" for k in trialnext.keys if k != "trial"]
        trialtrans = Database(trial)
        trialtrans.entries.pop(-1)
        trialtrans.join(trialnext)
        dbtrans.addTrial(trialtrans)

    print "Adding",numbetween,"between-trial transitions"
    
    for iter in xrange(numbetween):
        trial1 = random.choice(trials)
        trial2 = random.choice(trials)
        
        dbtrans.entries.append(trial1.entries[-1][:])
        next = Database()
        next.keys = trial1.keys[:]
        next.entries = [trial2.entries[0][:]]
        next.delete_key(*historyvars)
        dbtrans.entries[-1].extend(next.entries[0])
    print "Writing transitions to "+transfile
    dbtrans.writeCSV(transfile)


db = TimeSeriesDatabase()        
if len(sys.argv) == 2:
    if sys.argv[1]=='reach':
        db.readCSV('processed_data/reach.csv')
        print len(db.entries),"entries read"
        do_reach_default(db,'processed_data/reach_scaled.csv',
                         'processed_data/reach_trans_scaled.csv',
                         'processed_data/reach_init_scaled.csv',
                         'processed_data/reach-mean-scale.csv')
    elif sys.argv[1]=='traj':
        db.readCSV('processed_data/traj.csv')
        print len(db.entries),"entries read"
        do_traj_default(db,'processed_data/traj_scaled.csv',
                        'processed_data/traj_trans_scaled.csv',
                        'processed_data/traj_init_scaled.csv',
                        'processed_data/traj-mean-scale.csv')
    else:
        print "Usage: %s {reach,traj}"%(sys.argv[0],)
        exit(0)
else:
    if len(sys.argv) < 3:
        print "Usage: %s in.csv out.csv [commands]"%(sys.argv[0],)
        exit(0)

    infile = sys.argv[1]
    outfile = sys.argv[2]
    db.readCSV(infile)
    print len(db.entries),"entries read"

    for cmd in sys.argv[3:]:
        func,arg=cmd.split(" ",1)
        eval("db."+func+"("+arg+")")

    print len(db.entries),"entries processed"
    db.writeCSV(outfile)
