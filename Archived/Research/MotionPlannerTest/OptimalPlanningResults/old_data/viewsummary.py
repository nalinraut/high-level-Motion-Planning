#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt
import csv
from collections import defaultdict

if len(sys.argv) < 3:
    print "Usage: viewsummary.py csvfile item"
    exit(0)

labelmap = {"lazyrrg":"Lazy RRG*",
            "prm":"PRM*",
            "lazyprm":"Lazy PRM*",
            "lazyrrg_subopt0.1":"Lazy ARRG*, eps=0.1",
            "lazyrrg_subopt0.2":"Lazy ARRG*, eps=0.2",
            "rrt":"RRT*",
            "rrt_subopt0.1":"ARRT*, eps=0.1",
            "rrt_subopt0.2":"ARRT*, eps=0.2",
}
labelorder = ["prm","rrt","rrt_subopt0.1","rrt_subopt0.2","lazyprm",
              "lazyrrg","lazyrrg_subopt0.1","lazyrrg_subopt0.2"]
dashes = [[],[8,8],[4,4],[2,2],[1,1],[12,6],[4,2,2,2],[8,2,2,2,2,2]]
ylabelmap = {"best cost":"Path length",
             "edge checks":"# edge checks",
}

timevarname = 'time'
item = sys.argv[2]
with open(sys.argv[1],'r') as f:
    reader = csv.DictReader(f)
    items = defaultdict(list)
    for row in reader:
        time = dict()
        vmean = dict()
        vstd = dict()
        vmin = dict()
        vmax = dict()
        for (k,v) in row.iteritems():
            v = float(v) if len(v) > 0 else None
            words = k.split(None,1)
            label = words[0]
            if len(words) >= 2 and words[1] == timevarname:
                time[label] = v
            if len(words) >= 2 and words[1].startswith(item):
                suffix = words[1][len(item)+1:]
                if suffix=='mean': #will have min,max,mean,etc
                    vmean[label] = v
                elif suffix=='std':
                    vstd[label] = v
                elif suffix=='max':
                    vmax[label] = v
                elif suffix=='min':
                    vmin[label] = v
                elif suffix=='':
                    vmean[label] = v
                else:
                    print "Warning, unknown suffix",suffix
            for label,t in time.iteritems():
                if label in vmean:
                    items[label].append((t,vmean[label]))

    fig = plt.figure()
    ax1 = fig.add_subplot(111)    
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel(ylabelmap.get(item,item))
    for n,label in enumerate(labelorder):
        if label not in items: continue
        plot = items[label]
        x,y = zip(*plot)
        line = ax1.plot(x,y,label=labelmap[label],dashes=dashes[n])
    plt.legend(loc='upper right');
    #good for bugtrap cost
    #plt.ylim([2,3])
    #good for other cost
    #plt.ylim([1,2])
    #good for edge checks
    plt.ylim([0,100000])
    plt.xlim([0,2])
    plt.show()
