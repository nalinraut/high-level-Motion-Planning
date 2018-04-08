import sys
import os
import pickle
from collections import defaultdict

files = {}
for fn in os.listdir("world"):
    if fn.endswith(".pickle"):
        f = open("world/"+fn,'r')
        files[fn] = pickle.load(f)
        f.close()
keys = defaultdict(list)
n = 0
for (fn,values) in files.iteritems():
    keys['file'].append(fn)
    for (k,v) in values.iteritems():
        keys[k].append(v)
    for (k,v) in keys.iteritems():
        while len(v) <= n:
            v.append('')
    n += 1
print "Saving to grasp_results.csv"
f = open('grasp_results.csv','w')
f.write(','.join(sorted(keys.keys()))+'\n')
for i in range(n):
    f.write(','.join(str(keys[k][i]) for k in sorted(keys.keys())))
    f.write('\n')
f.close()
