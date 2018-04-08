import sys
import csv
import re

if len(sys.argv) < 3:
    print "Usage: make_relative_coords.py in out"

infile = sys.argv[1]
outfile = sys.argv[2]

keys = []
data = []
f = open(sys.argv[i],'r')
reader = csv.reader(f)
for lineno,entries in enumerate(reader):
    if lineno==0:
        keys = entries[:]
    else:
        data.append(entries[:])
    f.close()

refx_key = 'target x'
refy_key = 'target y'
refx_index = keys.index(refx_key)
refy_index = keys.index(refy_key)

ptx_key = 'widget x'
pty_key = 'widget y'
ptx_key_out = 'widget f'
pty_key_out = 'widget l'
history_pattern = '( [[+-]?\d+])'
history_suffix = ' [%d]'
ptx_history = re.compile(ptx_key+history_pattern)
pty_history = re.compile(pty_key+history_pattern)
history = set()
for i,k in enumerate(keys):
    match=ptx_history.match(k)
    if match:
        history.add(match.group(1))
    match=pty_history.match(k)
    if match:
        history.add(match.group(1))
print list(history)

out_keys = ptx_map.keys()+pty_map.keys()
for (i,k) in enumerate(keys):
    if k!=refx_key and k!=refy_key and i not in ptx_indices.values() and i not in pty_indices.values():
        out_keys.append(k)
out_keys.sort()
keyindex = dict(zip(keys,range(len(keys))))
outkeyindex = dict(zip(out_keys,range(len(out_keys))))

f = open(outfile,'w')
writer = csv.writer(f)
writer.writerow(out_keys)
for pt in xrange(data):
    datum = []
    for ptx_map
    for k in out_keys:
        if k in keyindex:
            datum.append(pt[keyindex[k]])
        else:
f.close()
