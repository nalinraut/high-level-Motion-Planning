import sys
import json
from database import Database

f = open(sys.argv[1],'r')
entries = []
for line in f.readlines():
	v = json.loads(line)
	entries.append(v)

db = Database()
keys = set()
for e in entries:
	for k in e.keys():
		keys.add(k)
keylist = sorted(list(keys))
keydict = dict((k,i) for (i,k) in enumerate(keylist))
db.keys = keylist
for e in entries:
	row = [None]*len(keydict)
	for (k,v) in e.iteritems():
		row[keydict[k]] = v
	db.entries.append(row)

db.writeCSV(sys.argv[2])
