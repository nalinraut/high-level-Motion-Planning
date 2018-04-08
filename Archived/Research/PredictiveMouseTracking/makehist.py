#!/usr/bin/python
import sys
from timeseriesdb import TimeSeriesDatabase


if len(sys.argv) < 3:
    print "Usage: %s in.csv out.csv [vars]"%(sys.argv[0],)
    exit(0)

infile = sys.argv[1]
outfile = sys.argv[2]
db = TimeSeriesDatabase()
db.readCSV(infile)
print len(db.entries),"entries read"

hvars = set()

assignments = []
for var in sys.argv[3:]:
    assert var[-1]==']'
    ind = var.rfind('[')
    vbase = var[:ind]
    vslice = var[ind+1:-1]
    assignment = '%s = {%s}[%s]'%(var,vbase,vslice)
    hvars.add(vbase)
    print assignment
    assignments.append(assignment)

oldkeylen = len(db.keys)

db.process(assignments)

#shuffle assignments so that assignments are right after history vars
lasthvar = max(db.keys.index(h) for h in hvars)
newkeys = db.keys[:lasthvar+1]+db.keys[oldkeylen:]+db.keys[lasthvar+1:oldkeylen]
db.shuffle_keys(newkeys)


print "Writing to",outfile
db.writeCSV(outfile)

