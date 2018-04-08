from __future__ import with_statement
import sys
import csv

key = '#iters'
field = 'solution cost'
rowcnt = []
rowsum = []
rowsumsq = []

solvetimecnt = 0
solvetimesum = 0

for fn in sys.argv[1:]:
    with open(fn,'r') as csvfile:
        reader = csv.DictReader(csvfile)
        vmin = float('inf')
        for r in reader:
            if r['op']!='e': continue
            index = int(r[key])
            while index >= len(rowcnt):
                rowcnt.append(0)
                rowsum.append(0)
                rowsumsq.append(0)
            v=float(r[field])
            if vmin > 1e100 and v < vmin:
                #first solve
                solvetimecnt += 1
                solvetimesum += float(r['time'])
            if v < vmin:
                vmin = v
            if vmin > 1e100:
                continue
            rowcnt[index] += 1
            rowsum[index] += vmin
            rowsumsq[index] += vmin*vmin

print 'Average solve time:',solvetimesum/solvetimecnt
print 'cnt,mean,var'
for (c,s,ss) in zip(rowcnt,rowsum,rowsumsq):
    if c==0:
        print '0,,'
    else:
        print '%d,%g,%g'%(c,s/c,ss/c-(s/c)**2)

