import csv
import sys
from collections import defaultdict
import math
import numpy as np

#settings
#python settings
time_column = 'time'
body1_column = 'body1'
body2_column = 'body2'
numContacts_column = 'numContacts'
position_columns = ['cpx_avg','cpy_avg','cpz_avg']
normal_columns = ['cnx_avg','cny_avg','cnz_avg']
force_columns = ['fx_avg','fy_avg','fz_avg']
moment_columns = ['mx_avg','my_avg','mz_avg']
"""
#SimTest settings
time_column = 'time'
body1_column = 'body1'
body2_column = 'body2'
numContacts_column = 'numContacts'
position_columns = ['cop x','cop y','cop z']
normal_columns = ['cop x','cop y','cop z']
force_columns = ['fx','fy','fz']
moment_columns = ['mx','my','mz']
"""
"""
#alessio settings
time_column = '#t'
body1_column = 'body1'
body2_column = 'body2'
numContacts_column = 'number_of_contacts'
position_columns = ['average_cp_x','average_cp_y','average_cp_z']
normal_columns = ['average_cn_x','average_cn_y','average_cn_z']
force_columns = ['f_x','f_y','f_z']
moment_columns = ['m_x','m_y','m_z']
"""

names = [time_column,body1_column,body2_column,numContacts_column]+position_columns+normal_columns+force_columns+moment_columns

def parse_contact_log(f,tmin=0,tmax=float('inf')):
    res = {}
    reader = csv.reader(f)
    headers = reader.next()
    headers = [n.strip() for n in headers]
    indices = {}
    for n in names:
        try:
            indices[n] = headers.index(n)
        except ValueError:
            indices[n] = 0
    numInContact = defaultdict(int)
    accumulators = defaultdict(lambda: defaultdict(list))
    for row in reader:
        time = float(row[indices[time_column]])
        if time < tmin or time > tmax: continue
        body1,body2 = row[indices[body1_column]].strip(),row[indices[body2_column]].strip()
        numContacts = float(row[indices[numContacts_column]])
        position = [float(row[indices[v]]) for v in position_columns]
        normal = [float(row[indices[v]]) for v in normal_columns]
        force = [float(row[indices[v]]) for v in force_columns]
        moment = [float(row[indices[v]]) for v in moment_columns]
        
        numInContact[time] += 1
        acc = accumulators[body1,body2]
        acc['time'].append(time)
        acc['numContacts'].append(numContacts)
        acc['position'].append(position)
        acc['normal'].append(normal)
        acc['force'].append(force)
        acc['moment'].append(moment)
    for pair,acc in accumulators.iteritems():
        acc['nstd'] = np.std(acc['numContacts'])
        acc['pcov'] = np.linalg.norm(np.cov(acc['position'],rowvar=0))
        acc['ncov'] = np.linalg.norm(np.cov(acc['normal'],rowvar=0))
        acc['fcov'] = np.linalg.norm(np.cov(acc['force'],rowvar=0))
        acc['fnorm'] = np.average([np.linalg.norm(f) for f in acc['force']])
    Nmin = min(numInContact.values())
    Nmax = max(numInContact.values())
    Ns = np.average([acc['nstd'] for acc in accumulators.itervalues()])
    Cf = np.average([math.sqrt(acc['fcov'])/max(1e-5,acc['fnorm']) for acc in accumulators.itervalues()])
    Cp = np.average([math.sqrt(acc['pcov']) for acc in accumulators.itervalues()])
    Cn = np.average([math.sqrt(acc['ncov']) for acc in accumulators.itervalues()])
    print "Number of bodies in contact: [%d,%d]"%(Nmin,Nmax)
    for pair,acc in accumulators.iteritems():
        print "  ",pair," time in contact: [%f,%f]"%(min(acc['time']),max(acc['time']))
    print "# contact variation:",Ns
    print "Contact force variation:",Cf
    print "Contact point variation:",Cp
    print "Contact normal variation:",Cn

fn = sys.argv[1]
f = open(fn,'r')
tmin = float(sys.argv[2])
tmax = float(sys.argv[3])
parse_contact_log(f,tmin,tmax)
