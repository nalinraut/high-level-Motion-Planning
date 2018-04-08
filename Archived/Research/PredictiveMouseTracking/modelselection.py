#!/usr/bin/python
import os
import subprocess

def gmmModelSelection(file,outfile):
    """Returns a pair: (modelfile,accuracy)"""
    p = subprocess.Popen("./gmmlearn %s 500 %s"%(file,outfile))
    p.wait()
    f = open('learningcurve.txt','r')
    for i,line in enumerate(f.readlines()):
        if i!=0:
            acc = float(line.split(',')[1])
    

#parameter set: history length, block size
paramvalues = [(1,5),(1,10),(1,20),
               (2,5),(2,10),(2,20),
               (3,1),(3,2),(3,5),(3,10),(3,20),
               (4,1),(4,2),(4,5),(4,10),
               (5,1),(5,2),(5,5),(5,10),
               (10,1),(10,2),(10,5)]

processdir = 'processed_data'
modeldir = 'models'
"""
infiles = ['reach_scaled.csv','reach_trans_scaled.csv',
           'traj_scaled.csv','traj_trans_scaled.csv']
outprefixes = ['reach_obs','reach_trans',
               'traj_obs','traj_trans']
"""
"""
infiles = ['reach_scaled.csv','reach_trans_scaled.csv']
outprefixes = ['reach_obs','reach_trans']
"""
infiles = ['traj_scaled.csv','traj_trans_scaled.csv']
outprefixes = ['traj_obs','traj_trans']

historyitems = ['widget dx','widget dy']

bestparams = None
bestaccuracy = -1
for (hlen,hblock) in paramvalues:
    print "History length",hlen,"block",hblock
    cmd = ""
    hblocks = []
    for i in xrange(hlen):
        hblocks.append((-(i+1)*hblock,-i*hblock-1))
        for hitem in historyitems:
            cmd += ' "%s[%d:%d]"'%(hitem,-(i+1)*hblock,-i*hblock-1)
    for infile,oprefix in zip(infiles,outprefixes):
        #make the history file
        infile = processdir+'/'+infile
        outfile = '%s/%s_h%d_b%d.csv'%(processdir,oprefix,hlen,hblock)
        res=os.system('python makehist.py %s %s'%(infile,outfile)+cmd)
        if res != 0: exit(0)

        #drop the trial variable
        res=os.system("""python process.py %s %s \"delete_key 'trial'\""""%(outfile,outfile))
        if res != 0: exit(0)

        """
        #cross validate the GMM
        gmmfile = '%s/%s_h%d_b%d.gmm'%(modeldir,oprefix,hlen,hblock)
        res,accuracy = gmmModelSelection(outfile,gmmfile)
        if accuracy > bestaccuracy:
            bestaccuracy = accuracy
            bestparams = (hlen,hblock)
        """
print "Done."



