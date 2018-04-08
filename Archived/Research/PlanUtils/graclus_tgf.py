#!/usr/bin/python
import sys
import os

GRACLUS = '~/graclus1.2/graclus'

os.system('./tgf2graclus.py '+sys.argv[1])
os.system(GRACLUS+' graph.grac '+sys.argv[2])
os.system('mv graph.grac.part.%s %s.%s.labels'%(sys.argv[2],sys.argv[1],sys.argv[2]))
os.system('cp %s roadmap.tgf'%(sys.argv[1]))
os.system('cp %s.%s.labels roadmap.labels'%(sys.argv[1],sys.argv[2]))

os.system('rm graph.grac')

 
