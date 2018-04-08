#!/usr/bin/python
import subprocess as sub
import re
import random

tries = 100
timeLimit = '5'
query = '0'
planner = 'planners/rrt0_1.xml'
space = 'spaces/bugtrap_0.01.xml'

    #the ?: denotes a non-capturing grouping
floatRE = re.compile('[+-]?\d+(?:\.\d*)?(?:[e|E][+-]?\d+)?');
intRE = re.compile('[+-]?\d+');
outputRE = re.compile('(%s) iterations, (%s) milestones, (%s)s elapsed'%(intRE.pattern,intRE.pattern,floatRE.pattern))

db = open('profile_0.01.csv','w')
db.write('Iterations,Milestones,Time\n')
i=0
failures=0
while failures<tries:
    print('Iteration '+str(i))
    #run PRMBuild
    seed = random.randint(0,1000000)
    cmd = './PRMBuild.exe -iters 100000 -seed %d -time %s -query %s -planner %s %s'%(seed,timeLimit,query,planner,space)
    p = sub.Popen(cmd.split(' '),stdout=sub.PIPE,stderr=sub.PIPE)
    output, errors = p.communicate()

    #parse for running time data
    matches = outputRE.search(output)
    if matches==None:
        print('PRMBuild failed, output:')
        print(output)
        print('Errors:')
        print(errors)
        print('Return code:')
        print(p.returncode)
        print('Command:')
        print(cmd)
        failures += 1
    else:
        i+=1
        db.write("%s,%s,%s\n"%(matches.group(1),matches.group(2),matches.group(3)))
        if i >= tries:
            break;
db.close()

if failures>=tries:
    raise IOError('Failed running PRMBuild %d times'%(tries))

