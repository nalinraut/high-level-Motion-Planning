import os
import scipy as sp
import math
from training import *
from KISSLearning import *


def readLibraryCSV(fn):
    library = PrimitiveLibrary()
    with open(fn,'r') as f:
        for line in f.readlines():
            p = [float(v) for v in line.split(',')]
            library.append(Primitive(p,None,problemFeatures=p))
    return library

def readAndAssignClusters(library,problemfn,costfn,epsilon=0.1):
    trainpts = []
    with open(problemfn,'r') as f:
        for line in f.readlines():
            p = [float(v) for v in line.split(',')]
            trainpts.append(p)
    trainClusters = TrainingClusters(TrainingData(library,trainpts))
    with open(costfn,'r') as f:
        costs = trainClusters.loadCosts(f)
        trainClusters.assignThreshold(costs,0.0,epsilon)
    return trainClusters

def readClusters(library,problemfn,costfn,closefn):
    trainpts = []
    with open(problemfn,'r') as f:
        for line in f.readlines():
            p = [float(v) for v in line.split(',')]
            trainpts.append(p)
    trainClusters = TrainingClusters(TrainingData(library,trainpts))
    with open(costfn,'r') as f:
        costs = trainClusters.loadCosts(f)
        trainClusters.assignBest(costs)
    with open(closefn,'r') as f:
        for i,line in enumerate(f.readlines()):
            trainClusters.clusterClose[i] = [int(v) for v in line.split()]
            trainClusters.clusterFar[i] = []
            for j in trainClusters.trainingMatrix[i].iterkeys():
                if j not in trainClusters.clusterClose[i]:
                    trainClusters.clusterFar[i].append(j)
    return trainClusters

def writeLibraryCSV(library,fn):
    with open(fn,'w') as f:
        for p in library.primitives:
            f.write(','.join(str(v) for v in p.problemFeatures+p.solutionFeatures))
            f.write('\n')


def writeClusters(clusters,problemfn,costfn,closefn):
    with open(problemfn,'w') as f:
        for p in clusters.problemFeatures:
            f.write(','.join(str(v) for v in p))
            f.write('\n')
    with open(costfn,'w') as f:
        clusters.saveCosts(f)
    with open(closefn,'w') as f:
        for i,c in enumerate(clusters.clusterClose):
            f.write(' '.join(str(v) for v in c))
            f.write('\n')

def mahalanobisScoringFunction(M):
    return lambda x,y:mahalanobis(x.problemFeatures,y,M)

def localMahalanobisScoringFunction(Ms,library):
    """Warning: uses primitive library as temporary storage. Don't use
    the pointers from multiple invocations of this function"""
    for M,p in zip(Ms,library.primitives):
        p.M = M
    return lambda x,y:mahalanobis(x.problemFeatures,y,x.M)

def run(primitivefile,trainfile,costfile,
        testfile,outfile='assignments.txt'):
    primitives = readLibraryCSV(primitivefile)
    trainClusters = readAndAssignClusters(primitives,trainfile,costfile)

    print "Learning from",trainfile
    learner = LocalKISSLearning()
    learner.run(trainClusters)
    trainClusters = None
    
    selector = PrimitiveSelector()
    selector.selectionScore = localMahalanobisScoringFunction(learner.solution(),primitives)
    #selector.selectionScore = mahalanobisScoringFunction(sp.eye(len(primitives.primitives[0].problemFeatures)))

    print "Saving to",outfile
    fout = open(outfile,'w')
    testPoints = readLibraryCSV(testfile)
    for p in testPoints.primitives:
        (i,primitive) = selector.select(primitives,p.problem)
        print i
        fout.write("%d\n"%(i,))
    fout.close()

def weightCosts(infn,func,outfn):
    out = open(outfn,'w')
    with open(infn,'r') as f:
        for line in f.readlines():
            items = line.split()
            assert len(items)==3
            items[2] = str(func(float(items[2])))
            out.write(' '.join(items))
            out.write('\n')
            out.flush()
    out.close()
    
if __name__=="__main__":
    #print "Flipping success rates to costs..."
    #weightCosts("motion_planning_data/4Linkage folding/processed_original_data/train_rates.txt",lambda x:1.0-x,"motion_planning_data/4Linkage folding/processed_original_data/train_costs.txt")
    run("motion_planning_data/4Linkage folding/processed_original_data/primitive_features.csv",
        "motion_planning_data/4Linkage folding/processed_original_data/train_features.csv",
        "motion_planning_data/4Linkage folding/processed_original_data/train_costs.txt",
        "motion_planning_data/4Linkage folding/processed_original_data/test_features.csv")
