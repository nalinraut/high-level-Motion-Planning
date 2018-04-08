import os
import scipy as sp
import math
from collections import defaultdict
from training import *
from covarianceLearning import *
from LMNNLearning import GlobalLMNNLearning
from KISSLearning import *
from localLMNNLearning import LocalLMNNLearning
from SVMLearning import GlobalSVMLearning,LocalSVMLearning
from directLearning import *
from regressionLearning import *
from plotclusters import *

class EuclideanLearning:
    def __init__(self):
        self.M = None
    def run(self,clusters):
        self.M = sp.eye(clusters.problemSpaceDims())
        return
    def solution(self):
        return self.M

learners = {'euclidean':EuclideanLearning(),
            'global covariance':GlobalCovarianceLearning(),
            #'global SVM':GlobalSVMLearning(),
            #'global SVM diag':GlobalSVMLearning(mask='diag'),
            #'global LMNN':GlobalLMNNLearning(),
            #'global LMNN diag':GlobalLMNNLearning(mu=0.1,mask='diag'),
            #'direct soft arg min':GlobalDirectLearning('softargmin'),
            #'direct soft arg min diag':GlobalDirectLearning('softargmin',mask='diag'),
            #'direct softmargin':GlobalDirectLearning('softmargin'),
            #'direct softmargin diag':GlobalDirectLearning('softmargin',mask='diag'),
            'global KISS diag':GlobalKISSLearning(mask='diag'),
            'global KISS':GlobalKISSLearning(),
            #'local KISS':LocalKISSLearning(),
            'local KISS diag':LocalKISSLearning(mask='diag'),
            #'regression':RegressionLearning(diag=True),
            #'regression full':RegressionLearning(diag=False),
            #'local covariance':LocalCovarianceLearning(),
            #'local SVM':LocalSVMLearning(),
            #'local LMNN':LocalLMNNLearning(),
            }

def transformsingle(p,power, noise):
    newthings = [0]*len(p)
    for i,v in enumerate(p):
        newthings[i] = v**power
    for i in range(0, noise):    
        newthings.append(sp.random.uniform(0.0, 1.0))
    return newthings

def mahalanobisScoringFunction(M):
    return lambda x,y:mahalanobis(x.problemFeatures,y,M)

def localMahalanobisScoringFunction(Ms,library):
    """Warning: uses primitive library as temporary storage. Don't use
    the pointers from multiple invocations of this function"""
    for M,p in zip(Ms,library.primitives):
        p.M = M
    return lambda x,y:mahalanobis(x.problemFeatures,y,x.M)

def save_assignments_csv(clusters,assignments,filename='clusters.csv'):
    f = open(filename,'w')
    problemIndices = defaultdict(list)
    for i,p in enumerate(assignments):
        problemIndices[p].append(i)
    for k,clusterIndices in problemIndices.iteritems():
        p = clusters.library.primitives[k].problemFeatures
        f.write('cluster_id,'+','.join(['center_x'+str(i+1) for i in range(len(p))])+'\n')
        f.write(str(k)+','+','.join(str(v) for v in p)+'\n')
        f.write('point_id,'+','.join(['point_x'+str(i+1) for i in range(len(p))])+'\n')
        for c in clusterIndices:
            q = clusters.problemFeatures[c]
            f.write(str(c)+','+','.join([str(v) for v in q])+'\n')
        f.write('\n')
    f.close()

def save_clusters_csv(clusters,scorefunc,filename='clusters.csv'):
    """Writes out clusters assigned by the given scoring function in csv format"""
    assignments = clusters.evalAssignments(scorefunc)
    save_assignments_csv(clusters,assignments,filename)

def save_matrix(mat,filename):
    with open(filename,'w') as f:
        for i in xrange(mat.shape[0]):
            f.write(' '.join(str(v.real) for v in mat[i,:]))
            f.write('\n')

def runTrainingRaw(trainClusters,prefix):
    """Returns true if any learner is run"""
    #get stats
    nprim = len(trainClusters.library.primitives)
    nedges = sum(len(v) for v in trainClusters.trainingMatrix)
    closeStats = stats([len(c) for c in trainClusters.clusterClose])
    farStats = stats([len(c) for c in trainClusters.clusterFar])
    print "%d primitives, %d edges, close set [%d,%d] avg %f, far set [%d,%d] avg %f"%(nprim,nedges,closeStats.min,closeStats.max,closeStats.avg,farStats.min,farStats.max,farStats.avg)
    print "Minimum train loss %f"%(trainClusters.minimumLoss(),)
    res = False
    for name,learner in learners.iteritems():
        if os.path.exists(prefix+'/results/'+name+'.txt'):
            print "Skipping learner",name
            continue
        res = True
        print "Running learner",name
        learner.logPrefix = prefix
        learner.run(trainClusters)
        s = learner.solution()
        if isinstance(s,(tuple,list)):
            score = localMahalanobisScoringFunction(learner.solution(),trainClusters.library)
        else:
            score = mahalanobisScoringFunction(s)
        assignments = trainClusters.evalAssignments(score)
        loss = trainClusters.assignmentLoss(assignments)
        error = trainClusters.assignmentError(assignments)
        clustererror = trainClusters.assignmentClusterError(assignments)
        with open(prefix+'/results/'+name+'.txt','a') as f:
            f.write(name + " train loss %f error %f clustererror %f\r\n"%(loss,float(error)/len(trainClusters.problems),float(clustererror)/len(trainClusters.problems)))
            f.write('\n')
        #csvfile = prefix+'/results/'+name+' train clusters.csv'
        #save_clusters_csv(trainClusters,score,csvfile)
        
        #save matrix
        if not isinstance(s,(tuple,list)):
            save_matrix(s,prefix+'/results/'+name+' parameters.txt')
        
    return res


def runTestsRaw(trainClusters,testClusters,prefix):
    """Returns true if any learner is run"""
    #get stats
    nprim = len(trainClusters.library.primitives)
    nedges = sum(len(v) for v in trainClusters.trainingMatrix)
    closeStats = stats([len(c) for c in trainClusters.clusterClose])
    farStats = stats([len(c) for c in trainClusters.clusterFar])
    print "%d primitives, %d edges, close set [%d,%d] avg %f, far set [%d,%d] avg %f"%(nprim,nedges,closeStats.min,closeStats.max,closeStats.avg,farStats.min,farStats.max,farStats.avg)
    print "Minimum train loss %f, test loss %f"%(trainClusters.minimumLoss(),testClusters.minimumLoss())
    res = False
    for name,learner in learners.iteritems():
        if os.path.exists(prefix+'/results/'+name+'.txt'):
            print "Skipping learner",name
            continue
        res = True
        print "Running learner",name
        learner.logPrefix = prefix
        learner.run(trainClusters)
        s = learner.solution()
        if isinstance(s,(tuple,list)):
            score = localMahalanobisScoringFunction(learner.solution(),trainClusters.library)
        else:
            score = mahalanobisScoringFunction(s)
        assignments = trainClusters.evalAssignments(score)
        loss = trainClusters.assignmentLoss(assignments)
        error = trainClusters.assignmentError(assignments)
        clustererror = trainClusters.assignmentClusterError(assignments)
        with open(prefix+'/results/'+name+'.txt','a') as f:
            f.write(name + " train loss %f error %f clustererror %f\r\n"%(loss,float(error)/len(trainClusters.problems),float(clustererror)/len(trainClusters.problems)))
            f.write('\n')
        csvfile = prefix+'/results/'+name+' train clusters.csv'
        save_clusters_csv(trainClusters,score,csvfile)
        #save matrix
        if not isinstance(s,(tuple,list)):
            save_matrix(s,prefix+'/results/'+name+' parameters.txt')
        
        if isinstance(s,(tuple,list)):
            score = localMahalanobisScoringFunction(s,testClusters.library)
        else:
            score = mahalanobisScoringFunction(s)
        assignments = testClusters.evalAssignments(score)
        loss = testClusters.assignmentLoss(assignments)
        error = testClusters.assignmentError(assignments)
        clustererror = testClusters.assignmentClusterError(assignments)
        with open(prefix+'/results/'+name+'.txt','a') as f:
            f.write(name + " test loss %f error %f clustererror %f\r\n"%(loss,float(error)/len(testClusters.problems),float(clustererror)/len(testClusters.problems)))
        csvfile = prefix+'/results/'+name+' test clusters.csv'
        save_clusters_csv(testClusters,score,csvfile)

    return res

def prepareDirectories(root):
    numtrials = 1
    prefix = root+str(numtrials)
    while os.path.exists(prefix):
        numtrials += 1
        prefix = root+str(numtrials)
    print "Saving to new folder",prefix
    os.makedirs(prefix)
    if not os.path.exists(prefix+'/data'):
        os.makedirs(prefix+'/data')
    if not os.path.exists(prefix+'/dumps'):
        os.makedirs(prefix+'/dumps')
    if not os.path.exists(prefix+'/logs'):
        os.makedirs(prefix+'/logs')
    if not os.path.exists(prefix+'/results'):
        os.makedirs(prefix+'/results')
    return prefix

def readLibraryCSV(fn):
    library = PrimitiveLibrary()
    with open(fn,'r') as f:
        for line in f.readlines():
            p = [float(v) for v in line.split(',')]
            library.append(Primitive(p,None,problemFeatures=p))
    return library

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
    
def runNewTests(trainClusters,testClusters,prefix):
    writeLibraryCSV(trainClusters.library,prefix+'/data/primitive_features.csv')
    writeClusters(trainClusters,prefix+'/data/train_features.csv',prefix+'/data/train_costs.txt',prefix+'/data/train_close.txt')
    writeClusters(testClusters,prefix+'/data/test_features.csv',prefix+'/data/test_costs.txt',prefix+'/data/test_close.txt')
    save_assignments_csv(trainClusters,trainClusters.problemToBestPrimitive,prefix+'/data/train_clusters.csv')
    save_assignments_csv(testClusters,testClusters.problemToBestPrimitive,prefix+'/data/test_clusters.csv')
    return runTestsRaw(trainClusters,testClusters,prefix)


def rerunTests(prefix):
    library = readLibraryCSV(prefix+'/data/primitive_features.csv')
    trainClusters = readClusters(library,prefix+'/data/train_features.csv',prefix+'/data/train_costs.txt',prefix+'/data/train_close.txt')
    testClusters = readClusters(library,prefix+'/data/test_features.csv',prefix+'/data/test_costs.txt',prefix+'/data/test_close.txt')
    return runTestsRaw(trainClusters,testClusters,prefix)

def runTests(trainClusters,testClusters,root,description,testNew):
    """A testing program that is robust to failures, and will restart at
    old failures."""
    numtrials = 1
    prefix = root+str(numtrials)
    oldTests = False
    while os.path.exists(prefix):
        print "Rerunning tests for existing folder",prefix
        if rerunTests(prefix):
            oldTests = True
        numtrials += 1
        prefix = root+str(numtrials)
    if not testNew:
        #done!
        return
    if not oldTests:
        prefix = prepareDirectories(root)
        #describe the synthetic transformation
        with open(prefix+'/description.txt', 'w') as xform:
            xform.write(description)
            xform.write('\n')
            xform.write("# primitives %d, # train %d, #test %d\n"%(len(trainClusters.library.primitives),len(trainClusters.problems),len(testClusters.problems)))
            xform.write("# positive train edges %d, # negative train edges %d\n"%(sum([len(c) for c in trainClusters.clusterClose]),sum([len(c) for c in trainClusters.clusterFar])))
            xform.write("Minimum train loss %f, test loss %f\n"%(trainClusters.minimumLoss(),testClusters.minimumLoss()))
        runNewTests(trainClusters,testClusters,prefix)
    else:
        print "Not running new tests this time, run again to start a new test"

#problem parameters and names
syntheticTests = {'synthetic_2d_noise':{'dims':2,'power':2,'noise':1,
                    'nprim':5,'ntrain':300,'ntest':1000,'nedgesperprim':None,
                    'suboptimality':0.1},
                  'synthetic_2d_noise_warp':{'dims':2,'power':5,'noise':1,
                    'nprim':5,'ntrain':300,'ntest':1000,'nedgesperprim':None,
                    'suboptimality':0.1},
                  'synthetic_10d':{'dims':10,'power':2,'noise':0,
                    'nprim':10,'ntrain':1000,'ntest':1000,'nedgesperprim':None,
                    'suboptimality':0.1},
                  'synthetic_10d_noise':{'dims':10,'power':2,'noise':3,
                    'nprim':10,'ntrain':1000,'ntest':1000,'nedgesperprim':None,
                    'suboptimality':0.1},
                  'synthetic_10d_noise_more':{'dims':10,'power':2,'noise':3,
                    'nprim':100,'ntrain':1000,'ntest':1000,'nedgesperprim':None,
                    'suboptimality':0.1},
                  'synthetic_10d_noise_train100_':{'dims':10,'power':2,'noise':3,
                    'nprim':10,'ntrain':100,'ntest':1000,'nedgesperprim':None,
                    'suboptimality':0.1},
                  'synthetic_10d_noise_train10000_':{'dims':10,'power':2,'noise':3,
                    'nprim':10,'ntrain':10000,'ntest':1000,'nedgesperprim':None,
                    'suboptimality':0.1},
                  'synthetic_10d_noise_subsample10_':{'dims':10,'power':2,'noise':3,
                    'nprim':10,'ntrain':1000,'ntest':1000,'nedgesperprim':10,
                    'suboptimality':0.1},
                  'synthetic_10d_noise_subsample50_':{'dims':10,'power':2,'noise':3,
                    'nprim':10,'ntrain':1000,'ntest':1000,'nedgesperprim':50,
                    'suboptimality':0.1},
                  'synthetic_10d_noise_subsample100_':{'dims':10,'power':2,'noise':3,
                    'nprim':10,'ntrain':1000,'ntest':1000,'nedgesperprim':100,
                    'suboptimality':0.1},
                  'synthetic_10d_warp':{'dims':10,'power':5,'noise':0,
                    'nprim':10,'ntrain':1000,'ntest':1000,'nedgesperprim':None,
                    'suboptimality':0.1},
                  'synthetic_2d_noise_self':{'dims':2,'power':2,'noise':1,
                    'nprim':1000,'ntrain':0,'ntest':1000,'nedgesperprim':50,
                    'suboptimality':0.2},
                  'synthetic_2d_noise_self_moreEdges':{'dims':2,'power':2,'noise':1,
                    'nprim':1000,'ntrain':0,'ntest':1000,'nedgesperprim':200,
                    'suboptimality':0.1},
                  'synthetic_2d_noise_self_large':{'dims':2,'power':2,'noise':1,
                    'nprim':10000,'ntrain':0,'ntest':1000,'nedgesperprim':200,
                    'suboptimality':0.1},
                  }

class stats:
    def __init__(self,array):
        self.count = len(array)
        self.min = min(array)
        self.max = max(array)
        self.avg = sum(array)/len(array)
        self.var = sum((v-self.avg)*(v-self.avg) for v in array)/len(array)
        self.stdev = math.sqrt(self.var)

def runSyntheticTests():
    #names = ['synthetic_2d_noise','synthetic_2d_noise_warp','synthetic_10d_noise3',
    #         'synthetic_2d_noise_self','synthetic_2d_noise_self_moreEdges']
    #names = ['synthetic_2d_noise_self','synthetic_2d_noise_self_moreEdges']
    names = ['synthetic_2d_noise_self','synthetic_2d_noise_self_large']
    #names = ['synthetic_10d_noise_train100_','synthetic_10d_noise_train10000_']
    #names = ['synthetic_2d_noise']
    testNew = False
    for name in names:
        problem = syntheticTests[name]
        #create the clusters
        print "Generating data for test "+name+"..."
        dims = problem['dims']
        power = problem['power']
        noise = problem['noise']
        nprim = problem['nprim']
        ntrain = problem['ntrain']
        ntest = problem['ntest']
        suboptimality = problem['suboptimality']
        nedgesperprim = problem['nedgesperprim']
        library,train,test = makeSyntheticData(dims,lambda(x):transformsingle(x,power,noise),nprim,ntrain,ntest,suboptimality,nedgesperprim)

        #do the tests
        description = "Synthetic, dims %d, power %f, noise %f"%(dims,power,noise)
        runTests(train,test,name,description,testNew)

def runSelfTraining(prefix):
    library = readLibraryCSV(prefix+'/data/primitive_features.csv')
    trainClusters = TrainingClusters(TrainingData(library,None))
    with open(prefix+'/data/train_costs.txt','r') as f:
        costs = trainClusters.loadCosts(f)
        trainClusters.assignBest(costs)
    runTrainingRaw(trainClusters,prefix)

if __name__=='__main__':
    #runSyntheticTests()
    #rerunTests('motion_planning_500_500')
    #rerunTests('motion_planning_900_100')
    #rerunTests('motion_planning_9000_1000')
    runSelfTraining('bugtrap')
