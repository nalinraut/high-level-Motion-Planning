import random

trainFraction = 0.9
epsilon = 0.1

primitives = []
problems = []
successRates = dict()
with open('data/primitive_features.csv') as f:
    for line in f.readlines():
        primitives.append(line)
with open('data/train_features.csv') as f:
    for line in f.readlines():
        problems.append(line)
with open('data/train_rates.txt') as f:
    for line in f.readlines():
        i,j,r = line.split()
        successRates[(int(i),int(j))] = float(r)
        assert(int(i) >= 0 and int(i) < len(problems))

ntrain = int(trainFraction*len(problems))
ntest = len(problems) - ntrain
print ntrain,"Training points",ntest,"Testing points"
trainIndices = random.sample(range(len(problems)),ntrain)
trainSet = dict((v,i) for (i,v) in enumerate(trainIndices))
testIndices = [i for i in range(len(problems)) if i not in trainSet]        
testSet = dict((v,i) for (i,v) in enumerate(testIndices))

training = []
trainCosts = dict()
for i in trainIndices:
    training.append(problems[i])
    for (i,j),r in successRates.iteritems():
        if j in trainSet:
            trainCosts[(i,trainSet[j])] = 1-r
testing = []
testCosts = dict()
for i in testIndices:
    testing.append(problems[i])
    for (i,j),r in successRates.iteritems():
        if j in testSet:
            testCosts[(i,testSet[j])] = 1-r

with open('train_features.csv','w') as f:
    for p in training:
        f.write(p)
with open('test_features.csv','w') as f:
    for p in testing:
        f.write(p)
with open('train_costs.txt','w') as f:
    for (i,j),c in sorted(trainCosts.iteritems()):
        f.write(str(i)+" "+str(j)+" "+str(c)+"\n")
with open('test_costs.txt','w') as f:
    for (i,j),c in sorted(testCosts.iteritems()):
        f.write(str(i)+" "+str(j)+" "+str(c)+"\n")
#find the top primitives for each problem
trainByProblem = [dict() for i in training]
testByProblem = [dict() for i in testing]
for (i,j),c in trainCosts.iteritems():
    trainByProblem[j][i]=c
for (i,j),c in testCosts.iteritems():
    testByProblem[j][i]=c
trainClose = [[] for i in primitives]
testClose = [[] for i in primitives]
for j,primDict in enumerate(trainByProblem):
    if len(primDict)==0: continue
    primitives = sorted((v,i) for (i,v) in primDict.iteritems())
    cmin = primitives[0][0]
    cmax = primitives[-1][0]
    cthresh = cmin + epsilon*(cmax-cmin)
    for v,i in primitives:
        if v > cthresh: break
        trainClose[i].append(j)
for j,primDict in enumerate(testByProblem):
    if len(primDict)==0: continue
    primitives = sorted((v,i) for (i,v) in primDict.iteritems())
    cmin = primitives[0][0]
    cmax = primitives[-1][0]
    cthresh = cmin + epsilon*(cmax-cmin)
    for v,i in primitives:
        if v > cthresh: break
        testClose[i].append(j)
with open('train_close.txt','w') as f:
    for c in trainClose:
        f.write(' '.join(str(v) for v in c)+"\n")
with open('test_close.txt','w') as f:
    for c in testClose:
        f.write(' '.join(str(v) for v in c)+"\n")
