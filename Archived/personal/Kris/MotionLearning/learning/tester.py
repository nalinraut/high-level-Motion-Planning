from decisiontree_missing import DecisionTree
from randomforest_missing import RandomForest
from nn_missing import NearestNeighbor
from database import Database
from asizeof import asizeof
import multiprocessing 
import multiprocessing.pool
from contextlib import closing
from collections import defaultdict
from randomforest_missing import parallel_execute
import numpy as np
from utils import mkdir_p

class NoDaemonProcess(multiprocessing.Process):
    # make 'daemon' attribute always return False
    def _get_daemon(self):
        return False
    def _set_daemon(self, value):
        pass
    daemon = property(_get_daemon, _set_daemon)

# We sub-class multiprocessing.pool.Pool instead of multiprocessing.Pool
# because the latter is only a wrapper function, not a proper class.
class Pool(multiprocessing.pool.Pool):
    Process = NoDaemonProcess

global _sampling_functions, _default_dataset, _datasets, _methods_to_test, _scenarios_to_test, _problem_name, _output_file, _default_output_file
_sampling_functions = {}
_datasets = {}
_default_dataset = None
_methods_to_test = []
_scenarios_to_test = []
_problem_name = None
_output_file = None
_default_output_file = 'trials.json'
_data_folder = ""

def sparse_features(problem):
	f = problem.features()
	r = problem.relevance()
	assert len(f)==len(r)
	res = {}
	for i,(a,b) in enumerate(zip(f,r)):
		if b:
			res[i] = a
	return res

def test_dt(traindb,trainlabels,test,testlabels,maxnodes=None):
	assert len(traindb.entries) == len(trainlabels)
	assert len(test) == len(testlabels)
	tree = DecisionTree()
	tree.maxnodes = maxnodes
	errors = tree.learn(traindb,trainlabels)

	print "Decision tree makes",errors,"errors"
	print "Depth",tree.depth(),"nodes",tree.numNodes()
	if tree.numNodes() < 100:
	    tree.pprint()
	temp = tree.pack()
	if errors > 0 and errors < 10:
	    print "Training errors:"
	    for id,(x,y) in enumerate(zip(traindb.entries,trainlabels)):
	        res = tree.predict(x)
	        if res != y:
	            if len(x) > 10:
	                print "  Error on",id,"prediction",res
	            else:
	                print "  Error on",x,"prediction",res

	tp,tn,fp,fn = 0,0,0,0
	for x,y in zip(test,testlabels):
	    res = tree.predict(x)
	    if res and y:
	        tp += 1
	    elif res and not y:
	        fp += 1
	    elif not res and y:
	        fn += 1
	    else:
	        tn += 1
	Ntest = len(test)
	tp = float(tp)/Ntest
	tn = float(tn)/Ntest
	fp = float(fp)/Ntest
	fn = float(fn)/Ntest
	res = {'accuracy':tp+tn,'tp':tp,'tn':tn,'fp':fp,'fn':fn}
	if maxnodes:
		res['maxnodes'] = maxnodes
	return res

def test_nn(traindb,trainlabels,test,testlabels,k=1):
	assert len(traindb.entries) == len(trainlabels)
	assert len(test) == len(testlabels)
	nn = NearestNeighbor(k)
	nn.learn(traindb,trainlabels)

	print "***%d-nearest neighbor***"%(k,)
	
	#if len(traindb.entries) <= 1000:
	#	print "L.o.o error:",1.0-nn.loo_accuracy()

	tp,tn,fp,fn = 0,0,0,0
	for x,y in zip(test,testlabels):
	    res = nn.predict(x)
	    if res and y:
	        tp += 1
	    elif res and not y:
	        fp += 1
	    elif not res and y:
	        fn += 1
	    else:
	        tn += 1
	Ntest = len(test)
	tp = float(tp)/Ntest
	tn = float(tn)/Ntest
	fp = float(fp)/Ntest
	fn = float(fn)/Ntest
	res = {'accuracy':tp+tn,'tp':tp,'tn':tn,'fp':fp,'fn':fn}
	res['k'] = k
	return res

def test_rf(traindb,trainlabels,test,testlabels,Ntrees=50):
	assert len(traindb.entries) == len(trainlabels)
	assert len(test) == len(testlabels)
	tree = RandomForest(Ntrees)
	tree.learn(traindb,trainlabels)

	print "***Random forest***"
	print "Average tree depth:",float(sum(t.depth() for t in tree.trees))/len(tree.trees)
	print "Average tree size:",float(sum(t.numNodes() for t in tree.trees))/len(tree.trees)
	"""
	print "Training error:"
	tp,tn,fp,fn = 0,0,0,0
	for x,y in zip(traindb.entries,trainlabels):
	    res = tree.predict(x)
	    if res and y:
	        tp += 1
	    elif res and not y:
	        fp += 1
	    elif not res and y:
	        fn += 1
	    else:
	        tn += 1
	Ntest = len(test)
	print "True +: %g, True -: %g"%(float(tp)/Ntest,float(tn)/Ntest)        
	print "False -: %g, False +: %g"%(float(fn)/Ntest,float(fp)/Ntest)
	print "Overall error: %g"%(float(fn+fp)/Ntest,)
	print
	"""

	tp,tn,fp,fn = 0,0,0,0
	for x,y in zip(test,testlabels):
	    res = tree.predict(x)
	    if res and y:
	        tp += 1
	    elif res and not y:
	        fp += 1
	    elif not res and y:
	        fn += 1
	    else:
	        tn += 1
	Ntest = len(test)
	tp = float(tp)/Ntest
	tn = float(tn)/Ntest
	fp = float(fp)/Ntest
	fn = float(fn)/Ntest
	res = {'accuracy':tp+tn,'tp':tp,'tn':tn,'fp':fp,'fn':fn}
	res['Ntrees'] = Ntrees
	return res


def test_idt(traindb,trainlabels,test,testlabels,maxnodes=None):
	return test_dt(traindb,trainlabels,test,testlabels,maxnodes)

def test_iedt(traindb,trainlabels,test,testlabels,maxnodes=None):
	#for each feature, create two features: one for 
	exptrain = Database()
	exptrain.keys = sum([[k,'1-'+k] for k in traindb.keys],[])
	exptrain.entries = []
	for e in traindb.entries:
		exp = []
		for i,k in enumerate(traindb.keys):
			if i in e:
				exp.append(e[i])
				exp.append(1-e[i])
			else:
				exp.append(0)
				exp.append(0)
		assert len(exp) == len(exptrain.keys)
		exptrain.entries.append(exp)
	exptest = []
	for x in test:
		exptest.append(sum([[v,1.0-v] for v in x],[]))
	return test_dt(exptrain,trainlabels,exptest,testlabels,maxnodes)
	#return test_rf(exptrain,trainlabels,exptest,testlabels,10)


def test_irf(traindb,trainlabels,test,testlabels,Ntrees=50):
	return test_rf(traindb,trainlabels,test,testlabels,Ntrees)

def test(method,*args,**kwargs):
	#load specified dataset
	global _default_dataset
	training,testing,Ntrain,Ntest = args[:4]
	if training is None:
		training = _default_dataset
	if testing is None:
		testing = _default_dataset
	print "Testing on",training,testing,Ntrain,Ntest
	dbtrain = Database()
	dbtrain.readCSV(_datasets[training][0],Ntrain)
	dbtrain.cast()
	trainlabels = [int(e[-1]) for e in dbtrain.entries]
	dbtrain.keys.pop(-1)
	dbtrain.entries = [np.array(e[:-1],dtype=float) for e in dbtrain.entries]
	dbtest = Database()
	dbtest.readCSV(_datasets[testing][1],Ntest)
	dbtest.cast()
	testing = [np.array(e[:-1]) for e in dbtest.entries]
	testlabels = [int(e[-1]) for e in dbtest.entries]
	del dbtest
	if 'irrelevance' in method:
		#form sparse database
		dbrel = Database()
		dbrel.readCSV(_datasets[training][2],Ntrain,storage='numpy',dtype=int)
		for row,(e,r) in enumerate(zip(dbtrain.entries,dbrel.entries)):
			assert len(e) == len(r),"feature / relevance lengths %d and %d do not match"%(len(e),len(r))
			rowdict = {}
			for i,(v,rv) in enumerate(zip(e,r)):
				if rv!=0:
					rowdict[i] = v
			dbtrain.entries[row] = rowdict
	if method == 'decision tree':
		return test_dt(dbtrain,trainlabels,testing,testlabels,*args[4:],**kwargs)
	elif method == 'random forest':
		return test_rf(dbtrain,trainlabels,testing,testlabels,*args[4:],**kwargs)
	elif method == 'irrelevance decision tree':
		return test_idt(dbtrain,trainlabels,testing,testlabels,*args[4:],**kwargs)
	elif method == 'irrelevance expanded decision tree':
		return test_iedt(dbtrain,trainlabels,testing,testlabels,*args[4:],**kwargs)
	elif method == 'irrelevance random forest':
		return test_irf(dbtrain,trainlabels,testing,testlabels,*args[4:],**kwargs)
	elif method == 'nearest neighbor':
		return test_nn(dbtrain,trainlabels,testing,testlabels,*args[4:],**kwargs)
	elif method == 'irrelevance nearest neighbor':
		return test_nn(dbtrain,trainlabels,testing,testlabels,*args[4:],**kwargs)
	raise ValueError("Invalid method "+method)

def _log_trial(trial,fn=None):
	global _problem_name,_output_file,_default_output_file
	import json
	if fn == None:
		if _output_file != None:
			fn = _output_file
		elif _problem_name != None:
			fn = _problem_name+".json"
		else:
			fn = _default_output_file
	print "Logging trial",trial,"to",fn
	f = open(fn,'a')
	special = ['type','accuracy','tp','tn','fp','fn']
	print "%s(%s):"%(trial['method'],','.join(str(k)+'='+str(v) for k,v in trial.iteritems() if k not in special))
	print "  overall: %f, tp: %f, tn: %f fp: %f, fn: %f"%(trial['accuracy'],trial['tp'],trial['tn'],trial['fp'],trial['fn'])
	f.write(json.dumps(trial))
	f.write('\n')
	f.close()

def _generate_file(distribution,N,outFile,relevanceFile=None):
	global _sampling_functions
	genfunc = _sampling_functions[distribution]
	dataset = [genfunc() for i in range(N)]
	
	print "Generating dataset",outFile
	print "N =",N
	db = Database()
	if hasattr(dataset[0],'featureNames'):
		db.keys = dataset[0].featureNames()
	else:
		for i,f in enumerate(dataset[0].features()):
			db.keys.append("f"+str(i))
	db.keys.append("label")
	for x in dataset:
		db.entries.append(x.features()+[x.concept()])
	db.writeCSV(outFile)
	print "Nfeatures =",len(db.entries[0])-1
	print "Fraction of positive examples =",float(sum(f[-1] for f in db.entries))/len(db.entries)
	print "Size of training set (mb)",float(asizeof(db.entries))/1024/1024
	print "Size of training example (bytes)",asizeof(db.entries[0])

	if relevanceFile != None:
		dbr = Database()
		dbr.keys = [v+"_relevance" for v in db.keys[:-1]]
		for x in dataset:
			dbr.entries.append([int(v) for v in x.relevance()])

		nrel = 0
		ntotal = 0
		for e in dbr.entries:
			nrel += sum(e)
			ntotal += len(e)
		print "Fraction of relevant features:",float(nrel)/float(ntotal)
		dbr.writeCSV(relevanceFile)

def _generate_file2(args):
	return _generate_file(*args)

def _make_tests(lock,Ntrain,Ntest,training,testing,**kwargs):
	global _methods_to_test
	global _problem_name

	extraInfo = {}
	extraInfo['training'] = training
	extraInfo['testing'] = testing
	if Ntrain is not None:
		extraInfo['Ntrain'] = Ntrain
	if Ntest is not None:
		extraInfo['Ntest'] = Ntest
	extraInfo['Ntrain'] = Ntrain
	if _problem_name is not None:
		extraInfo['problem'] = _problem_name
	if len(kwargs) > 0:
		extraInfo['args'] = kwargs
	tests = []
	for m,a,ka in _methods_to_test:
		tests.append((lock,m,(training,testing,Ntrain,Ntest)+a,ka,extraInfo))
	return tests

def _do_test(args):
	lock,method,args,kwargs,extraInfo = args

	try:
		print "Beginning trial",method
		res = test(method,*args,**kwargs)
	except KeyboardInterrupt:
		print "Test",method,"interrupted by Ctrl+C"
		raise
	res['method'] = method
	res.update(extraInfo)
	lock.acquire()
	_log_trial(res)
	lock.release()

def set_problem(name):
	global _problem_name
	_problem_name = name

def set_output_file(fn):
	global _output_file
	_output_file = fn

def set_data_folder(fn):
	global _data_folder
	_data_folder = fn
	if len(fn) > 0 and fn[-1] != '/':
		mkdir_p(_data_folder)
		_data_folder += "/"

def add_sampler(name,func):
	global _default_dataset
	global _sampling_functions
	if _default_dataset == None:
		_default_dataset = name
	_sampling_functions[name] = func

def add_dataset(name,trainFile,testFile,relevanceFile=None):
	global _default_dataset
	global _datasets
	global _data_folder
	if _default_dataset == None:
		_default_dataset = name
	if trainFile != None:
		trainFile = _data_folder + trainFile
	if testFile != None:
		testFile = _data_folder + testFile
	if relevanceFile != None:
		relevanceFile = _data_folder + relevanceFile
	_datasets[name] = (trainFile,testFile,relevanceFile)

def add_method(name,*args,**kwargs):
	global _methods_to_test
	_methods_to_test.append((name,args,kwargs))

def add_scenario(Ntrain=None,Ntest=None,training=None,testing=None,**kwargs):
	"""Either Ntrain, Ntest must be specified (then the default sampler is used)
	or training / testing must be specified (then the given sampler / dataset is used).
	"""
	global _scenarios_to_test
	_scenarios_to_test.append(((Ntrain,Ntest,training,testing),kwargs))

def run(numThreads=1):
	global _scenarios_to_test,_datasets,_sampling_functions,_default_dataset,_data_folder
	gen_files = defaultdict(list)
	for s in _scenarios_to_test:
		(Ntrain,Ntest,training,testing) = s[0]
		if training == None:
		 	training = _default_dataset
		if testing == None:
		 	testing = _default_dataset
		if training not in _datasets:
			assert training in _sampling_functions,"Invalid sampler or dataset specified: "+str(training)
			assert Ntrain != None,"Need to specify Ntrain for sampling dataset"
			gen_files[training].append((Ntrain,0))

		if testing not in _datasets:
			assert testing in _sampling_functions,"Invalid sampler or dataset specified: "+str(testing)
			assert Ntest != None,"Need to specify Ntest for sampling dataset"
			gen_files[testing].append((0,Ntest))
	genargs = []
	for dist,tests in gen_files.iteritems():
		trainmax = max(Ntrain for (Ntrain,Ntest) in tests)
		testmax = max(Ntest for (Ntrain,Ntest) in tests)
		trainFile = _data_folder+'train_'+dist+'_'+str(trainmax)+'.csv'
		testFile = _data_folder+'test_'+dist+'_'+str(testmax)+'.csv'
		relFile = _data_folder+'train_relevance_'+dist+'_'+str(trainmax)+'.csv'
		if trainmax>0:
			genargs.append((dist,trainmax,trainFile,relFile))
		if testmax>0:
			genargs.append((dist,testmax,testFile,None))
		_datasets[dist] = (trainFile,testFile,relFile)
	if len(genargs) > 0:
		parallel_execute(_generate_file2,genargs,numThreads)

	manager = multiprocessing.Manager()
	lock = manager.Lock()
	if numThreads == 1:
		for args,kwargs in _scenarios_to_test:
			tests = _make_tests(lock,*args,**kwargs)
			for t in tests:
				_do_test(t)
	else:
		tests = []
		for args,kwargs in _scenarios_to_test:
			tests += _make_tests(lock,*args,**kwargs)
		parallel_execute(_do_test,tests,numThreads)
