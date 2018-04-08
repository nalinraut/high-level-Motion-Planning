import sys
import primitivelib
from sklearn import linear_model
from sklearn import cross_validation
from sklearn.decomposition import PCA
from sklearn.covariance import EllipticEnvelope
import numpy as np
import csv

def mean_sq_score_func(y0,y1):
    return np.mean((y0-y1)**2)

def process_data(cols):
    labels = [c[0] for c in cols]
    labelind = dict((l,i) for (i,l) in enumerate(labels))
    counterparts = []
    for l in labels:
        if l[0]!='s': continue
        for m in labels:
            if m == 'd'+l[1:] and len(l[1:])>0:
                counterparts.append((labelind[l],labelind[m]))
                break
    for c in counterparts:
        i,j = c[0],c[1]
        col = ['d2'+labels[i][1:]]
        for di,dj in zip(cols[i],cols[j])[1:]:
            col.append((di-dj)**2)
        cols.append(col)
    return

if __name__=='__main__':
    drop = ['s','d','s[name]','d[name]','s[dt]','d[dt]','s[n]','d[n]','scratch_cost']
    targets = ['adapt_cost','adapt_iters','subopt_score']
    
    f = open(sys.argv[1],'r')
    reader = csv.reader(f)
    labels = None
    inputs = []
    outputs = []
    for i,line in enumerate(reader):
        if i ==0:
            labels = line[:]
        else:
            inputs.append(line)
 
    cols = [v for v in zip(labels,*inputs) if v[0] not in drop]
    for j,v in enumerate(cols):
        cols[j] = [v[0]] + [float(vi) for vi in v[1:]]
    process_data(cols)
    labels = [v[0] for v in cols if v[0] not in targets]
    outlabels = [v[0] for v in cols if v[0] in targets]
    print "# observations:",len(inputs)
    print "# variables:",len(labels)
    print "Input labels:",' '.join(labels)
    print "Output labels:",' '.join(outlabels)
    if len(outlabels) != len(targets):
        print "Warning, dataset is missing some targets"

    #perform PCA to detect help outliers
    pca = PCA(n_components=4,whiten=False)
    alldata = np.array(zip(*[v[1:] for v in cols]))
    alldata_pcs = pca.fit(alldata).transform(alldata)
    # Components
    print pca.components_
    # Percentage of variance explained for each components
    print 'explained variance ratio (first %d components):'%(len(pca.components_),), \
          pca.explained_variance_ratio_
    print 'total explained variance:',sum(pca.explained_variance_ratio_)

    outlier_classifier = EllipticEnvelope(contamination=.05)
    outlier_classifier.fit(alldata_pcs)
    inlier_classification = outlier_classifier.predict(alldata_pcs)
    #print "Outliers:",[i for (i,c) in enumerate(inlier_classification) if c<0]

    inputs = zip(*[v[1:] for v in cols if v[0] not in targets])
    outputs = zip(*[v[1:] for v in cols if v[0] in targets])
    #reject outliers
    inputs = [inputs[i] for (i,c) in enumerate(inlier_classification) if c>0]
    outputs = [outputs[i] for (i,c) in enumerate(inlier_classification) if c>0]
    print len(inputs),"inliers"
    #select output 0 (adapt_cost)
    #output = np.array([v[0] for v in outputs])
    #select output 2 (subopt_score)

    for ind in xrange(len(outputs[0])):
        print "Learning output",outlabels[ind]
        output = np.array([v[ind] for v in outputs])

        #perform linear regression
        #reg = linear_model.LinearRegression()
        reg = linear_model.Lasso(alpha=0.005,normalize=False)
        scores = cross_validation.cross_val_score(reg, inputs, output, cv=5,score_func = mean_sq_score_func)
        print "Cross-validated scores:",scores
        print "Mean: %0.2f, std: %0.2f"%(scores.mean(),scores.std())
        reg.fit(inputs,output)
        print "OLS coef:",reg.coef_,", intercept",reg.intercept_
        # The mean square error
        print ("Residual sum of squares: %.2f" %
               np.mean((reg.predict(inputs) - output) ** 2))
        # Explained variance score: 1 is perfect prediction
        print ('Variance score: %.2f' % reg.score(inputs, output))
        print
    
