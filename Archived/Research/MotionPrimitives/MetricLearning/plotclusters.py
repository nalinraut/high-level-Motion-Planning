import sys
import csv
import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
import numpy as np

def plotclusters(fn,filetype='show'):
    centers = []
    clusters = []
    with open(fn,'r') as csvfile:
        reader = csv.reader(csvfile)
        state = 'centerHeader'
        for row in reader:
                if len(row)==0:
                    #end of cluster
                    state = 'centerHeader'
                elif state == 'centerHeader':
                    state = 'center'
                elif state == 'center':
                    centers.append([float(f)*100 for f in row[1:]])
                    clusters.append([])
                    state = 'clusterHeader'
                elif state == 'clusterHeader':
                    state = 'cluster'
                else:
                    clusters[-1].append([float(f)*100 for f in row[1:]])
        print "read",len(centers),"clusters"

        fig = plt.figure(figsize=(5,5),dpi=150)
        ax1 = fig.add_subplot(111)

        markers = '+xspdh*'
        colors = ['black','red','green','orange','blue','brown','purple','grey','yellow']
        for i,(center,cluster) in enumerate(zip(centers,clusters)):
            ax1.scatter([center[0]], [center[1]], s=12*12, c=colors[i%len(colors)], marker=markers[i%len(markers)])
            circle = plt.Circle(center,0.02*100,color='black',fill=False)
            ax1.add_artist(circle)
            ax1.scatter([v[0] for v in cluster], [v[1] for v in cluster], s=6*6, c=colors[i%len(colors)],marker=markers[i%len(markers)])
        #plt.xlim(-0.1,1)
        #plt.ylim(-0.1,1)
        plt.xlim(-5,105)
        plt.ylim(-5,105)

        if filetype=='show':
            plt.show()
        else:
            pre,ext = fn.rsplit('.',1)
            print "Saving plot to",pre+filetype
            plt.savefig(pre+filetype)
        

if __name__=='__main__':
    import os
    fformat = '.png'
    if os.path.isdir(sys.argv[1]):
        path = sys.argv[1]
        for f in os.listdir(sys.argv[1]):
            if f.endswith('clusters.csv'):
                fn = os.path.join(path,f)
                if os.path.isfile(fn):
                    plotclusters(fn,fformat)
    else:
        plotclusters(sys.argv[1],fformat)
