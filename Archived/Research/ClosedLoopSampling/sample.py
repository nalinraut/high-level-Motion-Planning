import random
import math

def weightedSample(values):
    sumV = sum(values)
    v = random.random()*sumV
    for i in xrange(len(values)):
        if v < values[i]:
            return i
        v -= values[i]
    raise ValueError("Went past end of array for some reason")

def stratifiedAllocate(weights,total):
    """Evenly allocates total samples through the population weighted by the
    weights list"""
    assert(len(weights) > 0)
    cumweight = [0.0]*len(weights)
    cumweight[0] = weights[0]
    for i in xrange(1,len(weights)):
        cumweight[i] = cumweight[i-1]+weights[i]
    scale = total/cumweight[-1]
    psampled = [cw*scale for cw in cumweight]
    psampled[-1] = total

    num = [0]*len(weights)
    bucket = 0
    i = 0
    while i<total:
        f=int(math.floor(psampled[bucket])) 
        if(i < f): #fill in f-i elements into begin this is here so the algorithm  is O(n) where n is the number of buckets
            num[bucket] += (f-i);
            i += (f-i);
        val = i+random.random();
        assert val <= total
        #find the right bucket for this sample
        for j in xrange(bucket,len(num)):
            if val <= psampled[j]:
                #allocate sample i to j, begin at j
                num[j]+=1
                bucket=j
                i+=1
                break
    return num

def stratifiedSample(weights,total):
    num = stratifiedAllocate(weights,total)
    for j,n in enumerate(num):
        for i in xrange(n):
            yield j
    return
