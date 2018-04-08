from operator import itemgetter
import pickle

a = pickle.load(open('filenameasastring.p', 'rb'))

# a = [[[1,2],[1,2],[1,2]], [[3,4],[3, 4]], [[5, 6],[5,6],[5,6]], [[7,8],[7, 8]], [[9,10],[9, 10]], [[11,12],[11, 12]]]
b = [{0, 1}]
new = []
d = a[0]+a[1]
for ind in b:
    # new.append(reduce(lambda x, y: x + y, a[2:4], []))
    if len(ind) > 1:
        new.append(reduce(lambda x, y: x + y, itemgetter(*ind)(a), []))
    else:
        new.append(a[ind.pop()])


print new