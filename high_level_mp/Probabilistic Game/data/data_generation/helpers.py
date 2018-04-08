from itertools import chain, combinations



def merge_sets(all_clusts, merges):
    """
    Condenses a list of sets by combining all sets which can be intersected
    :param all_clusts: The set of all clusters before combining
    :param merges: The list of tuples of each overlapping pair of clusters
    :return: A list of sets of clusters to combine
    """
    def all_subsets(iterable):

        xs = list(iterable)
        # note we return an iterator rather than a list
        tuples = chain.from_iterable(combinations(xs,n) for n in range(len(xs)+1))
        tmp = map(list, tuples)
        tmp.pop(0)   # remove null combination
        return tmp

    good_ones=[]
    for subset in all_subsets(merges)[::-1]:
        subset = [set(i) for i in subset]
        bleh = len(set.intersection(*subset))

        if bleh != 0:
            candidate = set.union(*subset)
            if True not in [candidate.issubset(i) for i in good_ones]:
                good_ones.append(candidate)
                all_clusts = all_clusts - candidate

    for i in all_clusts:
        good_ones.append({i})

    return good_ones