from itertools import chain, combinations

def invert_dict(to_invert):
    """
    inverts a dictionary mapping a scalar to a set by performing join operations on all value sets
    :param to_invert: a dictionary mapping scalars to sets
    :return: an dictionary mapping scalars to sets
    """
    to_return = {}
    for key in to_invert:
        for item in to_invert[key]:
            if item in to_return:
                to_return[item].add(key)
            else:
                to_return[item] = {key}
    return to_return

def all_subsets(iterable):
    """
    all_subsets([1,2,3]) --> (1,) (2,) (3,) (1,2) (1,3) (2,3)
    """
    xs = list(iterable)
    # note we return an iterator rather than a list
    tuples = chain.from_iterable(combinations(xs,n) for n in range(len(xs)+1))
    tmp = map(set, tuples)
    tmp.pop(0)   # remove null combination
    # tmp.pop(-1)  # remove full combination
    return tmp

def project(effect, factors, F, groundings):
    """
    returns all groundings of effect NOT containing any state variables in any factor in factors
    :param effect: the effect to "remove" groundings from
    :param factors: the factors that returned groundings cannot be associated with
    :param F: the mapping of factors to state variables
    :param groundings: the details of each grounding
    :return:
    """
    states = []
    for f in factors:
        states += F[f]

    to_return = [g for g in effect
                 if 's' + str(groundings[g]['var_num']) not in states]
    return set(to_return)


def remove_duplicates(P, pfactors):
    """
    Removes dictionary elements of P which contain duplicate values
    :param P: Dictionary to prune
    :return: None
    :modifies: Mutates given dictionary
    """
    set_holder = set()
    prune_me = []
    for pi in P:
        possible_dup = frozenset(P[pi])

        if possible_dup in set_holder or len(possible_dup) == 0:
            prune_me.append(pi)
        else:
            set_holder.add(possible_dup)

    for to_prune in prune_me:
        del P[to_prune]
        del pfactors[to_prune]