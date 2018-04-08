import numpy as np

from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

import random
from itertools import compress
from itertools import combinations

from helpers import *


# --------------------------v Data Generation v-------------------------- #
ladder_pos = 5
lever_pos = 2
door_pos = 10
high_thresh = 90
low_thresh = 10

def getNewState(s, o):
    '''
        Evaluate the outcome of an option from a given state
    '''
    x, l = s
    if o == "go_left":
        if x > ladder_pos:
            x = ladder_pos
        elif x <= ladder_pos:
            x = lever_pos
    elif o == "go_right":
        if x >= ladder_pos:
            if l >= high_thresh:
                x = door_pos + 1
            else:
                x = door_pos - 1
        elif x < ladder_pos:
            x = ladder_pos
    elif o == "interact":
        success = random.uniform(0, 1) >= .2
        if success and l >= high_thresh:
            l = 0
        elif success and l <= low_thresh:
            l = 100
        elif not success and l >= high_thresh:
            l = random.uniform(high_thresh, 100)
        elif not success and l <= high_thresh:
            l = random.uniform(0, low_thresh)

    return (x, l)

def getRandomOption(s):
    '''
        given a state, select a random option that can be performed
    '''
    x, l = s

    option_list = []
    if x == lever_pos:
        option_list.append("interact")
    if l > high_thresh and x != door_pos + 1:
        option_list.append("go_right")
    if l < high_thresh and x != door_pos - 1:
        option_list.append("go_right")
    if x != lever_pos:
        option_list.append("go_left")
    return random.choice(option_list)

def generateSampleData():
    '''
        follows paper standards:
            1) create random initiation state
            2) perform 100 random options
            3) record (s, o, s')
            4) repeat 40 times
    '''
    records = []
    for i in range(40):
        x = int(random.uniform(0,8)) + 1                                        # 1..8
        l = random.choice([random.uniform(0, low_thresh), random.uniform(high_thresh, 100)])  # in high thresh or low thresh
        s_old = (x, l)
        for j in range(100):
            o = getRandomOption(s_old)
            s_new = getNewState(s_old, o)

            records.append((s_old, o, s_new))
            s_old = s_new
    return records

# ------------------------v Partitioning Helpers v------------------------ #

def getMask(s_old, s_new):
    '''
    computes the mask between two states (returns integer, mask is the equivalent binary)
    '''
    mask = []
    for i in range(len(s_old)):
        mask.append(int(s_old[i] != s_new[i]))

    to_return = "0b"
    for s_var in mask:
        to_return = to_return + str(s_var)

    return int(to_return, 2)

def partition_masks(option_training_data):
    '''
    given the collected training data for an option, partitions data based on masks
    '''
    partitions = {}       # maps mask (as int) to set of records
    for old, o, new in option_training_data:
        mask = getMask(old, new)
        if mask not in partitions.keys():
            partitions[mask] = []

        partitions[mask].append((old, new))
    return partitions

def int_to_bool_list(num, ss_len):
    return [bool(num & (1<<n)) for n in range(ss_len)][::-1]


def cluster(X):
    X = StandardScaler().fit_transform(X)

    # #############################################################################
    # Compute DBSCAN
    db = DBSCAN(eps=.3, min_samples=10).fit(X)  # .4/14;  5
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    return n_clusters_, np.array(labels)

def merge_partitions(effect_partitions):
    '''
    Identify and merge clusters that share substantially overlapping initiation sets
    :param effect_partitions: [
        [[s, s'], [s, s'], ...];    # partition 1 (by option o; effect cluster 1)
        [[s, s'], [s, s'], ...];    # partition 2 (by option o; effect cluster 2)
        ...                         # partition n (by option o; effect cluster n)
    ]
    :return: final_partitions (data of same type as input)
                              (finally, the data manipulation nightmare is over)
    '''
    merges = []
    all_clusters = range(len(effect_partitions))
    for i, j in combinations(all_clusters, 2):   # run nC2 clustering operations on effect_partitions
        p_i = [p[0] for p in effect_partitions[i]]  # extract only s (not s') from ith element
        p_j = [p[0] for p in effect_partitions[j]]  # extract only s (not s') from jth element

        n_clusters, labels = cluster(p_i + p_j)

        #TODO:  -1 labels might be a problem
        # don't let negatively labeled cases screw everything up
        p_i_in_all = len(set(labels[:len(p_i)]) - {-1}) == n_clusters
        p_j_in_all = len(set(labels[len(p_i):]) - {-1}) == n_clusters
        if(p_i_in_all and p_j_in_all):
            merges.append((i,j))

    #TODO: czech if the above section actually makes the right merges

    to_combine = merge_sets(set(all_clusters), merges)       # to_combine contains sets of clusters to be combined

    new_partitions = []

    for group in to_combine:
        new_partitions.append((effect_partitions[p] for p in group))

    print "merges made"

# -----------------------------v Main Logic v----------------------------- #

data = generateSampleData()

# "partition data points by option"
partitioned_options = {         # maps option name to {mask_int: [data_list]}
    "go_right": partition_masks([record for record in data if record[1] == "go_right"]),
    "go_left": partition_masks([record for record in data if record[1] == "go_left"]),
    "interact": partition_masks([record for record in data if record[1] == "interact"])
}

ss_len = len(data[0][0])

# for each option; for each mask partition; cluster mask-relevant data
for o in partitioned_options:
    effect_partitions = []  # this is the object we're wanting to build
    for mask_partition in partitioned_options[o]: # cluster
        X = [s[1] for s in partitioned_options[o][mask_partition]]      # reduce X from (s, s') to s'

        # TODO:  maybe create data in an already filtered state
        useful_indices = int_to_bool_list(mask_partition, ss_len)
        X = [list(compress(record, useful_indices)) for record in X]    # remove non-masked indices from s'

        n_clusters, labels = cluster(X)
        print o, mask_partition, n_clusters

        for i in range(n_clusters):
            X = np.array(partitioned_options[o][mask_partition])        # re-instantiate X as [(s,s'), ...]
            effect_partitions.append(X[labels == i])

    FOO = merge_partitions(effect_partitions)   # TODO:  do something with this


# #for each option...
# have:   "kinda"
#     m "labels" objects, where m == len(mask_partition)
#     each "labels" has n clusters
#
# want:
#     sum(max(labels_1), max(labels_2), max(labels_3), ...) choose 2 clustering operations
#     some logic on clusters
#
# how:
#     create a list of lists


    # perform (m*n)C
# 2 clustering operations