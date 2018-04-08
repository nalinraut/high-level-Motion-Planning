from data.data_generation.helpers import  *


def test_merge_sets():
    all_clusts = {1, 2, 3, 4, 5, 6, 7, 8, 9}
    merges = [(1, 2), (3, 5), (4, 5), (2, 6), (8, 9)]
    data = merge_sets(all_clusts, merges)
    assert {1, 2, 6} in data        # tests positive merges
    assert {3, 4, 5} in data        # tests positive merges
    assert {8, 9} in data           # test negative merges
    assert {7} in data              # tests non-merged inclusion
    assert len(data) == 4           # tests that there aren't weird extra things
