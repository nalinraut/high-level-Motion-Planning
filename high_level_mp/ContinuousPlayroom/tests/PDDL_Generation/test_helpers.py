from PDDL_Generation.helpers import *


'''
    This file includes unit tests for the file PDDL_generation
    You could can run these tests by typing `nosetests -v test_helpers.py` in the terminal.
'''


# -------------------------------------------------------------------------
def test_all_subsets():
    result = all_subsets({2, 4, 7})
    assert isinstance(result, list)             # return type test
    for item in [{2}, {4}, {7}, {2, 4}, {2, 7}, {4, 7}, {2, 4, 7}]:
        assert item in result                   # "standard" test
        assert isinstance(item, set)            # data type test

    assert all_subsets({'a'}) == [{'a'}]        # single element test
    assert all_subsets({}) == []                # null test


def test_invert_dict():
    to_try = {'a': {1}, 'b': {1, 2}}
    correct = {1: {'a', 'b'}, 2: {'b'}}
    res = invert_dict(to_try)

    assert isinstance(res, dict)                # type checking

    for key, value in res.iteritems():
        assert correct[key] == value            # functionality checking

    assert invert_dict({}) == {}                # null check

def test_remove_duplicates():
    P = {1: {1,2,3},
         2: {1,2,3},
         3: {2}}
    pfactors = {1: 'hi',
                2: 'bye',
                3: 'yolo'}
    remove_duplicates(P, pfactors)
    assert len(P) == 2
    assert len(pfactors) == 2
