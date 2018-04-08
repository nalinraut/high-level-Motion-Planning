import json
from definitions import *

import PDDL_Generation.generator as pddl_gen

# common data to test
masks = json.load(open(DATA_PATH + "/masks.json"))
initiation_set = json.load(open(DATA_PATH + "/initiation_set.json"))
effect_set = json.load(open(DATA_PATH + "/effect_set.json"))
groundings = json.load(open(DATA_PATH + "/groundings.json"))

P, effects_plus, effects_less, preconditions = pddl_gen.generate(masks, initiation_set, effect_set, groundings)

def test_data_shapes():
    '''
        Symbols:
            15 from effector on object
            5 from [music_on, music_off, light_on, light_off, monkey_screamed]
        PDDL Stuff:
            21 options by default
            -1 option for being worthless
    '''
    for item in (P, effects_less, effects_plus, preconditions):
        assert len(item) == 20
        assert(isinstance(item, dict))

    for p in P:                # should all be strings of pattern pN, where N is an integer
        assert isinstance(p, basestring)
        assert p[0] == "p"

        assert isinstance(P[p], set)
        for g in P[p]:          # should all be strings of pattern gN, where N is an integer
            assert isinstance(g, basestring)
            assert g[0] == "g"

    for pddl_item in (effects_less, effects_plus, preconditions):
        for option, p_set in pddl_item.iteritems():
            assert isinstance(p_set, set)
            for p in p_set:     # should all be strings of pattern gN, where N is an integer
                assert isinstance(p, basestring)
                assert p[0] == "p"


def test_preconditions():
    '''
        tests that the preconditions symbols should match the groundings in the initiation sets
    '''
    for option in initiation_set.keys():
        if "_to_" in option:                        # all movement options should have no preconditions
            assert len(preconditions[option]) == 0
        else:
            if len(effect_set[option]) == 0:        # interact_bell is a naughty boy
                assert option not in preconditions
            else:                                   # preconditions should match initiation_set
                g_list = {g for p in preconditions[option] for g in P[p]}
                assert set(initiation_set[option]) == g_list


def test_effects_plus():
    '''
        tests that the effects_plus symbols should match the groundings in the effect sets
    '''
    for option in effect_set.keys():
        if len(effect_set[option]) == 0:  # interact_bell is a naughty boy
            assert option not in effects_plus
        else:  # preconditions should match effect_set
            g_list = {g for p in effects_plus[option] for g in P[p]}
            assert set(effect_set[option]) == g_list


def test_effects_less():
    '''
        tests that effects_less contains what we conceptually understand to be negated by an option
    '''
    # evaluate effector movements
    eye_less = [effects_less[option] for option in filter(lambda option: "eye_to_" in option, effect_set.keys())]
    hand_less = [effects_less[option] for option in filter(lambda option: "hand_to_" in option, effect_set.keys())]
    marker_less = [effects_less[option] for option in filter(lambda option: "marker_to_" in option, effect_set.keys())]


    for effector in (eye_less, hand_less, marker_less):
        for ef_set1 in effector:
            assert len(ef_set1) == 4                # each motion should set symbols for non-moved-to objects to False
            leftover = set()
            for ef_set2 in effector:
                if ef_set1 == ef_set2:              # ignore the diagonal
                    continue
                assert len(ef_set1 & ef_set2) == 3  # shares 3 overlaps with all other same-effector symbol
                leftover = leftover | (ef_set2 - (ef_set1 & ef_set2))
            assert len(leftover) == 1               # the symbol for moving to effect set1 is remainder in all other sets

    hardcoded = {"interact_ball": set(),
                 "interact_green_button": {"g34"},
                 "interact_light_off": {"g32"},
                 "interact_light_on": {"g35"},
                 "interact_red_button": {"g31"}}

    # evaluate object interactions
    for option in effect_set.keys():
        if option in hardcoded:
            g_list = {g for p in effects_less[option] for g in P[p]}
            tmp = hardcoded[option]
            assert g_list == hardcoded[option]
        if len(effect_set[option]) == 0:  # interact_bell is a naughty boy
            assert option not in effects_plus
