"""
Concerns:
    [ ] Unknown if this solution will work for problems with non-independent options
    [ ] We did not represent non-compact initiation sets
    [ ] We cheated by making a light-on and light-off option
    [ ] Max did something screwy with effects calculation by ignoring the first truth check. It works but should it?

Next Steps:
    [ ] Do this again, but without hard-coded data/*.json files (learn them)
    [ ] Update playground.py, manual.py to match new understanding
    [ ] Create a new problem to test against that addresses some (all?) of the concerns
    [ ]  Magicially port this onto a physical system
"""

import time
import json
from definitions import *

import PDDL_Generation.generator as pddl_gen

# common data to test
masks = json.load(open(DATA_PATH + "/masks.json"))
initiation_set = json.load(open(DATA_PATH + "/initiation_set.json"))
effect_set = json.load(open(DATA_PATH + "/effect_set.json"))
groundings = json.load(open(DATA_PATH + "/groundings.json"))

P, effects_plus, effects_less, preconditions = pddl_gen.generate(masks, initiation_set, effect_set, groundings)


class Proposition:
    def __init__(self, symb_list, prec_option, parent):
        self.symb_list = symb_list
        self.prec_option = prec_option
        self.parent = parent

    def get_neighbors(self, visited):
        neighbors = []
        for o in preconditions:
            if not preconditions[o].issubset(self.symb_list):
                continue

            new_prop = Proposition((self.symb_list | effects_plus[o]) - effects_less[o], o, self)
            if new_prop.symb_list not in visited:
                visited.add(frozenset(new_prop.symb_list))
                neighbors.append(new_prop)

        return neighbors

def bfs(start, goal):
    start_time = time.time()

    queue = [Proposition(start, None, None)]
    cur = queue.pop()
    visited = {frozenset(cur.symb_list)}

    while not goal.issubset(cur.symb_list):
        queue = queue + cur.get_neighbors(visited)
        cur = queue.pop(0)     # this will break if there is no solution
    res = backtrace(cur)

    end_time = time.time()
    print "visited length: ", len(visited)
    print "Elapsed time: ", end_time - start_time, "seconds"
    print res
    print ""
    return res


def backtrace(cur):
    opt_list = []
    while not cur.prec_option == None:
        opt_list.append(cur.prec_option)
        cur = cur.parent

    return opt_list[::-1]


bfs({'p4', 'p7'}, {'p17'})
bfs({'p4', 'p7'}, {'p11'})
bfs({'p4', 'p7'}, {'p9'})