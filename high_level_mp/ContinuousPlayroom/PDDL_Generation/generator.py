import itertools
from helpers import *


def generate(masks, initiation_set, effect_set, groundings):
    """
    This is an implementation of "Algorithm 1: Generate a PDDL Domain Description From Characterizing Sets" found in
    http://irl.cs.brown.edu/pubs/orig_sym_jair.pdf (Konidaris, Kaelbling, Lozano-Perez)

    It converts direct options and their effects to an equivalent PDDL description.

    :param masks:           a dictionary mapping option names to a boolean list of possibly affected state variables
    :param initiation_set:  a dictionary mapping option names to a list of groundings describing the set of states from
                                which the option can be performed
    :param effect_set:      a dictionary mapping option names to a list of groundings describing the set of states
                                which could result from performing said option
    :param groundings:      a dictionary mapping grounding names to grounding tests

    :return: P:             a dictionary mapping generated symbol names to a set of groundings
             effects_plus:  a dictionary mapping option names to a list of symbols to be set to TRUE when an option
                                is performed
             effects_less:  a dictionary mapping option names to a list of symbols to be set to FALSE when an option
                                is performed
             preconditions: a dictionary mapping option names to a list of symbols that must be TRUE before an option
                                can be performed
    """
    # ------------------------Massage Given Data----------------------------------
    # convert masks into more preferable format
    modifies = {}
    for mask in masks:
        str_mask = set()
        for i in range(len(masks[mask])):
            if masks[mask][i]:
                str_mask.add('s' + str(i + 1))
        modifies[mask] = str_mask

    modifies = invert_dict(modifies)

    # ------------------------Compute Factors----------------------------------
    F = {}          # maps f's to {s}
    options = {}    # maps f's to {o}

    # Compute Factors
    for si in modifies:
        new_factor = True
        for fj in F:
            if options[fj] == modifies[si]:
                F[fj] = F[fj] | {si}        # update F mapping
                new_factor = False
        if new_factor:
            fj = 'f' + str(len(F) + 1)
            F[fj] = {si}                    # create new F mapping
            options[fj] = modifies[si]      # create new options mapping

    # ------------------------Generate Symbol Set----------------------------------
    factors = invert_dict(options)          # maps o's to {f}
    inv_F = invert_dict(F)                  # maps s's to {f}
    pfactors = {}                           # maps p's to {f}
    P = {}                                  # maps p's to {g}

    for oi in factors:
        f = factors[oi]
        e = effect_set[oi]

        for fi in f:  # factor out independent factors
            proj_fi = project(e, {fi}, F, groundings)
            proj_not_fi = project(e, f - {fi}, F, groundings)

            if (proj_fi | proj_not_fi) == set(e):   # Perform on independent factors
                pi = 'p' + str(len(P) + 1)
                P[pi] = proj_not_fi
                e = proj_fi
                f = f - {fi}
                pfactors[pi] = {fi}

        for fs in all_subsets(f):                   # Perform on sets of non-independent factors
            pi = 'p' + str(len(P) + 1)
            P[pi] = project(e, fs, F, groundings)
            pfactors[pi] = fs

    remove_duplicates(P, pfactors)  # Prune duplicate and null sets from symbols

    # ------------------------Generate Operator Descriptions----------------------------------
    effects_plus = dict.fromkeys(factors.keys(), set())        # maps oi to {p}
    effects_less = dict.fromkeys(factors.keys(), set())        # maps oi to {p}
    preconditions = dict.fromkeys(factors.keys(), set())       # maps oi to {p}

    for oi in factors:
        # direct effects
        for p in P:
            if P[p].issubset(effect_set[oi]):
                effects_plus[oi] = effects_plus[oi] | {p}

        Pnr = set(P.keys()) - effects_plus[oi]

        # side effects: full and partial overwrites
        for p in Pnr:
            if(#P[p].issubset(initiation_set[oi]) and
               pfactors[p].issubset(factors[oi])):
                effects_less[oi] = effects_less[oi] | {p}

        for p1, p2 in itertools.permutations(Pnr, 2):           # Get all 2-long permutations of Pnr-->O(N^2)
            if(#P[p1].issubset(initiation_set[oi]) and
               len(pfactors[p1] & factors[oi]) is not 0 and
               P[p2] == project(P[p1], factors[oi], F, groundings)):
                effects_plus[oi] = effects_plus[oi] | {p2}
                effects_less[oi] = effects_less[oi] | {p1}

        # compute preconditions
        Ioi = initiation_set[oi]
        relevant_symbols = filter(lambda p:  # Inject p to reducer
                                  reduce((lambda x, y: x or (y in Ioi)), P[p], False),  # Ioi contains some subset of P[p]
                                  P.keys())

        for Pc in all_subsets(relevant_symbols):
            # compute variables for checks
            Ioi_s = {"s" + str(groundings[g]["var_num"]) for g in Ioi}
            Ioi_factors = {list(inv_F[s])[0] for s in Ioi_s}
            g_list = {g for p in Pc for g in P[p]}
            Pc_factors = [f for p in Pc for f in pfactors[p]]

            if(len(Pc_factors) == len(set(Pc_factors)),         # Checks that there are no redundant factors in Pc
               Ioi_factors.issubset(set(Pc_factors)),           # Checks that the initiation set of oi is within Pc
               g_list.issubset(Ioi)):                           # Checks that the groundings of Pc is within init set of oi
                preconditions[oi] = preconditions[oi] | Pc  # TODO: Allow for multiple pc sets of preconditions

    return P, effects_plus, effects_less, preconditions
