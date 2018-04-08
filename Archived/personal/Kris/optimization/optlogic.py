import time

class Context:
    def __init__(self,name):
        self.name = name

context_map = dict()

def load_context(contextID):
    global context_map
    if contextID in context_map:
        return context_map[contextID]
    else:
        context = Context(contextID)
        context_map[contextID]=context
        context.load()
        return context

def save_all():
    global context_map
    for id,context in context_map.iteritems():
        context.save()

def save_context(contextID):
    global context_map
    if contextID not in context_map:
        raise ValueError("ContextID not initialized")
    context_map[contextID].save()

def clear_context(contextID):
    global context_map
    context = load_context(contextID)
    context.clear()
    context.save()

def wrap(f,contextID):
    context = load_context(contextID)
    def wrapped_f():
        t0 = time.time()
        res = f()
        t1 = time.time()
        context.data[f.__name__].time.collect(t1-t0)
        context.data[f.__name__].res.collect(res)
        return res
    return wrapped_f

class OptTester:
    """Tests a set of tests f1(x),...,fn(x) for equality to zero.
    Maintains statistics about evaluation time, success rate,
    max/average deviation from zero.  The stats are then used to determine
    the optimal testing order.
    """
    def __init__(self,*fs):
        self.tests = [wrapper(f) for f in fs]
        self.reset_history(self.tests[-1])
        self.allorder = self.anyorder = self.tests

    def add_test(self,f):
        self.tests.append(wrapper(f))
        self.reset_history(self.tests[-1])
        self.allorder = self.anyorder = self.tests

    def update_orders(self):
        thelist = [(f._sum_cost/f._num_fail,f) for f in self.tests]
        self.allorder = [t for (ec,t) in sorted(thelist)]
        thelist = [(f._sum_cost/f._num_pass,f) for f in self.tests]
        self.anyorder = [t for (ec,t) in sorted(thelist)]

    def all(self,*args):
        """Tests whether *args passes all tests.  Updates the stats and
        internal order"""
        for f in self.anyorder:
            t1 = time.time()
            res = f(*args)
            t2 = time.time()
            self.update_stats(f,t2-t1,res)
            if not res:
                self.update_order()
                return False
        self.update_order()
        return True

    def any(self,*args):
        """Tests whether *args passes any test.  Updates the stats and
        internal order"""
        for f in self.allorder:
            t1 = time.time()
            res = f(*args)
            t2 = time.time()
            self.update_stats(f,t2-t1,res)
            if res != 0:
                self.update_order()
                return True
        self.update_order()
        return True


    def all_expectation(self):
        """Returns (expected cost, expected success) of testing all tests
        in the current order."""
        c = 0.0
        p = 1.0
        for f in self.allorder:
            avgcost = f._sum_cost/(f._num_pass+f._num_fail)
            failrate = float(f._num_fail)/(f._num_pass+f._num_fail)
            c += p*avgcost
            p *= failrate
        return (c,p)

    def any_expectation(self):
        """Returns (expected cost, expected success) of testing any tests
        in the current order."""
        c = 0.0
        p = 0.0
        for f in self.anyorder:
            avgcost = f._sum_cost/(f._num_pass+f._num_fail)
            passrate = float(f._num_pass)/(f._num_pass+f._num_fail)
            c += (1-p)*avgcost
            p += (1-p)*passrate
        return (c,p)

    def reset_history(self,f,avg_cost=1.0,pr_pass=0.5,evidence=2.0):
        f._sum_cost = avg_cost*evidence
        f._num_pass = pr_pass*evidence
        f._num_fail = (1.0-pr_pass)*evidence
        f._max_dist = 0.
        f._sum_dist = 0.

    def update_stats(self,f,cost,res):
        f._sum_cost += cost
        if res==0: f._num_pass += 1
        else:
            f._num_fail += 1
            f._max_dist = max(f._max_dist,abs(res))
            f._sum_dist += abs(res)

    def stats(self,f):
        """Returns a dictionary describing the statistics of f"""
        res = dict()
        res['average cost']=f._sum_cost/(f._num_pass+f._num_fail)
        res['pass rate']=float(f._num_pass)/(f._num_pass+f._num_fail)
        res['evaluations']=f._num_pass+f._num_fail
        res['max distance']=f._max_dist
        res['average distance']=float(f._sum_dist)/(f._num_pass+f._num_fail)
        return res

    def init_stats(self,f,d):
        """Given a dictionary returned by a stats() call, fills in the
        appropriate statistics of f"""
        f._sum_cost = d['average cost']*d['evaluations']
        f._num_pass = d['evaluations']*d['pass rate']
        f._num_fail = d['evaluations']*(1.0-d['pass rate'])
        f._max_dist = d['max distance']
        f._sum_dist = d['average distance']*d['evaluations']

