from klampt import *
from klampt.model import subrobot
from klampt.io import resource
from klampt.math import *
from klampt import vis
import hand_models
import polytope
import stability
import numpy as np
import time
import cvxopt
import math

#robot_file = '/home/motion/rockclimber/platform/caesar/models/robosimian_caesar_new.urdf'
robot_file = 'robots/robosimian_with_spinyHand_4.rob'
hand_types = {'fr_hand:Link 0':'spiny_hand',
        'br_hand:Link 0':'spiny_hand',
        'bl_hand:Link 0':'spiny_hand',
        #'fl_hand:Link 0':'spiny_hand'
        }
hands = {}
hand_configs = {'fr_hand:Link 0':'grasp',
        'br_hand:Link 0':'unified_palm',
        'bl_hand:Link 0':'unified_palm',
        #'fl_hand:Link 0':'opposing_palm'
        }
world = None
robot = None
hand_subrobots = {}

hand_robot_names = {'spiny_hand':'floating_hand_default',
                    'spiny_opposing':'floating_opposing'}

hand_wrench_spaces = {}

def load_robot():
    global world,robot_file,robot,hand_subrobots,hands
    world = WorldModel()
    world.readFile(robot_file)
    robot = world.robot(0)
    children = [[] for i in range(robot.numLinks())]
    for i in range(robot.numLinks()):
        p = robot.link(i).getParent()
        if p >= 0:
            children[p].append(i)
    for link,hand in hand_types.iteritems():
        hand_links = [robot.link(link).index]
        q = [robot.link(link).index]
        while len(q) > 0:
            l = q.pop()
            hand_links += children[l]
            q += children[l]
        hand_subrobots[link] = subrobot.SubRobotModel(robot,hand_links)
        hands[link] = hand_models.hands[hand]
        hands[link].load_frames()

def load_hand_configs():
    global hand_types,hand_subrobots,hand_configs
    for link,hand in hand_types.iteritems():
        if hand_configs[link] is None:
            continue
        resource.setDirectory(hands[link].resource_directory)
        qhand = resource.get(hand_configs[link]+".config",type="Config",doedit=False)
        #take out the 6 floating DOFs
        hand_subrobots[link].setConfig(qhand[6:])

def load_wrench_spaces():
    spaces = {}
    for link,hand in hand_types.iteritems():
        savedir = "data/"+hand_robot_names[hand]+'/'+hand_configs[link]
        if savedir in spaces:
            W = spaces[savedir]
        else:
            W = polytope.GeneralizedWrenchPolytope()
            W.loadForceSamples(savedir)
            spaces[savedir] = W
        hand_wrench_spaces[link] = W

def edit_config():
    global world,robot
    resource.setDirectory("resources/")
    qhands = [r.getConfig() for r in hand_subrobots.values()]
    q = resource.get("robosimian_body_stability.config",default=robot.getConfig(),doedit=False)
    robot.setConfig(q)
    for r,qhand in zip(hand_subrobots.values(),qhands):
        r.setConfig(qhand)
    vis.add("world",world)
    vis.edit(("world",robot.getName()),True)
    vis.dialog()
    vis.edit(("world",robot.getName()),False)
    resource.set("robosimian_body_stability.config",robot.getConfig())

class WrenchSpaceIndices:
    def __init__(self,links,indices,mode='decomposition'):
        assert len(links) == len(indices)
        self.links = links
        self.indices = indices
        self.mode = mode
    def polytopes(self,hand_wrench_spaces):
        if self.mode == 'decomposition':
            return [hand_wrench_spaces[link].convex_decomposition[ind] for link,ind in zip(self.links,self.indices)]
        elif self.mode == 'inner':
            return [hand_wrench_spaces[link].inner_bounds[ind] for link,ind in zip(self.links,self.indices)]
        elif self.mode == 'outer':
            res = []
            for link,ind in zip(self.links,self.indices):
                if ind == 'all':
                    #stack all outer constraints
                    Ws = [(V.getHPolytope() if hasattr(V,'getHPolytope') else V) for V in hand_wrench_spaces[link].outer_bounds]
                    As = [p.A for p in Ws]
                    bs = [p.b for p in Ws]
                    res.append(polytope.HPolytope(np.vstack(As),np.hstack(bs)))
                else:
                    res.append(hand_wrench_spaces[link].outer_bounds[ind])
            return res
        else:
            raise RuntimeError("mode must be decomposiiton, inner, or outer")
    def transforms(self,robot):
        return [robot.link(l).getTransform() for l in self.links]
    def wrench_transforms(self,robot):
        Tw = []
        for link in self.links:
            (R,t) = robot.link(link).getTransform()
            Tw.append(np.block([[np.array(so3.matrix(R)),np.zeros((3,3))],
                [np.array(so3.matrix(so3.mul(so3.cross_product(t),R))),np.array(so3.matrix(R))]]))
        return Tw

def make_lp(indices,gravity=(0,0,-9.8),minimize_error=False,extremize_dir=None):
    """Makes an LP's constraints with the given wrench polytope to test feasibility of contact wrenches.

    The first 6n variables are the local wrenches w1,...,wn.
    If extremize_dir is True, the external wrench (added to the gravity wrench) is an additional 6 variables.
    If extremize_dir is an array (shape 6xk), the distance along external wrenches are an additional k variables.
    If minimize_error is True, an additional variable is the max constraint violation amount.  This only
      can be applied with H-polytopes.
    If the polytopes are V-polytopes, the remaining variables are the convex hull coefficients.

    Return value is (Aineq,bineq,Aeq,beq)

    v polytope representation (1)
    find wk,ck s.t.
      sum wk = wext
      wk = Vk^T ck
      ck^T e = 1
      ck >= 0
    can we find an equivalent separating plane problem that has a solution iff there is no set of feasible wrenches?
    
    Let zk^T wk - zk0 >= 0 have all the wrenches for link k on one side (Vk zk - zk0 >= 0).  Can we encode the requirement
    that for *all* w satisfying [I I ... I] w = wext, it must also satisfy z1^T w1 - z10 < 0 OR ... OR zN^T wN - zN0 < 0?

    UNRESOLVED...
    """
    global robot,hand_types,hand_wrench_spaces
    from scipy.sparse import coo_matrix,bmat
    extra_cols = 0
    if extremize_dir is not None:
        assert minimize_error == False,"Cannot have wrench extremization and error minimization simultaneously activated"
        if extremize_dir is True:
            extra_cols = 6
        else:
            assert extremize_dir.shape[0] == 6
            extra_cols = extremize_dir.shape[1]
    if minimize_error:
        extra_cols = 1
    cm = robot.getCom()
    totalmass = sum(robot.link(link).getMass().mass for link in xrange(robot.numLinks()))
    wext = vectorops.mul(gravity + vectorops.cross(cm,gravity),totalmass)

    volumes = indices.polytopes(hand_wrench_spaces)
    volumes = [V if not hasattr(V,'getHPolytope') else V.getHPolytope() for V in volumes]
    
    Tw = indices.wrench_transforms(robot)
    if extremize_dir is True:
        Tw.append(-np.eye(6))
    elif extremize_dir is not None:
        Tw.append(-extremize_dir)
    if minimize_error:
        Tw.append(np.zeros((6,1)))
    
    if any([isinstance(V,polytope.VPolytope) for V in volumes]):
        #make a volume LP
        assert not minimize_error,"Can't do V polytopes and error minimization at the same time yet"
        assert all([isinstance(V,polytope.VPolytope) for V in volumes])
        #for V polytopes, need to satisfy
        #wi = Vi*ci
        #sum ci = 1
        #ci >= 0
        blocks = [Tw + [None]*len(volumes)]
        for i,V in enumerate(volumes):
            row = [None]*(len(Tw)+len(volumes))
            row[i] = -np.eye(6)
            row[i+len(Tw)] = V.vertices.T
            blocks.append(row)
        for i,V in enumerate(volumes):
            row = [None]*(len(Tw)+len(volumes))
            row[i+len(Tw)] = np.ones((1,V.vertices.shape[0]))
            blocks.append(row)
        for row in blocks:
            for entry in row:
                if entry is None:
                    print ",",
                else:
                    print entry.shape,",",
            print 
        coo = bmat(blocks).tocoo()
        Aeq =  cvxopt.spmatrix(coo.data,coo.row.tolist(),coo.col.tolist(),size=coo.shape)
        print "Aeq # of nonzeros:",len(coo.row)
        beq = np.hstack([-np.array(wext),np.zeros(6*len(volumes)),np.ones(len(volumes))])
        beq = cvxopt.matrix(beq)
    else:
        Aeq = np.hstack(Tw)
        beq = -np.array(wext)
        Aeq = cvxopt.matrix(Aeq)
        beq = cvxopt.matrix(beq)
    Hrows = []
    Grows = []
    empties = []

    #use scipy sparse matrix stuff to assemble matrix
    blocks = []
    offsets = []
    for i,V in enumerate(volumes):
        row = [None]*len(Tw)
        if isinstance(V,polytope.HPolytope):
            row[i] = coo_matrix(V.A)
            offsets.append(V.b-1e-4)
        if minimize_error:
            row[-1] =coo_matrix(-np.ones((V.A.shape[0],1)))
        if isinstance(V,polytope.VPolytope):
            row[i] = np.zeros((V.vertices.shape[0],6))
            row += [None]*len(volumes)
            row[len(Tw)+i] = coo_matrix(-np.eye(V.vertices.shape[0]))
            offsets.append(np.zeros(V.vertices.shape[0]))
        if extra_cols > 0:
            nrows = 0
            for entry in row:
                if entry is not None:
                    nrows = entry.shape[0]
                    break
            assert nrows > 0
            row[len(Tw)-1] = np.zeros((nrows,extra_cols))
        blocks.append(row)
    for row in blocks:
        for entry in row:
            if entry is None:
                print ",",
            else:
                print entry.shape,",",
        print 
    coo = bmat(blocks).tocoo()
    Aineq =  cvxopt.spmatrix(coo.data,coo.row.tolist(),coo.col.tolist(),size=coo.shape)
    bineq = cvxopt.matrix(np.hstack(offsets))    
    print "Aineq # of nonzeros:",len(coo.row)
    print Aeq.size,beq.size
    print Aineq.size,bineq.size
    return (Aineq,bineq,Aeq,beq)

def solve_lp(indices,gravity,minimize_error=False):
    """Returns world wrenches, and if minimize_error = True, the constraint violation error"""
    global robot,hand_wrench_spaces
    n = len(indices.links)

    print "Assembling LP"
    t0 = time.time()
    cvxopt_args = make_lp(indices,gravity,minimize_error=minimize_error)
    Aineq,bineq,Aeq,beq = cvxopt_args
    c = np.zeros(Aineq.size[1])
    if minimize_error:
        c[n*6] = 1
    t1 = time.time()
    print "Took time",t1-t0
    #print "External wrench",-np.array(beq).ravel()

    print "Solving..."
    cvxopt.solvers.options['show_progress'] = False
    res = cvxopt.solvers.lp(cvxopt.matrix(c),Aineq,bineq,Aeq,beq)
    t2 = time.time()
    print "Took time",t2-t1

    if res['x'] != None:
        wrenches = []
        Tw = indices.wrench_transforms(robot)
        x = np.array(res['x'])
        i = 0
        #print "Link,world wrench,local wrench:"
        for link in indices.links:
            W = hand_wrench_spaces[link]
            wi = x[i*6:(i+1)*6].ravel()
            #print "Wrench space error",np.max(np.dot(W.inner_bounds[0].A,wi)-W.inner_bounds[0].b)
            if not minimize_error and (indices.mode == 'inner' or indices.mode == 'decomposition'):
                assert W.contains(wi)
            #print link,np.dot(Tw[i],wi).T,wi
            wrenches.append(np.dot(Tw[i],wi))
            i += 1
        if minimize_error:
            print "Max constraint violation",x[n*6]
            return wrenches,x[n*6]
        return wrenches
    else:
        print "No solution"
    return None

def compute_support_polygon(indices,gravity,numdivs=32):
    """Returns a support polygon (list of 2D points)"""
    global robot,hand_wrench_spaces
    import scipy.spatial
    n = len(indices.links)

    print "Assembling LP"
    t0 = time.time()
    totalmass = sum(robot.link(link).getMass().mass for link in xrange(robot.numLinks()))
    wbasis = np.zeros((6,2))
    wbasis[3:6,0] = np.array(vectorops.cross([1,0,0],gravity))*totalmass
    wbasis[3:6,1] = np.array(vectorops.cross([0,1,0],gravity))*totalmass
    cvxopt_args = make_lp(indices,gravity,extremize_dir=wbasis)
    Aineq,bineq,Aeq,beq = cvxopt_args
    t1 = time.time()
    print "Took time",t1-t0
    thetas = [math.pi*2*float(i)/numdivs for i in xrange(numdivs)]
    pts = []
    last = None
    for theta in thetas:
        x = math.cos(theta)
        y = math.sin(theta)
        #print "External wrench",-np.array(beq).ravel()
        c = np.zeros(Aineq.size[1])
        c[n*6:n*6+2] = np.array([x,y])

        print "Solving..."
        t1 = time.time()
        cvxopt.solvers.options['show_progress'] = False
        #it looks like warm starting doesn't help ...
        res = cvxopt.solvers.lp(cvxopt.matrix(c),Aineq,bineq,Aeq,beq,primalstart=last,dualstart=last)
        t2 = time.time()
        print "Took time",t2-t1
        if res['x'] == None:
            return None
        x,y = res['x'][n*6:n*6+2]
        pts.append([robot.getCom()[0]+x,robot.getCom()[1]+y])
        print "Extreme point",pts[-1]
        #last = res
        #print "External wrench",-(Aeq[0:6,n*6:n*6+2]*res['x'][n*6:n*6+2])
    ch = scipy.spatial.ConvexHull(np.array(pts))
    return [ch.points[v] for v in ch.vertices]

def test_stability_traditional(gravity=(0,0,-9.8),links=None):
    global robot,hands,hand_types
    if links is None:
        links = hand_types.keys()
    useTorqueLimits = True
    wrench_spaces = [hand_wrench_spaces[link] for link in links]
    origin = robot.getCom()
    #set up friction solver
    folder="FrictionCones/"
    friction_factories = {}
    for link in links:
        h = hands[link]
        for key,unit in h.all_units:
            if unit.type not in friction_factories:
                friction_factories[unit.type] = stability.GeneralizedFrictionConeFactory()
                friction_factories[unit.type].load(folder + unit.type)
    solver = stability.GeneralizedStabilitySolver(friction_factories)
    unitlinks = []
    for link in links:
        h = hands[link]
        for key,unit in h.all_units:
            #print "Adding unit on link",link
            roblink = link.split(":")[0]+':'+key
            assert robot.link(roblink).index >= 0,"Format for link "+roblink+" seems to be incorrect?"
            T = se3.mul(robot.link(roblink).getTransform(),unit.localTransform)
            solver.addContactPatch((T[0],vectorops.sub(T[1],origin)),unit.patchSize,unit.type)
            unitlinks.append(roblink)

    print len(solver.contactPoints),"contact points altogether"
    J = []
    for i in xrange(len(solver.contactPoints)):
        patch = solver.pointIndices[i]
        ci = vectorops.add(solver.contactPoints[i],origin)
        link = robot.link(unitlinks[patch])
        assert link.index >= 0
        ciloc = link.getLocalPosition(ci)
        Ji = link.getPositionJacobian(ciloc)
        #print Ji
        #raw_input()
        J.append(np.array(Ji))

    if useTorqueLimits:
        #Torque balance constraints
        tmax = robot.getTorqueLimits()
        tmin = [-v for v in tmax]
        q = robot.getConfig()
        qmin,qmax = robot.getJointLimits()
        #assume infinite bounds on first 6 links
        inf = float('inf')
        for i in xrange(6):
            tmin[i] = -inf
            tmax[i] = inf
        for i in xrange(robot.numLinks()):
            if q[i] == qmin[i]:
                #print "Joint",robot.link(i).getName(),"at minimum, assuming infinite positive torque limit"
                tmax[i] = inf
            if q[i] == qmax[i]:
                #print "Joint",robot.link(i).getName(),"at maximum, assuming infinite negative torque limit"
                tmin[i] = -inf
        #torque balance testing
        #t = G - sum Ji^T (-fi), tmin <= t <= tmax
        G = np.array(robot.getGravityForces(gravity))
        for i in xrange(robot.numLinks()):
            if tmin[i] > -inf or tmax[i] < inf:
                row = np.hstack([Ji.T[i,:] for Ji in J])
                rhs = G[i]
                solver.addLinearConstraint(row,tmin[i]-rhs,tmax[i]-rhs)
    totalmass = sum(robot.link(link).getMass().mass for link in xrange(robot.numLinks()))
    solver.setGravityWrench(totalmass,[0,0,0],gravity)
    t0 = time.time()
    #perform the solve
    try:
        wrenches = solver.solve()
    except ValueError as e:
        print e
        raise
        print "Probably don't have a valid static equilibrium solution"
        print "Equality constraints:"
        print solver.A_eq_extra
        wrenches = None
    t1 = time.time()
    print "Solver took time",t1-t0
    return wrenches

def test_stability(gravity=(0,0,-9.8),links=None):
    """Solves the problem
    wext = -sum{f=1,...N} wf
    with wf s.t.
    wf in Tf*Hf
    with Hf the volume of valid wrenches

    To transform a wrench w=(f,m) from one frame to another via the transform Tx = Rx+t
    the result is (Rf,t x Rf + Rm)

    To do the inverse, the inverse transform is T^-1 x = R^T x - R^T t, which gives the
    resulting wrench (R^T f, -R^T (t x f) + R^T m)

    The optimization is over local wrenches w1l,...,wNl satisfying wfl in Hf,
    wext = sum{f=1,...,N} Tf wfl
    """
    global robot,hand_types,hand_wrench_spaces
    if links is None:
        links = hand_types.keys()
    n = len(links)
    
    wrench_spaces = [hand_wrench_spaces[link] for link in links]
    innermax = max(len(W.inner_bounds) for W in wrench_spaces)
    outermax = max(len(W.outer_bounds) for W in wrench_spaces)
    decompmax = max(len(W.convex_decomposition) for W in wrench_spaces)
    assert decompmax == 1,"Can only handle convex wrench spaces at the moment"
    indices = WrenchSpaceIndices(links,[0]*n,'inner')
    for i in xrange(max(innermax,outermax)):
        if i < innermax:
            indices.mode = 'inner'
            indices.indices = [min(i,len(hand_wrench_spaces[link].inner_bounds)-1) for link in links]
            print
            print "Trying inner bound",indices.indices
            wrenches,error = solve_lp(indices,gravity,minimize_error=True)
            if wrenches != None and error < 0:
                print "Solved with inner bound",indices.indices
                print "Margin",error
                return True
            else:
                print "Inner margin of error",error
        if i < outermax:
            indices.mode = 'outer'
            indices.indices = [ min(i,len(hand_wrench_spaces[link].outer_bounds)-1) for link in links]
            print
            print "Trying outer bound",indices.indices
            wrenches = solve_lp(indices,gravity)
            if wrenches == None:
                print "Pruned with outer bound",indices.indices
                return False
    print "Trying overall outer bound"
    indices.mode = 'outer'
    indices.indices = ['all']*len(links)
    wrenches = solve_lp(indices,gravity)
    if wrenches == None:
        print "Pruned with outer bound",indices.indices
        return False

    indices.mode = 'decomposition'
    indices.indices = [0]*len(links)
    print
    print "Trying decomposition",indices.indices
    wrenches = solve_lp(indices,gravity)
    if wrenches == None:
        print "Not in decomposition"
        return False
    return True

def test_stability_running_time(gravity=(0,0,-9.8),links=None):
    global hand_wrench_spaces
    global robot,hand_types,hand_wrench_spaces
    if links is None:
        links = hand_types.keys()
    n = len(links)
    wrench_spaces = [hand_wrench_spaces[link] for link in links]
    inner_sizes = [12,25,50,100,250,500]
    outer_sizes = [50,100,250,500,1000,2500,5000]
    print "Sampling bounds..."
    for W in wrench_spaces:
        for i,s in enumerate(inner_sizes):
            W.sampleInnerBound(s,i)
        for i,s in enumerate(outer_sizes):
            W.sampleOuterBound(s,i+2)

    print "Saving results to running_time_stats.csv..."
    innermax = max(len(W.inner_bounds) for W in wrench_spaces)
    outermax = max(len(W.outer_bounds) for W in wrench_spaces)
    decompmax = max(len(W.convex_decomposition) for W in wrench_spaces)
    assert decompmax == 1,"Can only handle convex wrench spaces at the moment"

    f = open('running_time_stats.csv','w')
    f.write("Polytope type,size,time,stable\n")
    indices = WrenchSpaceIndices(links,[0]*n,'inner')
    for i in range(innermax):
        indices.indices = [i]*n
        try:
            t0 = time.time()
            wrenches = solve_lp(indices,gravity)
            t1 = time.time()
            success = (wrenches is not None)
            f.write("V,%d,%f,%d\n"%(wrench_spaces[0].inner_bounds[i].vertices.shape[0],t1-t0,int(success)))
            f.flush()
        except KeyboardInterrupt:
            break
    indices.mode = 'outer'
    for i in range(outermax):
        indices.indices = [i]*n
        try:
            t0 = time.time()
            wrenches = solve_lp(indices,gravity)
            t1 = time.time()
            success = (wrenches is not None)
            if i == 0:
                f.write("B,12,%f,%d\n"%(t1-t0,int(success)))
            else:
                f.write("H,%d,%f,%d\n"%(wrench_spaces[0].outer_bounds[i].A.shape[0],t1-t0,int(success)))
            f.flush()
        except KeyboardInterrupt:
            break
    print "Solving decomposition..."
    indices.mode = 'decomposition'
    indices.indices = [0]*n
    try:
        t0 = time.time()
        wrenches = solve_lp(indices,gravity)
        t1 = time.time()
        success = (wrenches is not None)
        f.write("V,%d,%f,%d\n"%(wrench_spaces[0].convex_decomposition[0].vertices.shape[0],t1-t0,int(success)))
    except KeyboardInterrupt:
        pass

    print "Done."
    f.close()

from klampt.model import trajectory
load_robot()
load_hand_configs()
load_wrench_spaces()

#inner_sizes = [12,25,50,100,250]
#outer_sizes = [50,100,250,500,1000]
inner_sizes = [250]
outer_sizes = [1000]
print "Sampling bounds..."
seed = 12346
import random
for W in hand_wrench_spaces.itervalues():
    for i,s in enumerate(inner_sizes):
        random.seed(seed)
        np.random.seed(seed)
        W.sampleInnerBound(s,i)
    for i,s in enumerate(outer_sizes):
        random.seed(seed)
        np.random.seed(seed)
        W.sampleOuterBound(s,i+2)
innermax = max(len(W.inner_bounds) for W in hand_wrench_spaces.itervalues())
outermax = max(len(W.outer_bounds) for W in hand_wrench_spaces.itervalues())
decompmax = max(len(W.convex_decomposition) for W in hand_wrench_spaces.itervalues())
assert decompmax == 1,"Can only handle convex wrench spaces at the moment"

vis.visualization._frontend.clearColor = (1,1,1,1)
vis.add("world",world)
vis.hideLabel("world")
while True:
    edit_config()
    #test_stability_running_time()
    #break
    #print test_stability()
    #print test_stability_traditional()
    links = hand_types.keys()
    indices = WrenchSpaceIndices(links,[0]*len(links),'inner')
    for i in xrange(innermax):
        indices.indices = [i]*len(links)
        sp = compute_support_polygon(indices,(0,0,-9.8))
        if sp is not None:
            points = [(p[0],p[1],0) for p in sp] 
            points.append(points[0])
            name = "inner "+str(inner_sizes[i])
            if len(inner_sizes) == 1:
                name = "inner"
            vis.add(name,trajectory.Trajectory(range(len(points)),points))
            vis.setColor(name,0,1,float(i)/float(innermax))
            vis.hideLabel(name)
    indices.mode = 'outer'
    if len(outer_sizes) > 1:
        for i in xrange(outermax):
            indices.indices = [i]*len(links)
            sp = compute_support_polygon(indices,(0,0,-9.8))
            if sp is not None:
                points = [(p[0],p[1],0) for p in sp] 
                points.append(points[0])
                if i == 0:
                    name = 'outer BB'
                elif i == 1:
                    name = 'outer f+m'
                else:
                    name = "outer "+str(outer_sizes[i-2])
                vis.add(name,trajectory.Trajectory(range(len(points)),points))
                vis.setColor(name,1,float(i)/float(outermax),0)
                vis.hideLabel(name)
    if len(outer_sizes) <= 1:
        indices.mode = 'outer'
        indices.indices = ['all']*len(links)
        sp = compute_support_polygon(indices,(0,0,-9.8))
        if sp is not None:
            points = [(p[0],p[1],0) for p in sp] 
            points.append(points[0])
            vis.add("outer",trajectory.Trajectory(range(len(points)),points))
            vis.setColor("outer",1,0,0)
            vis.hideLabel("outer")
    vis.show()
    while vis.shown():
        time.sleep(0.1)
