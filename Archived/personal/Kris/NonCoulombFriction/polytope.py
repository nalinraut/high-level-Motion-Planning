import numpy as np
import scipy.spatial
from klampt import TriangleMesh,Geometry3D
from klampt.math import vectorops,so3
from OpenGL.GL import *
import copy
import os
import math

#CONFIGURE THIS FOR APPROXIMATE CONVEX DECOMPOSITION
VHACDProgram = '/home/motion/Code/v-hacd/build/linux2/test/testVHACD'
#the amount of concavity allowed to avoid ACD calls
max_concavity = 0.1

def load_matrix(fn):
    """Loads a floating point numpy array from a simple text format
    M N
    v11 v12 ... v1n
    ...
    vm1 vm2 ... vmn
    """
    f = open(fn,'r')
    items = ' '.join(f.readlines())
    f.close()
    entries = items.split()
    m,n = int(entries[0]),int(entries[1])
    assert m*n == len(entries)-2
    mat = np.array([float(v) for v in entries[2:]])
    mat = mat.reshape((m,n))
    return mat

def save_matrix(A,fn):
    """Saves a numpy array to a simple text format
    M N
    v11 v12 ... v1n
    ...
    vm1 vm2 ... vmn
    """
    m,n = A.shape
    f = open(fn,'w')
    f.write("%d %d\n"%(m,n))
    for i in xrange(m):
        f.write(' '.join([str(A[i,j]) for j in xrange(n)]))
        f.write('\n')
    f.close()

def hull_to_klampt_geom(hull):
    """Converts a 3D hull to a Klamp't Geometry3D object."""
    if len(hull.points) != 0:
        if len(hull.points[0]) != 3:
            raise RuntimeError("Invalid hull, not 3D")
    mesh = TriangleMesh()
    ptov = dict()
    ind =  0
    for v in hull.vertices:
        p = hull.points[v] 
        mesh.vertices.append(p[0])
        mesh.vertices.append(p[1])
        mesh.vertices.append(p[2])
        ptov[v]=ind
        ind+=1
    for s in hull.simplices:
        assert len(s) == 3,"TODO: triangulate non-triangular facets"
        mesh.indices.append(ptov[s[0]])
        mesh.indices.append(ptov[s[1]])
        mesh.indices.append(ptov[s[2]])
    return Geometry3D(mesh)

def klampt_geom_to_hull(geom):
    """Calculates the convex hull of a Klamp't Geometry3D object."""
    if geom.type() != "TriangleMesh":
        raise RuntimeError("Geometry is not a TriangleMesh object")
    mesh = geom.getTriangleMesh()
    varray = np.array([v for v in mesh.vertices]).reshape((len(mesh.vertices)/3,3))
    return scipy.spatial.ConvexHull(varray)

def pockets(geom):
    """For a given Klamp't Geometry3D object, returns the indices of faces that are not
    on the convex hull."""
    if geom.type() != "TriangleMesh":
        raise RuntimeError("Geometry is not a TriangleMesh object")
    mesh = geom.getTriangleMesh()
    varray = np.array([v for v in mesh.vertices]).reshape((len(mesh.vertices)/3,3))
    iarray = np.array([v for v in mesh.indices],dtype=np.int32).reshape((len(mesh.indices)/3,3))
    hull = scipy.spatial.ConvexHull(varray)
    vint = []
    vext = set(hull.vertices)
    for i in xrange(varray.shape[0]):
        if i not in vext:
            #check coplanarity
            #print "Distance to pocket vertex",hull_distance(varray[i,:],hull)
            if hull_distance(varray[i,:],hull) < -1e-8:
                vint.append(i)
    vint = set(vint)
    pocketTris = []
    for i in xrange(iarray.shape[0]):
        if any(v in vint for v in iarray[i,:]):
            pocketTris.append(i)
    return pocketTris


def halfplanes(hull):
    """Given a scipy ConvexHull object, computes a matrix A and a vector b such
    that A x <= b describes the interior of the convex polytope"""
    A = hull.equations[:,0:hull.equations.shape[1]-1]
    b = -hull.equations[:,hull.equations.shape[1]-1]
    return A,b

def in_hull(point,hull):
    err = np.dot(hull.equations,point.tolist()+[1])
    return max(err) <= 0

def hull_distance(point,hull):
    err = np.dot(hull.equations,point.tolist()+[1])
    dmax = max(err)
    #if dmax > 0:
    #   for v in hull.vertices:
    #       dmax = min(dmax,np.linalg.norm(hull.points[v]-np.array(point)))
    return dmax

def approximate_convex_decomposition(geom):
    """Computes the approximate convex decomposition of the given Geometry3D object (but perhaps with vertices
    moved so it is not convex).  Result is a list of ConvexHull objects.

    Note: Not thread safe! There is a small chance of a race condition.

    For degenerate objects, this will raise either an IOException or scipy.spatial.qhull.QhullError.
    """
    from vrml.vrml97 import parser
    assert geom.type() == 'TriangleMesh'
    tempnam = 'temp'
    #possible race condition here with multi-threaded implementations...
    if os.path.exists(tempnam+'.obj'):
        found = False
        for i in range(1,100):
            if not os.path.exists('temp'+str(i)+'.obj'):
                tempnam = 'temp'+str(i)
                found=True
                break
        if not found:
            raise RuntimeError("Could not find a valid temp[N].obj file name... maybe need to manually clear out temp*.obj?")
    geom.saveFile(tempnam+'.obj')
    res = os.system(VHACDProgram+' --input %s.obj --output %s.wrl'%(tempnam,tempnam))
    if res != 0:
        print "Warning, didn't get a normal status flag from VHACD?",res
    p = parser.buildParser()
    f = open(tempnam+".wrl","r")
    wrl = p.parse(''.join(f.readlines()))
    
    os.remove(tempnam+'.obj')
    os.remove(tempnam+'.wrl')

    #now recursively extract meshes
    scenegraph = wrl[1][1]
    def extract_vrml_meshes(obj):
        res = []
        if hasattr(obj,'children'):
            for c in obj.children:
                res += extract_vrml_meshes(c)
        if hasattr(obj,'geometry'):
            indexlist = obj.geometry.coordIndex
            coords = obj.geometry.coord.point
            hull = scipy.spatial.ConvexHull(coords)
            #hull.points = coords
            #hull.vertices = range(coords.shape[0])
            #hull.simplices = np.array(indexlist).reshape((len(indexlist)/4,4))
            #assert all(v == -1 for v in hull.simplices[:,3])
            #hull.simplices = hull.simplices[:,0:3]
            res.append(hull)
        return res
    return extract_vrml_meshes(scenegraph)

def draw_hull(hull,scale=0.01):
    """Draws a ConvexHull with 3D texture coordinates"""
    if len(hull.simplices)==0:
        print "Hull with no simplices?"
        return
    centroid = sum([hull.points[v] for v in hull.vertices],[0.0]*3) / len(hull.vertices) * scale
    vmin = [min([hull.points[v][i] for v in hull.vertices])*scale for i in range(3)]
    vmax = [max([hull.points[v][i] for v in hull.vertices])*scale for i in range(3)]
    uvwscale = [1.0/(b-a) if b != a else 1.0 for (a,b) in zip(vmin,vmax)]

    for simplex in hull.simplices:
        ia,ib,ic = simplex
        a = vectorops.mul(hull.points[ia].tolist(),scale)
        b = vectorops.mul(hull.points[ib].tolist(),scale)
        c = vectorops.mul(hull.points[ic].tolist(),scale)
        n = vectorops.cross(vectorops.sub(b,a),vectorops.sub(c,a))
        if vectorops.dot(n,vectorops.sub(centroid,a)) > 0:
            b,c = c,b
            n = vectorops.mul(n,-1)
        try:
            n = vectorops.mul(n,1.0/vectorops.norm(n))
            glNormal3f(*n)
        except ZeroDivisionError:
            pass
        glBegin(GL_TRIANGLES)
        glTexCoord3f(uvwscale[0]*(a[0]-vmin[0]),uvwscale[1]*(a[1]-vmin[1]),uvwscale[2]*(a[2]-vmin[2]))
        glVertex3f(*a)
        glTexCoord3f(uvwscale[0]*(b[0]-vmin[0]),uvwscale[1]*(b[1]-vmin[1]),uvwscale[2]*(b[2]-vmin[2]))
        glVertex3f(*b)
        glTexCoord3f(uvwscale[0]*(c[0]-vmin[0]),uvwscale[1]*(c[1]-vmin[1]),uvwscale[2]*(c[2]-vmin[2]))
        glVertex3f(*c)
        glEnd()

def collapse_hull(hull):
    nonredundant = [0]
    for row in xrange(1,hull.equations.shape[0]):
        a = hull.equations[row-1]
        b = hull.equations[row]
        if any(abs(u-v) > 1e-8 for (u,v) in zip(a,b)):
            nonredundant.append(row)
    print len(nonredundant),"/",hull.equations.shape[0],"nonredundant"
    hull.equations = hull.equations[nonredundant,:]

def generalized_slice(n,Aineq,bineq,Aeq,beq,maxdepth=5,tol=1e-3):
    """For a bounded polytope specified by constraints 
    Aineq x <= bineq, Aeq x = beq,
    takes a slice of the first n dimensions through the origin.

    Return value is a ConvexHull object.
    """
    assert n >= 1 and n <= 3,"Can only take slices up to 3D at the moment"
    m = Aineq.shape[1]-n
    print "slice_hull: taking %d-D slice of hull with dimension %d"%(n,Aineq.shape[1])
    print "   # inequalities: %d, #equalities: %d"%(Aineq.shape[0],Aeq.shape[0])

    import cvxopt
    Aineq = cvxopt.sparse(cvxopt.matrix(Aineq))
    bineq = cvxopt.matrix(bineq)
    Aeq = cvxopt.sparse(cvxopt.matrix(Aeq))
    beq = cvxopt.matrix(beq)
    cvxopt.solvers.options['show_progress'] = False
    
    c = np.zeros(m+n)
    seeds = []
    for i in xrange(n):
        c[i] = 1
        res = cvxopt.solvers.lp(cvxopt.matrix(c),Aineq,bineq,Aeq,beq)
        if res['x'] == None:
            print "slice_hull: no intersection with hull"
            return None
        seeds.append(np.array(res['x'])[:n].ravel())
        c[i] = -1
        res = cvxopt.solvers.lp(cvxopt.matrix(c),Aineq,bineq,Aeq,beq)
        if res['x'] == None:
            print "slice_hull: no intersection with hull"
            return None
        seeds.append(np.array(res['x'])[:n].ravel())
        c[i] = 0

    if n == 1:
        return (seeds[0][0],seeds[1][0])
    else:
        #this is kinda expensive, but so what...
        changed = True
        numiters = 0
        print "slice_hull: Round 0 started with",len(seeds),"points"
        ch = scipy.spatial.ConvexHull(np.array(seeds))
        while changed and numiters < maxdepth:
            changed = False
            numiters += 1
            numadded = 0
            A,b = halfplanes(ch)
            for i in xrange(A.shape[0]):
                c[:n] = -A[i]
                res = cvxopt.solvers.lp(cvxopt.matrix(c),Aineq,bineq,Aeq,beq)
                if np.dot(A[i],np.array(res['x'])[:n].ravel()) > b[i] + tol:
                    #new point
                    seeds.append(np.array(res['x'])[:n].ravel())
                    numadded += 1
            print "slice_hull: Round",numiters,"added",numadded,"points"
            if numadded > 0:
                ch = scipy.spatial.ConvexHull(seeds)
                changed=True
        return ch


def slice_hull(hull,A,offset=None):
    """For a hull H in R^m, and an affine transform y = A*x+offset, returns a hull H' in R^n
    so that y in H iff x in H'.

    For each simplex s in H, this might intersect the constraints y = A*x+offset, but not necessarily.
    If x[1],...,x[m] are the vertices of the simplex, then there exists x and c[1],...,c[m] simultaneously
    satisfying:

      y = c[1]*x[1] + ... + c[m]*x[m] = A*x + offset
      c[1] + ... + c[m] = 1
      c[1], ..., c[m] >= 0

    If so, we can grow a simplex by extremizing x in different directions until all n directions are spanned.

    Another approach would be to formulate a similar LP but just extremize in all directions subject to
    polytope constraints.
    """
    assert A.shape[0] == hull.points.shape[1]
    if offset is None: offset = np.zeros(A.shape[0])
    assert offset.shape[0] == len(offset) and offset.shape[0] == A.shape[0]
    Aineq,bineq = halfplanes(hull)
    Aineq = np.hstack((np.zeros((Aineq.shape[0],A.shape[1])),Aineq))
    Aeq = np.hstack((A,-np.eye(A.shape[0])))
    beq = -offset
    n = A.shape[1]
    return generalized_slice(n,Aineq,bineq,Aeq,beq)


def normalize(hull):
    #normalize each equation so they have unit normal
    for i in xrange(hull.equations.shape[0]):
        n = np.linalg.norm(hull.equations[i,:-1])
        if n > 1e-5:
            hull.equations[i,:] *= 1.0/n

class Volume:
    def __init__(self):
        self.convex_decomposition = []
        self.convex_hull = None

    def setPoints(self,points):
        """Sets this as a convex hull of some set of points."""
        self.convex_hull = scipy.spatial.ConvexHull(points)
        self.convex_decomposition= [self.convex_hull]

    def setGeom(self,g):
        global max_concavity
        self.convex_hull = klampt_geom_to_hull(g)
        if len(self.convex_hull.vertices) < self.convex_hull.points.shape[0]:
            A,b = halfplanes(self.convex_hull)
            err = np.dot(A,self.convex_hull.points.T)-np.hstack([b.reshape((len(b),1))]*self.convex_hull.points.shape[0]);
            #print "Most interior point",np.min(np.max(err,axis=0))
            #raw_input()
            if np.min(np.min(np.max(err,axis=0))) > -max_concavity:
                #pretty close
                self.convex_decomposition = [self.convex_hull]
            else:
                self.convex_decomposition = approximate_convex_decomposition(g)
        else:
            self.convex_decomposition = [self.convex_hull]

    def load(self,folder):
        """Loads a nonconvex volume from a folder"""
        self.convex_decomposition = []
        for fn in sorted(os.listdir(folder)):
            hull = scipy.spatial.ConvexHull(load_matrix(os.path.join(folder,fn)))
            collapse_hull(hull)
            self.convex_decomposition.append(hull)
            print "Volume",fn,"has",len(self.convex_decomposition[-1].simplices),"faces"
        self.calc_convex_hull()

    def loadGeom(self,fn):
        """Loads a nonconvex volume from a Klamp't readable mesh (must be watertight)"""
        g = Geometry3D()
        g.loadFile(fn)
        self.setGeom(g)

    def transform(self,mat,scale=None,offset=None,orthogonal=False):
        """Returns a transformed Volume by the given matrix, scaling, and translation.  If orthogonal, mat is assumed to be a rotation.
        """
        #transform point based representation 
        #hull(p1,...,pn)
        #represented in V-form with the matrix P = [p1,...,pn]^T
        #and in the H-form with the linear inequality Ax >= b  (more specifically Ax - b >= 0 is packed in the equations array in the form [A|-b])
        #Find A',b' so that if A x - b > 0 then  A' cx - b' >= 0.  Want A' = A, too.
        #
        #P' = P by M^T
        #each row of A x = b is defined by (pi,pj,pk)
        #A pi = A pj = A pc = b
        #Find A' such that A' M pi = A' M pj = A' M pk = b
        #therefore, let A' = A M^-1
        res = Volume()
        res.convex_decomposition = []
        if scale is None:
            scale = 1.0
        Minv = np.linalg.inv(mat)
        for A in self.convex_decomposition:
            res.convex_decomposition.append(copy.copy(A))
            res.convex_decomposition[-1].points = np.dot(A.points,mat.T*scale) 
            res.convex_decomposition[-1].equations = np.hstack((np.dot(A.equations[:,:-1],Minv),A.equations[:,-1:]))
            if not orthogonal:
                normalize(res.convex_decomposition[-1])
            if scale != 1.0:
                assert scale > 0
                res.convex_decomposition[-1].equations[:,-1] *= scale
        res.convex_hull = copy.copy(self.convex_hull)
        if res.convex_hull is not None:
            res.convex_hull.points = np.dot(self.convex_hull.points,mat.T*scale)
            res.convex_hull.equations = np.hstack((np.dot(self.convex_hull.equations[:,:-1],Minv),self.convex_hull.equations[:,-1:]))
            if not orthogonal:
                normalize(res.convex_hull)
            if scale != 1.0:
                assert scale > 0
                res.convex_hull.equations[:,-1] *= scale
        if offset is not None:
            raise NotImplementedError("Volume.transform() affine offsets")
        return res
    def calc_convex_hull(self):
        Vs = [np.vstack([D.points[v] for v in D.vertices]) for D in self.convex_decomposition]
        self.convex_hull = scipy.spatial.ConvexHull(np.vstack(Vs))
        collapse_hull(self.convex_hull)
    def drawGL(self,size=0.01):
        glEnable(GL_LIGHTING)
        glEnable(GL_BLEND)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_CULL_FACE)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        for i,hull in enumerate(self.convex_decomposition):
            if i == 0:
                glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,(1,1,0,0.5))
            elif i == 1:
                glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,(1,0.5,0,0.5))
            elif i == 2:
                glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,(0,1,0,0.5))
            elif i == 3:
                glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,(0,1,0.5,0.5))
            draw_hull(hull,size)
        #glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,(0,1,1,0.25))
        #draw_hull(self.convex_hull,size)
        glDisable(GL_BLEND)

class BoundsPolytope:
    def __init__(self,bmin=None,bmax=None):
        if bmin is None:
            self.bmin = None
            self.bmax = None
        else:
            self.bmin = np.array(bmin)
            self.bmax = np.array(bmax)
    def setHull(self):
        vertices = hull.points[hull.vertices,:]
        self.bmin,self.bmax = np.min(vertices,axis=0),np.max(vertices,axis=0)
    def contains(self,x):
        return all(a <= v and v <= b for (a,b,v) in zip(self.bmin,self.bmax,x))
    def getHPolytope(self):
        b = HPolytope()
        b.setBounds(self.bmin,self.bmax)
        return b
    def transform(self,mat,offset=None):
        return self. getHPolytope().transform(mat,offset)
    def extremum(self,direction):
        raise NotImplementedError("TODO")

class VPolytope:
    """ A polytope in n-D space specified in V-representation
    - vertices: mxn np.array
    """
    def __init__(self,vertices=None):
        self.vertices = vertices

    def setHull(self,hull):
        self.vertices = hull.points[hull.vertices,:]

    def contains(self,x):
        """Returns true if this polytope contains the wrench x.  The LP searches for a separating plane between x and the polytope's vertices."""
        #return self.contains_scip(x)
        import cvxopt
        d = self.vertices.shape[1]
        n = self.vertices.shape[0]
        constraints = np.block([[self.vertices,-np.ones((n,1))],
                [x.reshape((1,len(x))),np.array([-1])]])
        rhs = np.zeros(self.vertices.shape[0]+1)
        rhs[-1] = 1.0
        c = -np.hstack((x,[-1]))
        cvxopt.solvers.options['show_progress'] = False
        res = cvxopt.solvers.lp(cvxopt.matrix(c),cvxopt.matrix(constraints),cvxopt.matrix(rhs))
        if res['x']:
            if np.linalg.norm(np.array(res['x'])) < 1e-6*math.sqrt(d):
                #only point was 0
                return True
        return res['x'] == None

    def contains_scip(self,x):
        """Uses pyscipopt... found to be much slower than cvxopt."""
        from pyscipopt import Model,quicksum
        if not hasattr(self,'model'):
            model = Model("Vpolytope")
            M = 1000
            zs = []
            z0 = model.addVar(vtype="C", name="z0", lb=-M, ub=M)
            zs.append(z0)
            for i in xrange(self.vertices.shape[1]):
                zi = model.addVar(vtype="C", name="z"+str(i+1), lb=-M, ub=M)
                zs.append(zi)

            # Set up inequality constraints
            for i in xrange(self.vertices.shape[0]):
                model.addCons(quicksum(zi*xi for (zi,xi) in zip(zs[1:],self.vertices[i])) <= 0,name="v"+str(i))
            #set up bound
            bnd = model.addCons(quicksum(zi*xi for (zi,xi) in zip(zs[1:],x))-z0 <= 1, name='bound')
            model.data = zs,bnd
            #TODO: need to upgrade pyscipopt
            #self.model = model
        else:
            model = self.model
            zs,bnd = model.data
            z0 = zs[0]
            model.chgLhs(bnd,quicksum(zi*xi for (zi,xi) in zip(zs[1:],x))-z0)
        # Set up objective
        model.setObjective(quicksum(zi*xi for (zi,xi) in zip(zs[1:],x))-z0, 'maximize')
        t0 = time.time()
        #print "Beginning SCIP optimization..."
        model.hideOutput()
        model.optimize()
        t1 = time.time()
        #print "SOLVE TIME",t1-t0
        #print "STATUS:",model.getStatus()
        if model.getStatus() != 'optimal':
            return True
        #print "Optimal value :",model.getObjVal()
        if model.getObjVal() < 1e-6*math.sqrt(self.vertices.shape[1]):
            return True
        return False

    def contains_gurobi(self,x):
        """Uses Gurobi... nottested yet"""
        from gurobipy import Model
        if not hasattr(self,'model'):
            model = Model("Vpolytope")
            M = 1000
            zs = []
            z0 = model.addVar(name="z0", lb=-M, ub=M)
            zs.append(z0)
            for i in xrange(self.vertices.shape[1]):
                zi = model.addVar(name="z"+str(i+1), lb=-M, ub=M)
                zs.append(zi)

            # Set up inequality constraints
            for i in xrange(self.vertices.shape[0]):
                model.addCons(sum(zi*xi for (zi,xi) in zip(zs[1:],self.vertices[i])) <= 0,name="v"+str(i))
            #set up bound
            bnd = model.addCons(sum(zi*xi for (zi,xi) in zip(zs[1:],x))-z0 <= 1, name='bound')
            model.data = zs,bnd
            #TODO: need to upgrade pyscipopt
            #self.model = model
        else:
            model = self.model
            zs,bnd = model.data
            z0 = zs[0]
            model.remove(bnd)
            bnd = model.addCons(sum(zi*xi for (zi,xi) in zip(zs[1:],x))-z0 <= 1,name='bound')
            model.data = zs,bnd
        # Set up objective
        model.setObjective(sum(zi*xi for (zi,xi) in zip(zs[1:],x))-z0, 'maximize')
        t0 = time.time()
        #print "Beginning Gurobi optimization..."
        model.hideOutput()
        model.optimize()
        t1 = time.time()
        #print "SOLVE TIME",t1-t0
        #print "STATUS:",model.getStatus()
        if model.getStatus() != 'optimal':
            return True
        #print "Optimal value :",model.getObjVal()
        if model.getObjVal() < 1e-6*math.sqrt(self.vertices.shape[1]):
            return True
        return False

    def reduce(self):
        """Produce a reduced version of this polytope with interior vertices taken out"""
        import cvxopt
        raise NotImplementedError("Not implemented yet")

    def extremum(self,direction):
        """Returns the extremum in the given direction"""
        x = np.dot(self.vertices,direction)
        return self.vertices[max(zip(x,range(len(x))))[1],:]

    def boundingPolytope(self):
        bmin,bmax = np.min(self.vertices,axis=0),np.max(self.vertices,axis=0)
        return BoundsPolytope(bmin,bmax)

    def transform(self,mat,offset=None):
        res = VPolytope()
        res.vertices = np.dot(self.vertices,mat.T)
        if offset is not None:
            res.vertices += np.array([offset]*self.vertices.shape[0])
        return res

    def slice(self,A,offset=None):
        """Same arguments and return value as slice_hull."""
        #sum ci*vi = A*x+offset
        #sum ci = 1
        #ci >= 0
        #so variables are X=x,c1,...,cn
        #inequality is [0|-I]X <= 0
        #equality is [0 |e^T]X = [1     ]
        #            [-A|V  ]    [offset]
        if offset is None: offset = np.zeros(A.shape[0])
        assert offset.shape[0] == len(offset) and offset.shape[0] == A.shape[0]
        n = A.shape[1]
        nc = self.vertices.shape[0]
        Aineq = np.hstack((np.zeros((nc,n)),-np.eye(nc)))
        bineq = np.zeros(nc)
        Aeq = np.block([[-A,self.vertices.T],
                        [np.zeros((1,n)),np.ones((1,nc))]])
        beq = np.hstack((offset,np.ones(1)))
        return generalized_slice(n,Aineq,bineq,Aeq,beq)

class HPolytope:
    """ A polytope in n-D space specified in H-representation Ax <= b
    - A: mxn np.array
    - b: mx1 np.array
    """
    def __init__(self,A=None,b=None):
        self.A = A
        self.b = b
        self.cvxopt_matrix = None
        self.cvxopt_vector = None

    def setHull(self,hull):
        self.A,self.b = halfplanes(hull)
        self.cvxopt_matrix = None
        self.cvxopt_vector = None

    def setBounds(self,bmin,bmax):
        self.b = np.hstack((bmax,-bmin))
        self.A = np.vstack([np.eye(len(bmin)),-np.eye(len(bmin))])
        self.cvxopt_matrix = None
        self.cvxopt_vector = None

    def setVPolytope(self,vrep):
        hull = scipy.spatial.ConvexHull(vrep.vertices)
        self.setHull(hull)

    def contains(self,x):
        """Returns true if this polytope contains the wrench x"""
        return np.max(np.dot(self.A,x) - self.b) <= 0
        
    def extremum(self,direction):
        """Returns the extremum in the given direction"""
        raise NotImplementedError()

    def transform(self,mat,offset=None):
        res = HPolytope()
        #if A*x <= b in the original polytope, we want A' and b' such that
        #A'*y <= b is an equivalent condition with y=mat*x + offset
        #x = mat^-1*(y-offset)
        #=> A' = A*mat^-1, b' = b + mat^-1*A*offset
        matinv = np.linalg.inv(mat)
        res.A = np.dot(self.A,matinv)
        res.b = self.b
        if offset is not None:
            res.b = self.b + np.dot(matinv,np.dot(self.A,offset))
        return res

    def slice(self,A,offset=None):
        """Same arguments and return value as slice_hull."""
        if offset is None: offset = np.zeros(A.shape[0])
        assert offset.shape[0] == len(offset) and offset.shape[0] == A.shape[0]
        #H (A x + offset) <= g
        Aineq = np.dot(self.A,A)
        bineq = self.b - np.dot(self.A,offset)
        Aeq = np.zeros((0,A.shape[1]))
        beq = np.zeros(0)
        return generalized_slice(A.shape[1],Aineq,bineq,Aeq,beq)

class GeneralizedWrenchPolytope:
    """
    A 6D wrench polytope that may be nonconvex. It is represented as a union of convex polytopes,
    each specified in V-representation.

    Members:
    - outer_bounds: one or more outer bound polytopes
    - inner_bounds: one or more inner bound polytopes
    - convex_decomposition: a list of convex polytopes representing the shape.
      Ultimately the polytope is the union of CH(V[1])...CH(V[n]) where V[i] is the i'th entry in convex_decomposition.

    File formats include 1) the pickled convex decomposition, or 2) a wrench_slices.csv file with a set
    of associated obj files.
    """
    def __init__(self):
        self.outer_bounds = []
        self.inner_bounds = []
        self.convex_decomposition = []
        self.numInnerSamples = 50
        self.numOuterSamples = 200

    def convex(self):
        return len(self.convex_decomposition) == 1

    def contains(self,x):
        """Returns true if this polytope contains the wrench x"""
        for b in self.outer_bounds:
            if not b.contains(x):
                return False
        for b in self.inner_bounds:
            if b.contains(x):
                assert any(c.contains(x) for c in self.convex_decomposition)
                return True
        for b in self.convex_decomposition:
            if b.contains(x):
                return True
        return False

    def sampleInnerBound(self,numInnerSamples,index=0):
        import random
        assert len(self.convex_decomposition) == 1,"Can only handle convex wrench spaces for now"
        vpolytope = self.convex_decomposition[0]
        inner = []
        for i in xrange(6):
            d = np.zeros(6)
            d[i] = 1.0
            inner.append(vpolytope.extremum(d))
            if np.linalg.norm(inner[-1]) < 1e-3:
                inner.pop(-1)
            d[i] = -1.0
            inner.append(vpolytope.extremum(d))
            if np.linalg.norm(inner[-1]) < 1e-3:
                inner.pop(-1)
        verts = np.random.choice(range(vpolytope.vertices.shape[0]),size=numInnerSamples-12,replace=False)
        for i in verts:
            d = vpolytope.vertices[i]
            #inner.append(vpolytope.vertices[i])
            inner.append(vpolytope.extremum(d))
            if np.linalg.norm(inner[-1]) < 1e-3:
                inner.pop(-1)
        inner.append(np.zeros(vpolytope.vertices.shape[1]))
        inner = np.array(inner)
        vinner = VPolytope(inner)
        while index >= len(self.inner_bounds):
            self.inner_bounds.append(None)
        self.inner_bounds[index] = vinner
        try:
            #vhinner = HPolytope()
            #vhinner.setVPolytope(vinner)
            #self.inner_bounds = [vhinner]
            pass
        except Exception:
            print "Could not compute inner polytope, may be highly skewed"
            return

    def sampleOuterBound(self,numOuterSamples,index=2):
        assert len(self.convex_decomposition) == 1,"Can only handle convex wrench spaces for now"
        import random
        vpolytope = self.convex_decomposition[0]
        directions = []
        extrema = []
        for i in xrange(numOuterSamples):
            #d = vpolytope.vertices[random.randint(0,vpolytope.vertices.shape[0]-1)]
            d = np.random.randn(6)
            directions.append(d)
            v = vpolytope.extremum(d)
            extrema.append(np.dot(v,d))
        hinner = HPolytope(np.array(directions),np.array(extrema))
        #hinner.A  = hinner.A[12:,:]
        #hinner.b  = hinner.b[12:]
        while index >= len(self.outer_bounds):
            self.outer_bounds.append(None)
        self.outer_bounds[index] = hinner


    def setFromForceSamples(self,forcevolumes,points):
        """From a set of force volumes measured at certain points, computes the representation of this wrench
        polytope. 

        Arguments:
        - forcevolumes: a list of Volumes
        - points: a list of 3-tuples where these volumes were measured.

        Each force volume is a 3D slice through wrench space.  Assuming convexity holds between slices, at least,
        the V-representation of the wrench is a union of the transformed V-representations.

        If Wp = [I   ]
                [[p] ]
        w(f) = [f    ] = Wp*f
               [p x f] 
        for such a w on the slice, [I|0]w = f,  [0|I]w = m,  p x f = m,  => [[p]|-I] w = 0, with Wp^+ = [[p]|-I] 

        The V-representation of the transformed volume is CH({ Wp v | v in V })
        
        The H-representation is {w | A [I | 0 ] w <= b, Wp^+ w = 0}

        Let w = c1 w1 + c2 w2 be a convex combination of valid wrenches on slices, with w1 on slice 1 and w2 on slice 2.
        Then Wp1^+ w1 = 0, Wp2^+ w2 = 0, A1 [I|0] w1 <= b1, and A2 [I|0] w2 <= b2 must hold.  Also, c1,c2>=0 and c1+c2 = 1
        Is there an equivalent feasibility condition on w? (without all of w1, w2, c1, and c2 being variables...)
        """
        import time
        allconvex = True
        for V,p in zip(forcevolumes,points):
            if (len(V.convex_decomposition) > 1):
                allconvex = False
        allvertices = None
        if allconvex:
            #just make the first v polytope
            wrenches = []
            for V,p in zip(forcevolumes,points):
                for v in V.convex_hull.vertices:
                    w = np.hstack((V.convex_hull.points[v],np.cross(p,V.convex_hull.points[v])))
                    wrenches.append(w)
            wrenches = np.vstack(wrenches)
            vpolytope = VPolytope(wrenches)
            self.convex_decomposition = [vpolytope]
            #TODO: determine whether it's worth computing / using the HPolytope
            """
            t0 = time.time()
            hpolytope = HPolytope()
            hpolytope.setVPolytope(vpolytope)
            t1 = time.time()
            print "# of faces:",hpolytope.A.shape[0],"computed in time",t1-t0
            self.convex_decomposition = [hpolytope]
            """

            #compute an inner approximation with a smaller number of vertices
            self.sampleInnerBound(self.numInnerSamples)

            #self.convex_decomposition[0] = self.convex_decomposition[0].reduce()
            allvertices = wrenches
        else:
            raise NotImplementedError("Handle nonconvex shapes")

        #do a bounding box
        #more refined limit: take the 3D bounds in force space and 3D bounds in moment space, and
        #intersect these generalized cylinders
        bmin,bmax = np.min(allvertices,axis=0),np.max(allvertices,axis=0)
        print "Bounds on polytope:",zip(bmin,bmax)
        bounds = BoundsPolytope(bmin,bmax)
        Af,bf = halfplanes(scipy.spatial.ConvexHull(allvertices[:,:3]))
        Am,bm = halfplanes(scipy.spatial.ConvexHull(allvertices[:,3:]))
        A = np.block([[Af,np.zeros((Af.shape[0],Am.shape[1]))],
            [np.zeros((Am.shape[0],Af.shape[1])),Am]])
        b = np.hstack([bf,bm])
        fmpolytope = HPolytope(A,b)
        self.outer_bounds = [bounds,fmpolytope]
        self.sampleOuterBound(self.numOuterSamples,2)

    def savePickle(self,fn):
        import pickle
        f = open(fn,'w')
        pickle.dump(self,f)
        f.close()

    def loadPickle(self,fn):
        import pickle
        f = open(fn,'r')
        res = pickle.load(f)
        self.convex_decomposition = res.convex_decomposition
        self.inner_bounds = res.inner_bounds
        self.outer_bounds = res.outer_bounds
        f.close()

    def saveForceSamples(self,directory,forcevolumes,points):
        """Saves to [directory]/wrench_slices.csv and associated obj files"""
        out = open("%s/wrench_slices.csv"%(directory,),'w')
        out.write("#index,x,y,z\n")
        
        for i,(V,pt) in enumerate(zip(forcevolumes,points)):
            if len(V.convex_decomposition) > 1:
                raise NotImplementedError("Convert nonconvex geometry to Geom")
            geom = polytope.hull_to_klampt_geom(V.convex_hull)
            out.write("%d,%f,%f,%f\n"%(i+1,pt[0],pt[1],pt[2]))
            geom.saveFile("%s/wrench_slice_%d.obj"%(directory,i+1))
        out.close()

    def loadForceSamples(self,directory):
        """Loads from [directory]/wrench_slices.csv and associated obj files"""
        f = open("%s/wrench_slices.csv"%(directory,),'r')
        lines = f.readlines()
        f.close()

        pts = []
        volumes = []
        numNonConvex = 0
        for i in xrange(1,len(lines)):
            entries = lines[i].split(',')
            assert len(entries) == 4
            ind,x,y,z = int(entries[0]),float(entries[1]),float(entries[2]),float(entries[3])
            geom = Geometry3D()
            geom.loadFile("%s/wrench_slice_%d.obj"%(directory,ind))
            V = Volume()
            V.setGeom(geom)
            if len(V.convex_decomposition) > 1:
                print "Encountered a non-convex force volume on CH point",i
                numNonConvex += 1
            
            pts.append((x,y,z))
            volumes.append(V)

        print "# of non-convex force volumes:",numNonConvex
        self.setFromForceSamples(volumes,pts)

    def numVertices(self):
        n = 0
        for V in self.convex_decomposition:
            if isinstance(V,VPolytope):
                n += V.vertices.shape[0]
        return n

    def numConstraints(self):
        n = 0
        for V in self.convex_decomposition:
            if isinstance(V,HPolytope):
                n += V.A.shape[0]
        return n

    def transform(self,mat,offset=None):
        res = GeneralizedWrenchPolytope()
        res.outer_bounds =[bound.transform(mat,offset) for bound in self.outer_bounds]
        res.convex_decomposition = [bound.transform(mat,offset) for bound in self.convex_decomposition]
        return res

    def getSlice(self,p):
        """Gets the wrench space slice at the given point of application.  Return value is a 3D Volume
        whose coordinates are the force dimensions f of the wrench
          [f    ]
          [p x f].
        """
        v = Volume()
        W = np.vstack((np.eye(3),so3.matrix(so3.cross_product(p))))
        for c in self.convex_decomposition:
            if isinstance(c,(VPolytope,HPolytope)):
                cslice = c.slice(W)
            else:
                cslice = slice_hull(c,W)
            v.convex_decomposition.append(cslice)
        return v

    def test(self,N):
        import time
        print "# of vertices:",self.numVertices()
        print "# of constraints:",self.numConstraints()
        #t0 = time.time()
        #hpolytope = HPolytope()
        #hpolytope.setVPolytope(self.convex_decomposition[0])
        #t1 = time.time()
        #print "# of faces:",hpolytope.A.shape[0],"computed in time",t1-t0
        print "6D bounds:",self.outer_bounds[0].bmin,self.outer_bounds[0].bmax
        print "# of faces in fm polytope",self.outer_bounds[1].A.shape[0]
        print "# of faces in sampled polytope",self.outer_bounds[2].A.shape[0]
        print
        fouter = [0]*len(self.outer_bounds)
        touter = [0]*len(self.outer_bounds)
        sinner = [0]*len(self.inner_bounds+self.convex_decomposition)
        tinner = [0]*len(self.inner_bounds+self.convex_decomposition)
        width = (self.outer_bounds[0].bmax-self.outer_bounds[0].bmin)/3
        threp = 0
        tinorder = 0
        fallouter = 0
        for sample in xrange(N):
            w = np.multiply(np.random.randn(6),width)
            outerpassed = True
            for i,outer in enumerate(self.outer_bounds):
                t0 = time.time()
                failed = not outer.contains(w)
                t1 = time.time()
                if outerpassed:
                    tinorder += t1-t0
                if failed: fouter[i] += 1
                if failed: outerpassed = False
                touter[i] += t1-t0
            if not outerpassed:
                fallouter += 1
            innerpassed = False
            for i,inner in enumerate(self.inner_bounds+self.convex_decomposition):
                t0 = time.time()
                passed = inner.contains(w)
                t1 = time.time()
                if outerpassed and not innerpassed:
                    tinorder += t1-t0
                if passed: sinner[i] += 1
                if passed: innerpassed = True
                tinner[i] += t1-t0
            #t0 = time.time()
            #passed = hpolytope.contains(w)
            #t1 = time.time()
            #threp += t1-t0
        print "Out of",N,"tests"
        print "In-order testing time",tinorder
        print "# failed any outer test",fallouter
        print "# failed for outer tests",fouter
        print "time for outer tests",touter
        print "# passed for inner tests",sinner
        print "time for inner tests",tinner
        #print "time for inner tests, H rep",threp


if __name__ == '__main__':
    import time
    import random
    v = VPolytope()
    v.vertices = np.array([[0,0],
                           [1,0],
                           [1,2],
                           [0.5,2.5]])
    print "0.5,1.0 (should be True)",v.contains(np.array([0.5,1.0]))
    print "2.0,1.0 (should be False)",v.contains(np.array([2.0,1.0]))
    print "0.0,2.0 (should be False)",v.contains(np.array([0.0,2.0]))
    b = v.boundingPolytope()
    print "2D bounds:",b.bmin,b.bmax

    nfc = 20
    npt = 10
    h = 1
    mu = 1
    pyramid = [[0,0,0]]
    for i in range(nfc):
        theta = float(i)*math.pi*2/nfc
        pyramid.append([h*mu*math.cos(theta),h*mu*math.sin(theta),h])
    pyramid.append([0,0,h])
    vpyramid = Volume()
    vpyramid.setPoints(pyramid)
    newhull = slice_hull(vpyramid.convex_hull,np.array([[1.,0.],[0.,1.],[0.,0.]]),np.array([0.,0.,0.5]))
    if newhull:
        print [newhull.points[v] for v in newhull.vertices]
    
    #sample points from some region
    center = [0,0,0]
    radius = 0.1
    points = []
    for i in range(npt):
        theta = random.uniform(0,math.pi*2)
        #r = radius*math.sqrt(random.uniform(0,1))
        r = radius
        points.append([center[0]+r*math.cos(theta),center[1]+r*math.sin(theta),center[2]])

    wpolytope = GeneralizedWrenchPolytope()
    wpolytope.setFromForceSamples([vpyramid]*4,points)
    
    testpt = np.array([0,0,1.0,0,0,0.04])
    print "Test point",testpt
    print "Contains pt?",wpolytope.contains(testpt)
    for outer in wpolytope.outer_bounds:
        print "outer",outer.contains(testpt)
    for inner in wpolytope.convex_decomposition:
        print "inner",inner.contains(testpt)
    print
    wpolytope.test(1000)
