import csv
import copy
import re
import math

class Database:
    """
    A simple associative database.

    Conditions are given by a string of the form 'expr({arg1},...,{argn})'
    where the bracketed arguments are database keys.
    
    Transform operations are given by expressions of the form
    'name = expr({arg1},...,{argn})' where the bracketed arguments are
    database keys.
    """
    
    def __init__(self,*args):
        self.keys = []
        self.entries = []
        if len(args) != 0:
            self.keys = args[0].keys[:]
            self.entries = copy.deepcopy(args[0].entries)
        
    def readCSV(self,fn):
        self.keys = []
        self.entries = []
        f = open(fn,'r')
        reader = csv.reader(f)
        for lineno,line in enumerate(reader):
            if lineno == 0:
                self.keys = line[:]
            else:
                self.entries.append(line)
        f.close()
        
    def writeCSV(self,fn):
        f = open(fn,'w')
        writer = csv.writer(f)
        writer.writerow(self.keys)
        for pt in self.entries:
            writer.writerow(pt)
        f.close()
    def get(self,index,key):
        keyindex = self.keys.index(key)
        return self.entries[index][keyindex]
    def set(self,index,key,v):
        keyindex = self.keys.index(key)
        self.entries[index][keyindex] = v
    def cast(self,key=None):
        """Casts entries of the given key to the most appropriate type"""
        if key == None:
            types = self.type(None)
            for e in self.entries:
                for (i,t) in enumerate(types):
                    e[i] = t(e[i])
        else:
            index = self.keys.indx(key)
            type = self.type(key)
            for e in self.entries:
                e[index] = type(e[index])
    def type(self,key=None):
        """Finds the most appropriate type for the given key"""
        if key == None:
            #return a list of types
            return [self.type(k) for k in self.keys]
        else:
            index = self.keys.index(key)
            vals = [e[index] for e in self.entries]
            ints = 0
            floats = 0
            for v in vals:
                if isinstance(v,int):
                    ints += 1
                elif isinstance(v,float):
                    floats += 1
                else:
                    try:
                        temp = int(v)
                        ints += 1
                    except:
                        try:
                            temp = float(v)
                            floats += 1
                        except:
                            return str
            if ints == len(vals):
                return int
            elif ints + floats == len(vals):
                return float
    def subst(self,statement,prefix):
        items = re.split("(\{.+?\})",statement)
        res = ""
        for r in items:
            if len(r)==0: continue
            if r[0]=='{' and r[-1]=='}':
                k = r[1:-1]
                if k not in self.keys:
                    raise KeyError('Key "%s" not in database'%(str(k),))
                res = res+prefix+"["+str(self.keys.index(k))+"]"
            else:
                res = res+r
        return res
    def delete_key(self,*argv):
        """Deletes all of the given keys"""
        for v in argv:
            if v not in self.keys:
                raise KeyError("Key "+str(v)+" not in database")
            ind = self.keys.index(v)
            self.keys.pop(ind)
            for e in self.entries:
                e.pop(ind)
    def delete_if(self,cond,data={}):
        cond = self.subst(cond,'_e_')
        newList = []
        for e in enumerate(self.entries):
            evals = [t(v) for t,v in zip(types,e)]
            locals = {'_e_':evals}
            locals.update(data)
            if not eval(cond,globals(),locals):
                newList.append(e)
        self.entries = newList
    def get_if(self,cond,data={}):
        """Returns a subset of this database such that cond(entry) is true."""
        cond = self.subst(cond,'_e_')
        db = Database()
        db.keys = self.keys[:]
        for e in enumerate(self.entries):
            evals = [t(v) for t,v in zip(types,e)]
            locals = {'_e_':evals}
            locals.update(data)
            if eval(cond,globals(),locals):
                db.entries.append(e)
        return db
    def shuffle_keys(self,keyorder):
        """Rearranges the order of the keys given the new key order.
        This can also be used to select a subset of the keys."""
        indices = [self.keys.index(k) for k in keyorder]
        self.keys = keyorder[:]
        for i,e in enumerate(self.entries):
            self.entries[i] = [e[ind] for ind in indices]
    def join(self,db):
        """A simple join of the two databases formed just by concatenating all
        entries of db onto this one."""
        assert len(self.entries) == len(db.entries)
        self.keys = self.keys + db.keys
        for (e1,e2) in zip(self.entries,db.entries):
            e1.extend(e2)
    def process(self,assignments,data={}):
        """Joins a transform(assignments) into self by adding the transformed
        items onto each entry in the self database.  If the transformed
        item has the same name as a key in self, then that entry's attribute
        according to that key is replaced."""
        db = None
        if hasattr(assignments,'__iter__'):
            db=self.transform(assignments,data)
        else:
            db=self.transform([assignments],data)
        indices = []
        for k in db.keys:
            if k in self.keys:
                indices.append(self.keys.index(k))
            else:
                self.keys.append(k)
                indices.append(len(self.keys)-1)
        for e1,e2 in zip(self.entries,db.entries):
            while len(e1) < len(self.keys):
                e1.append(None)
            for (i,v) in zip(indices,e2):
                e1[i] = v
            assert len(e1)==len(self.keys)
    def split(self,key):
        """Splits the db into separate databases given the values of the key"""
        if self.type(key) == float:
            raise ValueError("Can't split on floating point attributes")
        index = self.keys.index(key)
        vals = dict()
        for e in self.entries:
            vals.setdefault(e[index],[]).append(e)
        res = [None]*len(vals)
        for (i,v) in enumerate(vals.itervalues()):
            res[i] = Database()
            res[i].keys = self.keys[:]
            res[i].entries = copy.deepcopy(v)
        return res
    def transform(self,assignments,data={}):
        """Assignments can be a command of the form 'name = expr(items)'
        where name is the name of the new entry, and expr(keys) is some
        python expression in which keys can be referenced using the string
        {key}.  E.g. 'foo = -{bar}*3'.  Multiple assignments can also be
        given as a list"""
        db = Database()
        commands = []
        for assignment in assignments:
            rhs,lhs = assignment.split('=',1)
            rhs = rhs.strip()
            db.keys.append(rhs)
            lhs = self.subst(lhs,'_e_')
            commands.append(lhs)
        types = self.type()
        for e in self.entries:
            evals = [t(v) for t,v in zip(types,e)]
            locals = {'_e_':evals}
            locals.update(data)
            newentry = []
            for cmd in commands:
                try:
                    newentry.append(eval(cmd,globals(),locals))
                except Exception:
                    print "Unable to evaluate",cmd,"in env",locals
                    newentry.append(None)
            db.entries.append(newentry)
        return db
    def append(self,db):
        """Appends the entries of a database onto this one.  Does a little
        more error checking than would appending the entries."""
        if len(self.keys) == 0:
            self.keys = db.keys[:]
        assert (self.keys == db.keys)
        self.entries = self.entries + [e[:] for e in db.entries]
    def range(self,entry=None):
        if entry == None:
            #return a list of ranges
            return [self.range(k) for k in self.keys]
        else:
            t = self.type(entry)
            if t == str:
                return None
            index = self.keys.index(entry)
            return min(e[index] for e in self.entries),max(e[index] for e in self.entries)
    def mean(self,entry=None):
        if entry == None:
            #return a list of means
            return [self.mean(k) for k in self.keys]
        else:
            t = self.type(entry)
            if t == str:
                return None
            index = self.keys.index(entry)
            return sum([float(e[index]) for e in self.entries])/len(self.entries)
    def stdev(self,entry=None):
        if entry == None:
            #return a list of stdevs
            return [self.stdev(k) for k in self.keys]
        else:
            t = self.type(entry)
            if t == str:
                return None
            index = self.keys.index(entry)
            mean = self.mean(entry)
            ss = sum([pow(float(e[index])-mean,2) for e in self.entries])
            return math.sqrt(ss/len(self.entries))
    def whiten(self,entry=None):
        if entry == None:
            for k in self.keys:
                self.whiten(k)
        else:
            t = self.type(entry)
            if t == str:
                return
            index = self.keys.index(entry)
            mean = self.mean(entry)
            stdev = self.stdev(entry)
            if stdev == 0.0: return
            for e in self.entries:
                e[index] = (float(e[index])-mean)/stdev
