from database import Database
import re

class TimeSeriesDatabase(Database):
    """
    A time series database.  Each trial in the database is identified by a
    unique trial key (self.trialkey) and is stored in sequential order.
    Each trial is also assumed stored as a contiguous block in time-
    increasing order.

    Adds to the capabilities of the basic database by enabling time-sensitive
    history operations.  Each statement of the form {arg}[k] where k is
    integer is interpreted as looking forward k steps.  Negative k looks
    backward -k steps.

    Each expression of the form {arg}[a:b] generates a moving average of all
    indices from a:b (inclusive of b).

    Boundary conditions simply duplicate the first/last entry in a time series.
    """

    def __init__(self,trialkey='trial'):
        self.trialkey = trialkey
        Database.__init__(self)
    def getTrial(self,trialid):
        return self.get_if("{%s}==%d"%(self.trialkey,trialid))
    def getTrials(self):
        return self.split(self.trialkey)
    def addTrial(self,trial):
        if len(self.keys)==0:
            #initialize my keys
            if self.trialkey not in trial.keys:
                self.keys = [self.trialkey]+trial.keys
            else:
                self.keys = trial.keys[:]
        trialindex = self.keys.index(self.trialkey)
        trialcount = 1
        #reshuffle the trial if necessary
        mod = Database(trial)
        if self.trialkey not in trial.keys:
            mod.keys.append(self.trialkey)
            for e in mod.entries:
                e.append(trialcount)
        mod.shuffle_keys(self.keys)
        if len(self.entries)!=0:
            trialrange = self.range(self.trialkey)
            trialcount = trialrange[1]+1
        for e in mod.entries:
            e[trialindex]=trialcount
        self.append(mod)
        return
    def addTrials(self,trials):
        if len(trials)==0: return
        if len(self.keys)==0:
            #initialize my keys
            if self.trialkey not in trials[0].keys:
                self.keys = [self.trialkey]+trials[0].keys
            else:
                self.keys = trials[0].keys[:]
        trialindex = self.keys.index(self.trialkey)
        trialcount = 1
        if len(self.entries)!=0:
            trialrange = self.range(self.trialkey)
            trialcount = trialrange[1]+1
        #reshuffle the trial if necessary
        for trial in trials:
            mod = Database(trial)
            if self.trialkey not in trial.keys:
                mod.keys.append(self.trialkey)
            mod.shuffle_keys(self.keys)
            for e in mod.entries:
                e[trialindex]=trialcount
            trialcount += 1
            self.append(mod)
        return
    def offset_time(self,index,offset):
        """Finds the index of the same trial as entry[index] but offset
        in time by the given offset.  Handles boundary conditions by clamping.
        """
        trialindex = self.keys.index(self.trialkey)
        trialid = self.entries[index][trialindex]
        if index+offset >= 0 and index+offset < len(self.entries):
            if self.entries[index+offset][trialindex] == trialid:
                return index+offset
        if offset > 0:
            #find the last entry with the given trial ID
            lastok = index
            for i in range(index+1,min(index+offset+1,len(self.entries))):
                if self.entries[i][trialindex] != trialid:
                    break
                lastok = i
            return lastok
        else:
            assert offset < 0
            #find the last entry with the given trial ID
            lastok = index
            for i in range(1,min(-offset+1,index+1)):
                if self.entries[index-i][trialindex] != trialid:
                    break
                lastok -= 1
            return lastok

    def slice_time(self,index,omin,omax):
        """Returns the slice of entries with the same trial as entry[index]
        but offset in time by the range [omin,omax] (inclusive).
        """
        return (self.offset_time(index,omin),self.offset_time(index,omax))

    def subst_time(self,index,statement):
        """Performs variable substitution in the given statement with
        knowledge of the trial index."""
        items = re.split("(\{.+?\})",statement)
        res = ""
        skip = False;
        for r,decorator in zip(items,items[1:]+["x"]):
            if len(r)==0: continue
            if skip:
                #flag to indicate r is a decorator
                skip=False
                continue
            if r[0]=='{' and r[-1]=='}':
                k = r[1:-1]
                if k not in self.keys:
                    raise KeyError('Key "%s" not in database'%(str(k),))
                keyindex = self.keys.index(k)
                if len(decorator)>2 and decorator[0]=='[' and decorator.find(']')>=1:
                    skip = True
                    rbracketpos = decorator.find(']')
                    #it's a time-based index, see if it's a single entry or
                    #a range
                    ind = decorator[1:rbracketpos]
                    try:
                        ofs = int(ind)
                        ind = self.offset_time(index,ofs)
                        res = res+str(self.entries[ind][keyindex])+decorator[rbracketpos+1:]
                    except ValueError as e:
                        print e
                        vals=ind.split(':')
                        if len(vals)==2:
                            try:
                                a,b=int(vals[0]),int(vals[1])
                                aind,bind = self.slice_time(index,a,b)
                                #take an average of the slice [aind:bind]
                                #TODO: boundary conditions?
                                avg = sum(float(self.entries[i][keyindex]) for i in xrange(aind,bind+1))/(bind-aind+1)
                                res = res+str(avg)+decorator[rbracketpos+1:]
                            except ValueError:
                                raise ValueError("Invalid slice index '%s'"%(ind,))
                        else:
                            raise ValueError("Invalid slice index '%s'"%(ind,))
                else:
                    #normal reference
                    res = res+str(self.entries[index][keyindex])
            else:
                res = res+r
        return res
    def transform(self,assignments,data={}):
        db = TimeSeriesDatabase(self.trialkey)
        commands = []
        for assignment in assignments:
            rhs,lhs = assignment.split('=',1)
            rhs = rhs.strip()
            db.keys.append(rhs)
            commands.append(lhs)
        for i,e in enumerate(self.entries):
            newentry = []
            for cmd in commands:
                substcmd = self.subst_time(i,cmd)
                try:
                    newentry.append(eval(substcmd,globals(),data))
                except Exception:
                    print "Unable to evaluate command",substcmd,", subst from",cmd
                    newentry.append(None)
            db.entries.append(newentry)
        return db
