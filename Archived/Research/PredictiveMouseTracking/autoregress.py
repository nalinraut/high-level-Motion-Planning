from database import Database
import numpy as np
import sys

class PredictiveAutoregression:
    def __init__(self,trials,discount=1.0,eps=1e-10):
        self.trials = trials
        self.discount = discount
        n = len(self.trials[0][0])
        self.A = np.zeros((n,n))
        self.b = np.zeros(n)
        self.err = np.zeros((n,n))

        if self.discount == 1.0:
            self.predcount = max(len(trial) for trial in self.trials)
        else:
            #discount ^ c = eps
            self.predcount = int(math.log(eps)/math.log(self.discount))

    def solveNormal(self):
        """Sets A, b, and err to the result of a normal autoregression"""
        n = len(self.trials[0][0])
        mean1 = np.zeros(n)
        mean2 = np.zeros(n)
        cov = np.zeros((2*n,2*n))
        num = 0
        for trial in self.trials:
            for (xi,xn) in zip(trial[:-1],trial[1:]):
                mean1 += xi
                mean2 += xn
                num += 1
        mean1 /= num
        mean2 /= num
        for trial in self.trials:
            for (xi,xn) in zip(trial[:-1],trial[1:]):
                xin = np.hstack([xi-mean1,xn-mean2])
                cov += np.outer(xin,xin)
        cov /= num
        cov11=cov[0:n,0:n]
        cov12=cov[0:n,n:2*n]
        cov22=cov[n:2*n,n:2*n]
        cov11inv = np.linalg.inv(cov11)
        self.A = np.dot(cov12.T,cov11inv)
        self.b = mean2 - np.dot(self.A,mean1)
        self.err = cov22 - np.dot(cov12.T,np.dot(cov11inv,cov12))
        print self.A
        print self.b
        print self.err
        return

    def predictions(self,trial,A,b):
        preds = []
        for t,element in enumerate(trial):
            preds.append([element])
            for i in range(1,min(self.predcount,len(trial)-t-1)):
                preds[-1].append(np.dot(A,preds[-1][-1])+b)
        return preds

    def predictionSSE(self,trial,A,b):
        """Returns the discounted sum of squared prediction errors on the
        given trial for the prediction horizon given by self.predcount, and the
        current linear process coefficients A,b."""
        err = 0.0
        for t,element in enumerate(trial):
            pred = element
            disc = 1.0
            for i in range(1,min(self.predcount,len(trial)-t-1)):
                pred  = np.dot(A,pred)+b
                err += np.dot(pred-trial[t+i],pred-trial[t+i])*disc
                disc *= self.discount
        return err

    def predictionMSE(self,A,b):
        return sum(self.predictionSSE(trial,A,b) for trial in self.trials)/sum(len(trial) for trial in self.trials)

    def grad_diff(self,A,b,h=1e-4):
        Adiff = np.zeros(A.shape)
        bdiff = np.zeros(b.shape)
        for i in xrange(A.shape[0]):
            for j in xrange(A.shape[1]):
                A0 = A[i,j]
                A[i,j] += h
                ff = self.predictionMSE(A,b)
                A[i,j] -= 2.0*h
                fb = self.predictionMSE(A,b)
                Adiff[i,j] = (ff-fb)/(2.0*h)
                A[i,j] = A0
        for i in xrange(b.shape[0]):
            b0 = b[i]
            b[i] += h
            ff = self.predictionMSE(A,b)
            b[i] -= 2.0*h
            fb = self.predictionMSE(A,b)
            bdiff[i] = (ff-fb)/(2.0*h)
            b[i] = b0
        return (Adiff,bdiff)

    def grad(self,A,b):
        """Returns the gradient of the SSE function over the entire trials dataset
        for linear process coefficients A,b"""
        gradA = np.zeros(A.shape)
        gradb = np.zeros(b.shape)
        Ai = [np.eye(len(b))]
        for i in xrange(self.predcount):
            Ai.append(np.dot(A,Ai[-1])+np.eye(len(b)))
        num = 0.0
        for trial in self.trials:
            num += len(trial)
            preds = self.predictions(trial,A,b)
            for t,pred in enumerate(preds):
                corrected_residuals = []
                for (i,p) in reversed([y for y in enumerate(pred)]):
                    if i+1 == len(pred):
                        corrected_residuals.append(p-trial[t+i])
                    else:
                        corrected_residuals.append(p-trial[t+i] + self.discount*np.dot(A.T,corrected_residuals[-1]))
                disc = 1.0
                for r,p in zip([x for x in reversed(corrected_residuals)][1:],pred[0:-1]):
                    gradA += np.outer(r,p)*disc
                    disc *= self.discount
                disc = 1.0
                for i,p in enumerate(pred):
                    gradb += np.dot(Ai[i],p-trial[t+i])*disc
                    disc *= self.discount
        gradA *= 2.0/num
        gradb *= 2.0/num
        return (gradA,gradb)

    def solve(self,maxsteps=200,alpha0=1e-4):
        mse = self.predictionMSE(self.A,self.b)
        print "MSError",mse
        alpha = alpha0
        for step in xrange(maxsteps):
            (gA,gb) = self.grad(self.A,self.b)
            print "Gradient A",gA
            print "Gradient b",gb
            #(gA,gb) = self.grad_diff(self.A,self.b)
            #print "Differenced",gA,gb

            #now do a line search
            gradsize = max(abs(gA).max(),abs(gb).max())
            print "Beginning line search with alpha =",alpha,"gradsize",gradsize
            numsearchiters = 0
            foundreduction = False
            while(alpha*gradsize > 1e-12):
                A = self.A-alpha*gA
                b = self.b-alpha*gb
                msenew = self.predictionMSE(A,b)
                print "    alpha =",alpha,"mse",msenew
                if msenew < mse:
                    mse = msenew
                    self.A,self.b = A,b
                    foundreduction = True
                    break
                numsearchiters += 1
                alpha *= 0.5
            if not foundreduction:
                print "Converged"
                break
            if numsearchiters==0:
                print "Trying line search growth"
                #increase step size
                foundreduction = False
                while True:
                    A = self.A-alpha*1.5*gA
                    b = self.b-alpha*1.5*gb
                    msenew = self.predictionMSE(A,b)
                    print "    alpha =",alpha*1.5,"mse",msenew
                    if msenew > mse:
                        break
                    mse = msenew
                    foundreduction = True
                    alpha *= 1.5
                if foundreduction:
                    self.A-=alpha*gA
                    self.b-=alpha*gb
            print "New step size",alpha
            print "MSError",mse

    def readCoeffs(self,fn):
        #read coefficients from disk
        f = open(fn,'r')
        #read A
        entries = f.readline().split()
        while len(entries)==0:
            entries = f.readline().split()
        [m,n]=entries
        m,n=int(m),int(n)
        self.A = np.zeros((m,n))
        for i in xrange(m):
            entries = f.readline().split()
            while len(entries) == 0:
                entries = f.readline().split()
            assert(len(entries)==n)
            for j in xrange(n):
                self.A[i,j] = float(entries[j])
        #read b
        entries = f.readline().split()
        while len(entries)==0:
            entries = f.readline().split()
        assert(len(entries)==1)
        m = int(entries[0])
        self.b = np.zeros(m)
        entries = f.readline().split()
        while len(entries)==0:
            entries = f.readline().split()
        assert(len(entries)==m)
        for i in xrange(m):
            self.b[i] = float(entries[i])
        #read noise
        entries = f.readline().split()
        while len(entries)==0:
            entries = f.readline().split()
        [m,n]=entries
        m,n=int(m),int(n)
        self.err = np.zeros((m,n))
        for i in xrange(m):
            entries = f.readline().split()
            while len(entries) == 0:
                entries = f.readline().split()
            assert(len(entries)==n)
            for j in xrange(n):
                self.err[i,j] = float(entries[j])
        f.close()
        return True
    
    def writeCoeffs(self,fn):
        f = open(fn,'w')
        f.write("%d %d\n"%(self.A.shape[0],self.A.shape[1]))
        for i in xrange(self.A.shape[0]):
            for j in xrange(self.A.shape[1]):
                f.write("%f "%(self.A[i,j],))
            f.write("\n")
        f.write("\n")
        f.write("%d \n"%(self.b.shape[0],))
        for i in xrange(self.b.shape[0]):
            f.write("%f "%(self.b[i],))
        f.write("\n\n")
        f.write("%d %d\n"%(self.err.shape[0],self.err.shape[1]))
        for i in xrange(self.err.shape[0]):
            for j in xrange(self.err.shape[1]):
                f.write("%f "%(self.err[i,j],))
            f.write("\n")
        f.close()

class HiddenPredictiveAutoregression:
    """The model is as follows: x is the observed variable, y is the
    unobserved one.  Given a series of observations x[0]...x[T], the predictions
    are obtained through the following process

       y[0] = Ax[0]+b
       y[t] = Cx[t]+Dy[t-1]+e

    This class is used to estimate A, b, C, D, and e
    """
    def __init__(self,trials,observedIndices):
        self.observedIndices = observedIndices[:]
        self.discount = discount
        n = len(self.trials[0][0])
        nx = len(observedIndices)
        ny = n-ny
        unobservedIndices = [i for i in range(n) if i not in observedIndices]
        self.unobservedIndices = unobservedIndices
        self.A = np.zeros((ny,nx))
        self.b = np.zeros(ny)
        self.C = np.zeros((ny,nx))
        self.D = np.zeros((ny,ny))
        self.e = np.zeros(ny)

        self.xtrials = [[x[observedIndices] for x in trial] for trial in trials]
        self.ytrials = [[x[unobservedIndices] for x in trial] for trial in trials]

    def solveNormal(self):
        """Sets A, b, C, D, and e to the result of a normal autoregression"""
        n = len(self.trials[0][0])
        mean0 = np.zeros(n)
        mean1 = np.zeros(n)
        mean2 = np.zeros(n)
        cov0 = np.zeros((n,n))
        cov = np.zeros((2*n,2*n))
        num = 0
        for trial in self.trials:
            mean0 += trial[0]
            for (xi,xn) in zip(trial[:-1],trial[1:]):
                mean1 += xi
                mean2 += xn
                num += 1
        mean0 /= len(self.trials)
        mean1 /= num
        mean2 /= num
        for trial in self.trials:
            cov0 += np.outer(trial[0]-mean0,trial[0]-mean0)
            for (xi,xn) in zip(trial[:-1],trial[1:]):
                xin = np.hstack([xi-mean1,xn-mean2])
                cov += np.outer(xin,xin)
        cov0 /= len(self.trials)
        cov /= num
        cov11=cov[0:n,0:n]
        cov12=cov[0:n,n:2*n]
        cov22=cov[n:2*n,n:2*n]
        cov11inv = np.linalg.inv(cov11)
        Aall = np.dot(cov12.T,cov11inv)
        ball = mean2 - np.dot(Aall,mean1)
        #Do schur complement
        #[x[t]] = [A11 A12]*[x[t-1]]+[b1]
        #[y[t]]   [A21 A22] [y[t-1]] [b2]
        #A11^-1 (x[t]-b1-A12 y[t-1]) = x[t-1]
        #y[t] =  A21 x[t-1] + A22 y[t-1] + b2
        #     = A21 A11^-1 x[t] + (A22 - A21 A11^-1 A12) y[t-1] + A21 A11^-1 b2
        xind = self.observedindices
        yind = self.unobservedindices
        A11inv = np.inv(Aall[xind,xind])
        self.A = np.dot(cov0[yobs,xobs],np.inv(cov0[xobs,xobs]))
        self.b = mean0[yobs] - np.dot(self.A,mean0[xobs])
        self.C = np.dot(Aall[yind,xind],A11inv)
        self.D = Aall[yind,yind]
        self.D -= np.dot(Aall[yind,xind],np.dot(A11inv,Aall[xind,yind]))
        self.e = ball[yind] - np.dot(self.C,ball[xind])
        return

    def predictions(self,xs):
        preds = [np.dot(self.A,xs[0])+self.b]
        for x in xs[1:]:
            preds.append(np.dot(self.C,x)+np.dot(self.D,preds[-1])+self.d)
        return preds

    def predictionSSE(self,xs,ys):
        """Returns the discounted sum of squared prediction errors on the
        given trial."""
        err = 0.0
        pred = np.dot(self.A,xs[0])+self.b
        err += np.dot(pred-ys[0],pred-ys[0])
        for x,y in zip(xs,ys):
            pred = np.dot(self.C,x)+np.dot(self.D,pred)+self.d
            err += np.dot(pred-y,pred-y)            
        return err

    def predictionMSE(self):
        return sum(self.predictionSSE(xtrial,ytrial) for xtrial,ytrial in zip(self.xtrials,self.ytrials))/sum(len(trial) for trial in self.trials)

    def grad(self,A,b,C,D,e):
        """Returns the gradient of the SSE function over the entire trials dataset
        for linear process coefficients A,b,C,D,e"""
        gradA = np.zeros(A.shape)
        gradb = np.zeros(b.shape)
        gradC = np.zeros(C.shape)
        gradD = np.zeros(D.shape)
        grade = np.zeros(e.shape)
        Ai = [np.eye(len(b))]
        for i in xrange(self.predcount):
            Ai.append(np.dot(A,Ai[-1])+np.eye(len(b)))
        num = 0.0
        for xtrial,ytrial in zip(self.xtrials,self.ytrials):
            num += len(xtrial)
            preds = self.predictions(xtrial)
            corrected_residuals = []
            for t,pred in enumerate(preds):
                if i+1 == len(preds):
                    corrected_residuals.append(p-ytrial[t+i])
                else:
                    corrected_residuals.append(p-ytrial[t+i] + np.dot(D.T,corrected_residuals[-1]))
                corrected_residuals = [x for x in reversed(corrected_residuals)]
                for r,p in zip(corrected_residuals[1:],preds[0:-1]):
                    gradD += np.outer(r,p)
                for r,x in zip(corrected_residuals[1:],xtrial[1:]):
                    gradC += np.outer(r,x)

                for i,p in enumerate(pred):
                    gradb += np.dot(Ai[i],p-trial[t+i])*disc
                    disc *= self.discount
        gradA *= 2.0/len(self.trials)
        gradb *= 2.0/len(self.trials)
        gradC *= 2.0/num
        gradD *= 2.0/num
        return (gradA,gradb,gradC,gradD,grade)

    def solve(self,maxsteps=200,alpha0=1e-4):
        mse = self.predictionMSE(self.A,self.b)
        print "MSError",mse
        alpha = alpha0
        for step in xrange(maxsteps):
            (gA,gb) = self.grad(self.A,self.b)
            print "Gradient A",gA
            print "Gradient b",gb
            #(gA,gb) = self.grad_diff(self.A,self.b)
            #print "Differenced",gA,gb

            #now do a line search
            gradsize = max(abs(gA).max(),abs(gb).max())
            print "Beginning line search with alpha =",alpha,"gradsize",gradsize
            numsearchiters = 0
            foundreduction = False
            while(alpha*gradsize > 1e-12):
                A = self.A-alpha*gA
                b = self.b-alpha*gb
                msenew = self.predictionMSE(A,b)
                print "    alpha =",alpha,"mse",msenew
                if msenew < mse:
                    mse = msenew
                    self.A,self.b = A,b
                    foundreduction = True
                    break
                numsearchiters += 1
                alpha *= 0.5
            if not foundreduction:
                print "Converged"
                break
            if numsearchiters==0:
                print "Trying line search growth"
                #increase step size
                foundreduction = False
                while True:
                    A = self.A-alpha*1.5*gA
                    b = self.b-alpha*1.5*gb
                    msenew = self.predictionMSE(A,b)
                    print "    alpha =",alpha*1.5,"mse",msenew
                    if msenew > mse:
                        break
                    mse = msenew
                    foundreduction = True
                    alpha *= 1.5
                if foundreduction:
                    self.A-=alpha*gA
                    self.b-=alpha*gb
            print "New step size",alpha
            print "MSError",mse

    def readCoeffs(self,fn):
        #read coefficients from disk
        f = open(fn,'r')
        #read A
        entries = f.readline().split()
        while len(entries)==0:
            entries = f.readline().split()
        [m,n]=entries
        m,n=int(m),int(n)
        self.A = np.zeros((m,n))
        for i in xrange(m):
            entries = f.readline().split()
            while len(entries) == 0:
                entries = f.readline().split()
            assert(len(entries)==n)
            for j in xrange(n):
                self.A[i,j] = float(entries[j])
        #read b
        entries = f.readline().split()
        while len(entries)==0:
            entries = f.readline().split()
        assert(len(entries)==1)
        m = int(entries[0])
        self.b = np.zeros(m)
        entries = f.readline().split()
        while len(entries)==0:
            entries = f.readline().split()
        assert(len(entries)==m)
        for i in xrange(m):
            self.b[i] = float(entries[i])
        #read noise
        entries = f.readline().split()
        while len(entries)==0:
            entries = f.readline().split()
        [m,n]=entries
        m,n=int(m),int(n)
        self.err = np.zeros((m,n))
        for i in xrange(m):
            entries = f.readline().split()
            while len(entries) == 0:
                entries = f.readline().split()
            assert(len(entries)==n)
            for j in xrange(n):
                self.err[i,j] = float(entries[j])
        f.close()
        return True
    
    def writeCoeffs(self,fn):
        f = open(fn,'w')
        f.write("%d %d\n"%(self.A.shape[0],self.A.shape[1]))
        for i in xrange(self.A.shape[0]):
            for j in xrange(self.A.shape[1]):
                f.write("%f "%(self.A[i,j],))
            f.write("\n")
        f.write("\n")
        f.write("%d \n"%(self.b.shape[0],))
        for i in xrange(self.b.shape[0]):
            f.write("%f "%(self.b[i],))
        f.write("\n\n")
        f.write("%d %d\n"%(self.err.shape[0],self.err.shape[1]))
        for i in xrange(self.err.shape[0]):
            for j in xrange(self.err.shape[1]):
                f.write("%f "%(self.err[i,j],))
            f.write("\n")
        f.close()


def predictive_autoregression(trialdblist,discount=1.0,startfn=None):
    trials = []
    for trial in trialdblist:
        trials.append([])
        for e in trial.entries:
            trials[-1].append(np.array([float(v) for v in e]))
    regression = PredictiveAutoregression(trials,discount)
    regression.predcount = 20
    if startfn==None:
        regression.solveNormal()
    else:
        print "Reading coefficients from",startfn
        regression.readCoeffs(startfn)
        
    #for trial in trials:
    #    print "MSE:",regression.predictionSSE(trial,regression.A,regression.b)/len(trial)
    regression.solve()
    if startfn != None:
        print "Saving coefficients to",startfn
        regression.writeCoeffs(startfn)
    return (regression.A,regression.b,regression.err)
    

fn = sys.argv[1]
db = Database()
db.readCSV(fn)

trialkey = 'trial'
trials = db.split(trialkey)
for t in trials:
    t.delete_key(trialkey)

#autoregression with predictive component
startfn = sys.argv[2] if len(sys.argv)>=3 else None
(A,b,err) = predictive_autoregression(trials,startfn=startfn)
print A
print b
print err
