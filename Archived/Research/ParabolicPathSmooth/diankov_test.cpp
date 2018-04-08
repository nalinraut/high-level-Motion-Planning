#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstdlib>
#include <assert.h>
using namespace std;

typedef double Real;
Real Sqr(Real x ) {
    return x*x;
}

Real CalcMinAccel(Real endTime,Real v, Real x0, Real dx0, Real x1, Real dx1)
{
    Real den=endTime*v - (x1-x0);
    Real a = (Sqr(v) - v*(dx0+dx1) + 0.5*(Sqr(dx0)+Sqr(dx1)))/den;
    return a;
}

Real CalcTotalTime(Real a,Real v, Real x0, Real dx0, Real x1, Real dx1)
{
    Real t1 = (v-dx0)/a;
    Real t2mT = (dx1-v)/a;
    Real y1 = 0.5*(Sqr(v) - Sqr(dx0))/a + x0;
    Real y2 = 0.5*(Sqr(dx1) - Sqr(v))/a + x1;
    Real t2mt1 = (y2-y1)/v;
    //if(t1 < 0 || t2mT > 0 || t2mt1 < 0) return -1;
    //if(!IsFinite(t1) || !IsFinite(t2mT)) return -1;
    return t1 + t2mt1 - t2mT;
}

int main()
{
    Real x0= 0.20405512898043426,
         dx0= -1.0471999999999999,
         x1= -0.27105963545846945,
         dx1= -1.0471999972636687,
         vmax= 1.0471999999999999,
         endTime= 0.45370012003319604;

    Real a1 = CalcMinAccel(endTime,vmax,x0,dx0,x1,dx1);
    Real a2 = CalcMinAccel(endTime,-vmax,x0,dx0,x1,dx1);
    Real a = 1e300;
    Real v = 1e300;
    if(fabs(a1) < a) {
        a = a1;
        v = vmax;
    }
    if(fabs(a2) < a) {
        a = a2;
        v = -vmax;
    }
    if( fabs(a) == 0 ) {
        cout << "a==0" << endl;
        return 0;
    }
    Real ttotal = CalcTotalTime(a,v,x0,dx0,x1,dx1);
    cout << std::setprecision(18) << ttotal-endTime << endl;
}
