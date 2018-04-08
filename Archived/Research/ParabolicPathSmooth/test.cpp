#include "ParabolicRamp.h"
#include "DynamicPath.h"
#include <stdio.h>
#include <set>
using namespace std;
using namespace ParabolicRamp;

void WriteRamp(const char* fn,const ParabolicRamp1D& ramp,Real h=0.01)
{
  FILE* f=fopen(fn,"w");
  Real t=0;
  for(t=0;t<=ramp.ttotal;t+=h) {
    fprintf(f,"%g %g\n",t,ramp.Evaluate(t));
  }
  fclose(f);
}

void WriteRamp(const char* fn,const ParabolicRampND& ramp,Real h=0.01)
{
  FILE* f=fopen(fn,"w");
  Real t=0;
  Vector v;
  for(t=0;t<=ramp.endTime;t+=h) {
    ramp.Evaluate(t,v);
    for(size_t i=0;i<v.size();i++)
      fprintf(f,"%g ",v[i]);
    fprintf(f,"\n");
  }
  fclose(f);
}

void WriteRamp(const char* fn1,const char* fn2,const char* fn3,const ParabolicRamp1D& ramp,Real h=0.01)
{
  FILE* f=fopen(fn1,"w");
  Real t=0;
  for(t=0;t<ramp.tswitch1;t+=h) {
    fprintf(f,"%g %g\n",t,ramp.Evaluate(t));
  }
  t = ramp.tswitch1;
  fprintf(f,"%g %g\n",t,ramp.Evaluate(t));
  fclose(f);
  
  f=fopen(fn2,"w");
  for(;t<ramp.tswitch2;t+=h) {
    fprintf(f,"%g %g\n",t,ramp.Evaluate(t));
  }
  t = ramp.tswitch2;
  fprintf(f,"%g %g\n",t,ramp.Evaluate(t));
  fclose(f);

  f=fopen(fn3,"w");
  for(;t<ramp.ttotal;t+=h) {
    fprintf(f,"%g %g\n",t,ramp.Evaluate(t));
  }
  t = ramp.ttotal;
  fprintf(f,"%g %g\n",t,ramp.Evaluate(t));
  fclose(f);
}

void WriteInflections(const char* fn,const ParabolicRampND& ramp,int index=-1)
{
  FILE* f=fopen(fn,"w");
  set<Real> pts;
  if(index < 0) {
    for(size_t i=0;i<ramp.ramps.size();i++) {
      if(ramp.ramps[i].tswitch1 > 0)
	pts.insert(ramp.ramps[i].tswitch1);
      if(ramp.ramps[i].tswitch2 > ramp.ramps[i].tswitch1)
	pts.insert(ramp.ramps[i].tswitch2);
    }
  }
  else {  //just write the inflections for that path
    if(ramp.ramps[index].tswitch1 > 0)
      pts.insert(ramp.ramps[index].tswitch1);
    if(ramp.ramps[index].tswitch2 > ramp.ramps[index].tswitch1)
      pts.insert(ramp.ramps[index].tswitch2);
  }

  Vector v;
  for(set<Real>::iterator t=pts.begin();t!=pts.end();t++) {
    ramp.Evaluate(*t,v);
    for(size_t i=0;i<v.size();i++)
      fprintf(f,"%g ",v[i]);
    fprintf(f,"\n");
  }
  fclose(f);
}

void WriteEndpoints(const char* fn,const ParabolicRampND& ramp)
{
  FILE* f = fopen(fn,"w");
  for(size_t i=0;i<ramp.x0.size();i++)
    fprintf(f,"%g ",ramp.x0[i]);
  fprintf(f,"\n");
  for(size_t i=0;i<ramp.x1.size();i++)
    fprintf(f,"%g ",ramp.x1[i]);
  fprintf(f,"\n");
  fclose(f);
}


void GetRampLimits(const ParabolicRamp1D& ramp,Real& bmin,Real& bmax)
{
  bmin = Min(ramp.x0,ramp.x1);
  bmax = Max(ramp.x0,ramp.x1);
  Real tflip1 = -ramp.dx0/ramp.a1;
  if(tflip1 > ramp.tswitch1) tflip1 = 0;
  Real tflip2 = ramp.ttotal-ramp.dx1/ramp.a2;
  if(tflip2 < ramp.tswitch2) tflip2 = 0;
  if(0 < tflip1 && ramp.ttotal > tflip1) {
    Real xflip = ramp.Evaluate(tflip1);
    if(xflip < bmin) bmin = xflip;
    else if(xflip > bmax) bmax = xflip;
  }
  if(0 < tflip2 && ramp.ttotal > tflip2) {
    Real xflip = ramp.Evaluate(tflip2);
    if(xflip < bmin) bmin = xflip;
    else if(xflip > bmax) bmax = xflip;
  }
}

void WriteRampGnuplot(const char* fn,const ParabolicRamp1D& ramp)
{
  char buf[256],bufa[256],bufb[256],bufc[256],bufp[256],bufi[256];
  sprintf(buf,"%s.gnuplot",fn);
  sprintf(bufa,"%sa.txt",fn);
  sprintf(bufb,"%sb.txt",fn);
  sprintf(bufc,"%sc.txt",fn);
  sprintf(bufp,"%sp.txt",fn);
  sprintf(bufi,"%si.txt",fn);
  WriteRamp(bufa,bufb,bufc,ramp);

  FILE* f;
  f=fopen(bufp,"w");
  fprintf(f,"%g %g\n",0.0,ramp.x0);
  fprintf(f,"%g %g\n",ramp.ttotal,ramp.x1);
  fclose(f);

  f=fopen(bufi,"w");
  fprintf(f,"%g %g\n",ramp.tswitch1,ramp.Evaluate(ramp.tswitch1));
  fprintf(f,"%g %g\n",ramp.tswitch2,ramp.Evaluate(ramp.tswitch2));
  fclose(f);

  //write gnuplot file
  f=fopen(buf,"w");
  fprintf(f,"set term pdf enh color dashed linewidth 2 dl 1 size 1.5, 1\n");
  fprintf(f,"set output '%s.pdf'\n",fn);
  fprintf(f,"set xzeroaxis\n");

  fprintf(f,"set key off\n");
  Real len = Min(1,ramp.ttotal*0.2);
  fprintf(f,"set arrow from %g,%g to %g,%g\n",0.0,ramp.x0,len,ramp.x0+ramp.dx0*len);
  fprintf(f,"set arrow from %g,%g to %g,%g\n",ramp.ttotal,ramp.x1,ramp.ttotal+len,ramp.x1+ramp.dx1*len);

  //set a box around it
  Real xmin = -ramp.ttotal*0.05;
  Real xmax = ramp.ttotal*1.05 + len*1.1;
  Real ymin,ymax;
  GetRampLimits(ramp,ymin,ymax);
  Real ymin2 = ymin - 0.05*(ymax-ymin);
  Real ymax2 = ymax + 0.05*(ymax-ymin);
  ymin2 = Min(ymin2,ramp.x1 + ramp.dx1*len*1.1);
  ymin2 = Min(ymin2,ramp.x0 + ramp.dx0*len*1.1);
  ymax2 = Max(ymax2,ramp.x1 + ramp.dx1*len*1.1);
  ymax2 = Max(ymax2,ramp.x0 + ramp.dx0*len*1.1);
  ymin = ymin2;
  ymax = ymax2;
  fprintf(f,"set xrange [%g:%g]\n",xmin,xmax);
  fprintf(f,"set yrange [%g:%g]\n",ymin,ymax);
  fprintf(f,"plot '%s' with lines, '%s' with lines, '%s' with lines, '%s' with points lt 0 pt 4, '%s' with points lt -1 pt 7\n",bufb,bufa,bufc,bufi,bufp);
  fclose(f);
  char cmd[512];
  sprintf(cmd,"gnuplot %s",buf);
  system(cmd);
  sprintf(cmd,"rm %s %s %s %s %s",buf,bufa,bufb,bufc);
  system(cmd);
  sprintf(cmd,"rm %s %s",bufp,bufi);
  system(cmd);
}

void WriteRampGnuplot(const char* fn,const ParabolicRampND& ramp)
{
  char buf[256],bufpath[256],bufinfx[256],bufinfy[256],bufp[256];
  sprintf(buf,"%s.gnuplot",fn);
  sprintf(bufpath,"%sc.txt",fn);
  sprintf(bufinfx,"%six.txt",fn);
  sprintf(bufinfy,"%siy.txt",fn);
  sprintf(bufp,"%sp.txt",fn);
  WriteRamp(bufpath,ramp);
  WriteInflections(bufinfx,ramp,0);
  WriteInflections(bufinfy,ramp,1);
  WriteEndpoints(bufp,ramp);

  FILE* f=fopen(buf,"w");
  fprintf(f,"set term pdf enh color dashed linewidth 2 dl 1 size 1.5, 1\n");
  fprintf(f,"set output '%s.pdf'\n",fn);

  fprintf(f,"set key off\n");
  Real len = 1;
  fprintf(f,"set arrow from %g,%g to %g,%g\n",ramp.x0[0],ramp.x0[1],ramp.x0[0]+ramp.dx0[0]*len,ramp.x0[1]+ramp.dx0[1]*len);
  fprintf(f,"set arrow from %g,%g to %g,%g\n",ramp.x1[0],ramp.x1[1],ramp.x1[0]+ramp.dx1[0]*len,ramp.x1[1]+ramp.dx1[1]*len);

  Real xmin,xmax,ymin,ymax;
  GetRampLimits(ramp.ramps[0],xmin,xmax);
  GetRampLimits(ramp.ramps[1],ymin,ymax);

  Real xmin2 = xmin - 0.05*(xmax-xmin);
  Real xmax2 = xmax + 0.05*(xmax-xmin);
  Real ymin2 = ymin - 0.05*(ymax-ymin);
  Real ymax2 = ymax + 0.05*(ymax-ymin);
  xmin2 = Min(xmin2,ramp.x1[0] + ramp.dx1[0]*len*1.1);
  xmin2 = Min(xmin2,ramp.x0[0] + ramp.dx0[0]*len*1.1);
  xmax2 = Max(xmax2,ramp.x1[0] + ramp.dx1[0]*len*1.1);
  xmax2 = Max(xmax2,ramp.x0[0] + ramp.dx0[0]*len*1.1);
  ymin2 = Min(ymin2,ramp.x1[1] + ramp.dx1[1]*len*1.1);
  ymin2 = Min(ymin2,ramp.x0[1] + ramp.dx0[1]*len*1.1);
  ymax2 = Max(ymax2,ramp.x1[1] + ramp.dx1[1]*len*1.1);
  ymax2 = Max(ymax2,ramp.x0[1] + ramp.dx0[1]*len*1.1);
  xmin = xmin2;
  xmax = xmax2;
  ymin = ymin2;
  ymax = ymax2;
  xmin = -0.5;
  xmax = 3.2;
  ymin = -0.1;
  ymax = 2;
  fprintf(f,"set xrange [%g:%g]\n",xmin,xmax);
  fprintf(f,"set yrange [%g:%g]\n",ymin,ymax);
  fprintf(f,"set parametric\n");
  fprintf(f,"plot '%s' with lines, '%s' with points lt 0 pt 4, '%s' with points lt 2 pt 12, '%s' with points lt -1 pt 7\n",bufpath,bufinfx,bufinfy,bufp);
  fclose(f);
  char cmd[512];
  sprintf(cmd,"gnuplot %s",buf);
  system(cmd);
  sprintf(cmd,"rm %s %s %s %s %s",buf,bufpath,bufinfx,bufinfy,bufp);
  //system(cmd);
}

int main(int argc,char** argv)
{
  /*
  ParabolicRamp1D ramp;
  ramp.x0 = 0;
  ramp.dx0 = 0;
  ramp.x1 = 1;
  ramp.dx1 = 0;
  ramp.SolveMinTime(1,1); 
  WriteRampGnuplot("startstop_1",ramp);

  ramp.x1 = 3;
  ramp.SolveMinTime(1,1); 
  WriteRampGnuplot("startstop_3",ramp);

  ramp.x0 = 0;
  ramp.dx0 = 1;
  ramp.x1 = 0;
  ramp.dx1 = 0;
  ramp.SolveMinTime(1,1); 
  WriteRampGnuplot("ramp_0_1_0_0",ramp);

  ramp.x0 = 0;
  ramp.dx0 = 1;
  ramp.x1 = -0.5;
  ramp.dx1 = 1;
  ramp.SolveMinTime(1,1); 
  WriteRampGnuplot("ramp_0_1_-0.5_1",ramp); 
  */

  ParabolicRampND ramp;
  Vector vmax(2,1.0);
  Vector amax(2,1.0);
  ramp.x0.resize(2,0.0);
  ramp.x1.resize(2,0.0);
  ramp.dx0.resize(2,0.0);
  ramp.dx1.resize(2,0.0);
  ramp.x0[0] = 0;
  ramp.dx0[0] = 1;
  ramp.x1[0] = 0;
  ramp.dx1[0] = 1;
  ramp.x1[1] = 1;
  //ramp.SolveMinTime(vmax,amax);
  ramp.SolveMinTime(vmax,amax);
  WriteRampGnuplot("ramp2d_test",ramp); 


  /*
  ParabolicRampND ramp;
  Vector vmax(2,1.0);
  Vector amax(2,1.0);
  ramp.x0.resize(2,0.0);
  ramp.x1.resize(2,0.0);
  ramp.dx0.resize(2,0.0);
  ramp.dx1.resize(2,0.0);
  ramp.x1[0] = 3;
  ramp.x1[1] = 1;
  //ramp.SolveMinTime(vmax,amax);
  ramp.SolveMinTimeLinear(vmax,amax);
  WriteRampGnuplot("ramp2d_3_1",ramp); 

  ramp.dx1[0] = 0;
  ramp.dx1[1] = -1;
  ramp.SolveMinTime(vmax,amax);
  WriteRampGnuplot("ramp2d_3_1_down",ramp); 

  ramp.dx0[0] = -0.5;
  ramp.dx0[1] = 0.5;
  ramp.SolveMinTime(vmax,amax);
  WriteRampGnuplot("ramp2d_3_1_left_down",ramp); 

  ramp.dx0[0] = 1;
  ramp.dx0[1] = 0;
  ramp.SolveMinTime(vmax,amax);
  WriteRampGnuplot("ramp2d_3_1_right_down",ramp); 
  */
  return 0;
}
