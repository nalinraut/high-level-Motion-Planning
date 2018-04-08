#ifndef INCREMENTAL_AVERAGER_H
#define INCREMENTAL_AVERAGER_H

//a class that incrementally averages some value using only the 
//interpolate(a,b,u,out) function
template <class T>
struct Averager
{
  inline Averager() : weight(0) {}
  inline Averager(const T& t,double w=1.0) : value(t),weight(w) {}

  inline void set(const T& t,double w=1.0) {
    value = t;
    weight = w;
  }

  inline void inc(const T& t) {
    T temp=value;
    interpolate(temp,t,1.0/(weight+1.0),value);
    weight += 1.0;
  }

  inline void inc(const Averager<T>& t) {
    T temp=value;
    interpolate(temp,t.value,t.weight/(weight+t.weight),value);
    weight += t.weight;
  }

  inline void add(const Averager<T>& a,const Averager<T>& b) {
    interpolate(a.value,b.value,b.weight/(a.weight+b.weight),value);
    weight = a.weight+b.weight;
  }

  template <class InterpFunc>
  inline void inc(const T& t,InterpFunc& interpolate) {
    T temp=value;
    interpolate(temp,t,1.0/(weight+1.0),value);
    weight += 1.0;
  }

  template <class InterpFunc>
  inline void inc(const Averager<T>& t,InterpFunc& interpolate) {
    T temp=value;
    interpolate(temp,t.value,t.weight/(weight+t.weight),value);
    weight += t.weight;
  }

  template <class InterpFunc>
  inline void add(const Averager<T>& a,const Averager<T>& b,InterpFunc& interpolate) {
    interpolate(a.value,b.value,b.weight/(a.weight+b.weight),value);
    weight = a.weight+b.weight;
  }

  T value;
  double weight;
};

#endif
