#ifndef SPLINE_POLYNOMIAL_H
#define SPLINE_POLYNOMIAL_H

#include <vector>
#include <iostream>
#include <assert.h>

namespace Spline {

/** @brief A simple polynomial class, p(x) = sum_{i=0}^n coef[i]*x^i.
 */
template <class T=double>
class Polynomial
{
 public:
  std::vector<T> coef;

  Polynomial() {}
  Polynomial(T c) {
    coef.resize(1);
    coef[0] = c;
  }
  Polynomial(const std::vector<T>& _coef)
    :coef(_coef)
    {}
  Polynomial(const Polynomial<T>& p)
    :coef(p.coef)
    {}

  //cast constructor
  template <class T2>
  Polynomial(const Polynomial<T2>& p)
  {
    coef.resize(p.coef.size());
    for(size_t i=0;i<coef.size();i++)
      coef[i] = T(p.coef[i]);
  }

  void Resize(size_t s) { coef.resize(s,T(0)); }
  size_t Size() const { return coef.size(); }

  int Degree() const
  {
    for(size_t i=0;i<coef.size();i++)
      if(coef[coef.size()-1-i]!=T(0))
	return (int)coef.size()-1-i;
    return 0;
  }

  void SetCoef(size_t i,T value)
  {
    if(i >= coef.size())
      Resize(i+1);
    coef[i] = value;
  }

  T GetCoef(size_t i) const
  {
    if(i >= coef.size()) return T(0);
    return coef[i];
  }
 
  std::ostream& operator << (std::ostream& out) const
  {
    for(size_t i=0;i<coef.size();i++) {
      out << coef[coef.size()-1-i] << "x^" << coef.size()-1-i;
      if(i+1 != coef.size())
	out << " + ";
    }
    return out;
  }

  T Evaluate (T x) const
  {
    //horner's rule
    T s = 0;
    for (size_t i=0;i<coef.size();i++)
      s = coef[coef.size()-i-1] + ( x * s );
    return s;
  }

  Polynomial<T> Evaluate(const Polynomial<T>& x) const 
  {
    Polynomial<T> s(0);
    for (size_t i=0;i<coef.size();i++)
      s = Polynomial<T>(coef[coef.size()-i-1]) + ( x * s );
    return s;
  }

  T Derivative (T x) const
  {
    T s = 0;
    T xi = 1;
    for (size_t i=1;i<coef.size();i++) {
      s += T(i)*coef[i]*xi;
      xi *= x;
    }
    return s;
  }

  T Derivative (T x,int n) const
  {
    assert(n >= 0);
    if(n == 0) return Evaluate(x);
    else if(n == 1) return Derivative(x);
    return Differentiate(n).Evaluate(x);
  }

  // return the derivative of this polynomial
  Polynomial<T> Differentiate() const
  {
    if(Size() <= 1) {
      return Polynomial<T>(0);
    }

    Polynomial<T> deriv;
    deriv.coef.resize(coef.size()-1);
    for(size_t i=0;i+1<coef.size();i++)
      deriv.coef[i] = (T(i+1))*coef[i+1];
    return deriv;
  }

  // return the antiderivative of this polynomial
  Polynomial<T> AntiDifferentiate() const
  {
    if(Size() <= 1) {
      return Polynomial<T>(0);
    }

    Polynomial<T> anti;
    anti.coef.resize(coef.size()+1);
    for(size_t i=0;i<coef.size();i++)
      anti.coef[i+1] = coef[i]/(T(i+1));
    return anti;
  }

  // return the n'th derivative of this polynomial
  // if n < 0, returns the -n'th antiderivative
  Polynomial<T> Differentiate(int n) const
  {
    if(n < 0) {
      if(n == -1) return AntiDifferentiate();
      else return Differentiate(n+1).AntiDifferentiate();
    }
    else {
      if(n >= (int)Size()) return Polynomial<T>(0);
      if(n == 0) return *this;
      if(n == 1) return Differentiate();
      return Differentiate(n-1).Differentiate();
    }
  }

  inline T operator () (T x) const { return Evaluate(x); }
  inline Polynomial<T> operator () (const Polynomial<T>& x) const { return Evaluate(x); }

  void operator += (T val)
  {
    if(coef.empty()) Resize(1);
    for(size_t i=0;i<coef.size();i++) coef[i] += val;
  }

  void operator -= (T val)
  {
    if(coef.empty()) Resize(1);
    for(size_t i=0;i<coef.size();i++) coef[i] -= val;
  }
 
  void operator *= (T b)
  {
    for (size_t i=0;i<coef.size();i++)
	coef[i] *= b;
  }

  void operator /= (T b)
  {
    for (size_t i=0;i<coef.size();i++)
	coef[i] /= b;
  }

  void operator += (const Polynomial<T>& b)
  {
    if(b.coef.size() > coef.size()) Resize(b.Size());
    for(size_t i=0;i<b.coef.size();i++) coef[i] += b.coef[i];
  }

  void operator -= (const Polynomial<T>& b)
  {
    if(b.coef.size() > coef.size()) Resize(b.Size());
    for(size_t i=0;i<b.coef.size();i++) coef[i] -= b.coef[i];
  }
 
  void operator *= (const Polynomial<T>& b)
  {
    std::vector<T> newCoef((Degree()+1)*(b.Degree()+1),T(0));
    for (size_t i=0;i<coef.size();i++)
      for (size_t j=0;j<b.coef.size();j++)
	newCoef[i+j] += (coef[i] * b.coef[j]);
    coef = newCoef;
  }
};


//unary -
template <class T>
inline Polynomial<T> operator - (const Polynomial<T>& a)
{
  Polynomial<T> res=a;
  res *= -1.0;
  return res;
}

//operations with field type
template <class T>
inline Polynomial<T> operator + (const Polynomial<T>& a, T b)
{
  Polynomial<T> res=a;
  res += b;
  return res;
}

template <class T>
inline Polynomial<T> operator - (const Polynomial<T>& a, T b)
{
  Polynomial<T> res=a;
  res -= b;
  return res;
}

template <class T>
inline Polynomial<T> operator * (const Polynomial<T>& a, T b)
{
  Polynomial<T> res=a;
  res *= b;
  return res;
}

template <class T>
inline Polynomial<T> operator / (const Polynomial<T>& a, T b)
{
  Polynomial<T> res=a;
  res /= b;
  return res;
}


template <class T>
inline Polynomial<T> operator + (T a,const Polynomial<T>& b)
{
  Polynomial<T> res=b;
  res += a;
  return res;
}

template <class T>
inline Polynomial<T> operator - (T a,const Polynomial<T>& b)
{
  Polynomial<T> res=a;
  res -= b;
  return res;
}

template <class T>
inline Polynomial<T> operator * (T a,const Polynomial<T>& b)
{
  Polynomial<T> res=b;
  res *= a;
  return res;
}

//polynomial operations
template <class T>
inline Polynomial<T> operator + (const Polynomial<T>& a, const Polynomial<T>& b)
{
  Polynomial<T> res=a;
  res += b;
  return res;
}

template <class T>
inline Polynomial<T> operator - (const Polynomial<T>& a, const Polynomial<T>& b)
{
  Polynomial<T> res=a;
  res -= b;
  return res;
}

template <class T>
inline Polynomial<T> operator * (const Polynomial<T>& a, const Polynomial<T>& b)
{
  Polynomial<T> res=a;
  res *= b;
  return res;
}

} //namespace Spline

#endif
