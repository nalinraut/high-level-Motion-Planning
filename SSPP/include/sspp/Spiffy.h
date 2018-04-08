#ifndef SSPP_SPIFFY_H
#define SSPP_SPIFFY_H

#include "Service.h"
#include <map>

namespace SSPP {
  using namespace std;

///Configure the address of the spiffy server.  Default is tcp://localhost:4567.
///Cannot reconfigure this after spiffy is running.
void SetSpiffyServer(const char* addr);
///Returns the address of the current spiffy server
const char* GetSpiffyServer();
///Start a spiffy client, and set the current spiffy context.  The default context
///is "".  Context strings should be of the form ".path" or nested like ".path.subpath" etc.
bool StartSpiffy(const char* context=NULL);
///Stop spiffy client
void StopSpiffy();
///Sets the spiffy context.  This can be changed while the client is running.
///The default context is "".  Context strings should be of the form ".path" or 
///nested like ".path.subpath" etc.
void SetSpiffyContext(const char* context=NULL);

/** @brief Base class for a Spiffy variable.  You will usually not use
 * this class, but rather Spiffy<T> where T is the type that the
 * variable should have.
 * 
 * Variables are given a valid identifier on construction, which is 
 * a string containing alphanumeric characters, and any characters except
 * for ., [, and ].  Recursive nesting is permitted via JSON paths like
 * ".subdir.arrayname[index].varname".
 *
 * Variables can be set to push mode, which means the server will send 
 * updates whenever the variable is written to.  get() will then immediately
 * return the last received value, without communication with the server.
 *
 *  By default a variable is in  pull mode, which means it is read from the
 * server whenever get() is called.  If the variable is read frequently, but
 * not written to frequently, then it is better to set the variable to push
 * mode.  
 */
class AnySpiffy
{
 public:
  ///Initializes a Spiffy variable with the given name on the current Spiffy server/context
  AnySpiffy(const char* name);
  virtual ~AnySpiffy();
  ///Returns the name of this variable
  const string& GetName() const { return name; }
  ///Returns the full path on the Spiffy server 
  const string& GetFullPath() const { return path; }
  ///If enabled=true, marks that this process is the only one that can write to this
  ///Spiffy variable. Returns false if another process has claimed exclusive write.
  ///If enabled=false, disables exclusive write for this process.  Returns false
  ///if this process does not have exclusive write.
  bool SetExclusiveWrite(bool enabled=true);
  ///Returns true if this node is allowed to write to this Spiffy variable
  bool CanWrite();
  ///Returns true if the Spiffy variable has been previously set()
  bool CanRead();
  ///Configure this variable to read on get().  This is helpful when get() is not called
  ///as often as the variable is set().
  void SetPullMode();
  ///Configure this variable to be changed whenever it is set() by another process.
  ///This is helpful when get() is called more often than the variable is set().
  void SetPushMode();

  ///If this is a push mode variable, returns true if there's a new value since the last
  ///get() call.
  bool HasNewValue();

  ///Sets the variable to the given value, returns true if successful.
  ///false might be returned if this process is not allwed to write.
  bool set(const AnyCollection& value);
  ///Sets the variable to the given value, returns true if successful
  bool get(AnyCollection&);

  ///Internally called for push mode variables
  void _updated(const AnyCollection& value);

 protected:
  string name;
  string path;
  int id;
  AnyCollection value;
  bool pushMode;
  bool newValue;
};

/** @brief A variable of type T hosted on the spiffy server.
 *
 * Usage (example for an integer variable):
 *   Spiffy<int> myvar(identifier);
 *   int value = myvar.get(); 
 *   printf("Value: %d\n",value);
 *   value = myvar;   //automatic casting
 *   printf("Casted value: %d\n",value);
 *   myvar.set(value+1);
 *   value = myvar.get();
 *   printf("New value: %d\n",value);
 *   myvar = value + 1;   //automatic assignment
 *   value = myvar.get();
 *   printf("New value (assignment): %d\n",value);
 *
 * Inherits from AnySpiffy, which has the useful functions
 * SetPullMode/SetPushMode(), HasNewValue(), SetExclusiveWrite().
 */
template <class T>
class Spiffy : public AnySpiffy
{
 public:
  ///Initializes a Spiffy variable with the given name on the current Spiffy server/context
  Spiffy(const char* name);
  Spiffy(const char* name,const T& defaultValue);
  ///Automatic get() / convert to type T. 
  operator T();
  ///Automatic get() / convert to type T.  If success != NULL, its value is set to
  ///the success of getting/converting.
  T get(bool* success=NULL);
  ///Automatic convert from type T / set().  Returns false if set() failed.
  bool operator=(const T& value);
};

template <class T>
Spiffy<T>::Spiffy(const char* name)
  :AnySpiffy(name)
{}

template <class T>
Spiffy<T>::Spiffy(const char* name,const T& defaultValue)
  :AnySpiffy(name)
{
  AnyCollection val;
  AnySpiffy::get(val);
  if(val.size()==0) {
    set(defaultValue);
  }
}

template <class T>
Spiffy<T>::operator T()
{
  return get();
}

template <class T>
T Spiffy<T>::get(bool* success)
{
  T value;
  AnyCollection res;
  if(!AnySpiffy::get(res)) {
    if(success) *success = false;
  }
  else {
    if(res.as(value)) {
      if(success) *success = true;
    }
    else {
      if(success) *success = false;
    }
  }
  return value;
}

template <class T>
bool Spiffy<T>::operator=(const T& value)
{
  return AnySpiffy::set(AnyCollection(value));
}

} //namespace SSPP

#endif 
