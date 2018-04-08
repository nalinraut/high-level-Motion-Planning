#ifndef EBOLABOT_SYSTEM_CONFIG_H
#define EBOLABOT_SYSTEM_CONFIG_H

#include <KrisLibrary/utils/PropertyMap.h>
#include <string>
#include <fstream>

class EbolabotSystemConfig
{
 public:
  ///Retrieves the named key from the config file. Usually users will just call
  ///this or GetDefault
  template <class T> static bool Get(const char* key,T& value);
  ///Retrieves the named key from the config file, setting a default value
  ///if it was not found. Usually users will just call
  ///this or Get
  template <class T> static bool GetDefault(const char* key,T& value,const T& defaultValue);
  ///Sets the location of the file system_config.json.
  ///This doesn't usually have to be called if your program is being run
  ///from the Ebolabot main folder.
  static void SetFileName(const char* fn) { system_config_fn = fn; }
  ///Called internally
  static bool EnsureLoaded();  ///Can be checked to raise an error message if the config file was not found
  static bool IsLoaded();

  static std::string system_config_fn;
  static PropertyMap system_config;
};

template <class T> 
bool EbolabotSystemConfig::Get(const char* key,T& value)
{
  if(!EnsureLoaded())  return false;
  if(!system_config.get(key,value)) {
    printf("EbolabotSystemConfig: Couldn't read key %s from %s",key,system_config_fn.c_str()); 
    return false;
  }
  return true;
}

template <class T> 
bool EbolabotSystemConfig::GetDefault(const char* key,T& value,const T& defaultValue)
{
  if(!EnsureLoaded()) { 
    printf("EbolabotSystemConfig: Couldn't read key %s from %s, using default value",key,system_config_fn.c_str()); 
    value = defaultValue; 
    return false; 
  }
  if(!system_config.get(key,value)) {
    printf("EbolabotSystemConfig: Couldn't read key %s from %s, using default value",key,system_config_fn.c_str()); 
    value = defaultValue;
    return false;
  }
  return true;
}

#endif

