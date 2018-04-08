#include "system_config.h"
#include <stdlib.h>
using namespace std;

bool EbolabotSystemConfig::EnsureLoaded()
{
  if(IsLoaded()) return true;
  if(system_config_fn.length()==0){
    const char* prefix = getenv("EBOLABOT_PATH");
    if(prefix == NULL) {
      printf("EbolabotSystemConfig: Couldn't read EBOLABOT_PATH environment variable, assuming program is run from main path\n");
      system_config_fn = "Common/system_config.json";
    }
    else
      system_config_fn = string(prefix)+"/Common/system_config.json";
  }
  ifstream in(system_config_fn.c_str(),ios::in);
  if(!in) {
    printf("EbolabotSystemConfig: Couldn't load system configuration from %s, check paths\n",system_config_fn.c_str()); 
    return false;
  }
  if(!system_config.LoadJSON(in)) {
    printf("EbolabotSystemConfig: Couldn't read system configuration from %s, check JSON file format\n",system_config_fn.c_str()); 
    return false;
  }
  return true;
}

bool EbolabotSystemConfig::IsLoaded() { return system_config.size() > 0; }

string EbolabotSystemConfig::system_config_fn = "";
PropertyMap EbolabotSystemConfig::system_config;
