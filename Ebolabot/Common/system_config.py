import json

class EbolabotSystemConfig:
    system_config_fn = ""
    system_config = {}
    @staticmethod
    def get(key,astype=None):
        if not EbolabotSystemConfig.ensureLoaded():
            raise ValueError("Ebolabot system config %s not loaded"%(EbolabotSystemConfig.system_config_fn,))
        if astype == None:
            return EbolabotSystemConfig.system_config[key]
        else:
            return astype(EbolabotSystemConfig.system_config[key])

    @staticmethod
    def getdefault(key,default,astype=None):
        if not EbolabotSystemConfig.ensureLoaded():
            print "Ebolabot system config",EbolabotSystemConfig.system_config_fn,"not loaded, returning default"
            return default
        if astype == None:
            try:
                return EbolabotSystemConfig.system_config[key]
            except KeyError:
                print "Ebolabot system config",EbolabotSystemConfig.system_config_fn,"does not have key",key,", returning default"
                return default
        else:
            try:
                return astype(EbolabotSystemConfig.system_config[key])
            except Exception:
                print "Ebolabot system config",EbolabotSystemConfig.system_config_fn,"does not have key",key,", returning default"
                return default

    @staticmethod
    def get_ip(key):
        """Convenience function for getting IP addresses back in tuple form
        (IP,port)."""
        read_key = EbolabotSystemConfig.get(key)
        if read_key.startswith("tcp://"):
            read_key = read_key[6:]
            # print "read_key = " ,read_key
        res = read_key.split(':')
        return tuple([res[0], int(res[1])])

    @staticmethod
    def getdefault_ip(key,default):
        """Convenience function for getting IP addresses back in tuple form
        (IP,port)."""
        res = EbolabotSystemConfig.getdefault(key,default)
        if isinstance(res,(str,unicode)):
            if res.startswith("tcp://"):
                res = res[6:]
            res = res.split(':')
            res[1] = int(res[1])
            return tuple(res)
        return res

    @staticmethod
    def setFileName(fn):
        EbolabotSystemConfig.system_config_fn = fn

    @staticmethod
    def ensureLoaded():
        if len(EbolabotSystemConfig.system_config)==0:
            if len(EbolabotSystemConfig.system_config_fn)==0:
                import os
                EbolabotSystemConfig.system_config_fn = os.path.join(os.getenv('EBOLABOT_PATH',''),'Common/system_config.json')
            try:
                EbolabotSystemConfig.system_config = json.load(open(EbolabotSystemConfig.system_config_fn,'r'))
                return True
            except IOError:
                return False
        return True

    @staticmethod
    def isLoaded():
        return len(EbolabotSystemConfig.system_config)>0

