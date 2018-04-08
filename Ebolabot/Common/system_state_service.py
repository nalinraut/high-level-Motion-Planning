#!/usr/bin/python

from sspp.structure_service import StructureService
import os
import sys
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Common.system_config import EbolabotSystemConfig

system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip',('localhost',4568))

if __name__ == '__main__':
    print "Starting system structure service on",system_state_addr[0],":",system_state_addr[1]
    service = StructureService()
    service.verbosity = 3
    service.open(system_state_addr,asServer=True)
    service.run(100)
