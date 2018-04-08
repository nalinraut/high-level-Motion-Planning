import sys
import os
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
sys.path.append(os.path.join(ebolabot_root,"Hardware/Pointer"))
from Common.system_config import EbolabotSystemConfig
from sspp import topic
try:
	from pointer_client import PPUClient
except ImportError:
	print "ppu_system_state_broadcaster must be run from Ebolabot main directory"
	print sys.path
	raise
	exit(1)

#Configuration variable: where's the state server?
system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip',('localhost',4568))

class PPUSystemStateBroadcaster(topic.TopicClient):
	def __init__(self):
		topic.TopicClient.__init__(self)
		try:
			self.ppu = PPUClient()
		except RuntimeError:
			print "Unable to connect to PPU server?"
			self.ppu = None
			raise
	def onUpdate(self):
		if not self.opened:
			print "Error occurred?"
			return
		if self.ppu == None:
			self.set('.ppu',None)
		else:
			pan = self.ppu.getCmdRad('pan')
			tilt = self.ppu.getCmdRad('tilt')
			ext = self.ppu.getCmdStrokPrct()
			span,stilt,sext = self.ppu.getSnsConfig('rad/prct')
			self.set('.ppu.cmd',{'pan':pan,'tilt':tilt,'ext':ext})
			self.set('.ppu.sns',{'pan':span,'tilt':stilt,'ext':sext})

if __name__ == '__main__':
	broadcaster = PPUSystemStateBroadcaster()
	broadcaster.open(system_state_addr)
	if not broadcaster.opened:
		print "System state server is not running in address %s:%d"%(system_state_addr[0],system_state_addr[1])
		exit(1)
	#run forever with frequency 10Hz
	broadcaster.run(10)
