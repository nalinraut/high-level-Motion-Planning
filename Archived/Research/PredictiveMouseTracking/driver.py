#!/usr/bin/env python
import random
import math
import Tkinter
import time
import os
import vectorops

# msec between updates
SPEED = 20
WIDTH = 800
HEIGHT = 600
BUFFER = 100

def segment_closest_point(s,x):
	"""Given a segment s=(a,b) and a point x, returns a tuple containing
	the closest point parameter in [0,1] and the closest point on s"""
	dir = vectorops.sub(s[1],s[0])
	d = vectorops.sub(x,s[0])
	dp = vectorops.dot(d,dir)
	dnorm2 = vectorops.dot(dir,dir)
	if dp < 0:
		return (0.0,s[0])
	if dp > dnorm2:
		return (1.0,s[1])
	proj = vectorops.add(s[0],vectorops.mul(dir,dp/dnorm2))
	return (dp/dnorm2,proj)

def ray_ray_intersection(s1,d1,s2,d2,eps=1e-6):
	#solve the coordinates x = s1+t*d1 = s2-u*d2 for some t, u
	#[d1,d2]*[t,u]^T = [s2-s1]
	det = d1[0]*d2[1]-d1[1]*d2[0]
	if abs(det)<eps: return None
	#inv mat is 1/det [d2[1] -d2[0]]
	#                 [-d1[1] d1[0]]
	b = vectorops.sub(s2,s1)
	t = (d2[1]*b[0]-d2[0]*b[1])/det
	u = (-d1[1]*b[0]+d1[0]*b[1])/det
	return vectorops.madd(s1,d1,t)

class GrabberSimScenario:
	def __init__(self):
		pass
	def step(self,dt):
		pass
	def draw(self,canvas):
		pass
	def instructions(self):
		return ""
	def params(self):
		"""Return a dict of problem parameters"""
		pass
	def state(self):
		"""Return a dict of problem state"""
		pass
	def mouse_click(self,x,y):
		"""Return true if processed"""
		return False
	def mouse_release(self,x,y):
		"""Return true if processed"""
		return False
	def mouse_motion(self,x,y):
		"""Return true if processed"""
		return False
	def done(self):
		pass
	def reset(self):
		pass

class GrabberReachScenario(GrabberSimScenario):
	def __init__(self):
		pass
	def step(self,dt):
		pass
	def draw(self,canvas):
		canvas.create_oval(self.target[0]-self.target_radius,self.target[1]-self.target_radius,self.target[0]+self.target_radius,self.target[1]+self.target_radius, fill='red', outline='black')
		canvas.create_oval(self.widget[0]-self.widget_radius,self.widget[1]-self.widget_radius,self.widget[0]+self.widget_radius,self.widget[1]+self.widget_radius, fill='yellow', outline='black')
	def instructions(self):
		return 'Drag the yellow dot to the red target.'
	def params(self):
		return {"target x":self.target[0],"target y":self.target[1],"target r":self.target_radius}
	def state(self):
		return {"widget x":self.widget[0],"widget y":self.widget[1]}
	def mouse_click(self,x,y):
		if (self.widget[0]-x)**2 + (self.widget[1]-y)**2 <= self.widget_radius**2:
			self.clicked = True
			self.click_ofs = (self.widget[0]-x,self.widget[1]-y)
			return True
		return False
	def mouse_release(self,x,y):
		self.clicked = False
		return False
	def mouse_motion(self,x,y):
		if self.clicked:
			self.widget = (self.click_ofs[0]+x,self.click_ofs[1]+y)
			return True
		return False
	def done(self):
		return ((self.widget[0]-self.target[0])**2 + (self.widget[1]-self.target[1])**2 <= self.target_radius**2)
	def reset(self):
		self.target = (random.randrange(BUFFER,WIDTH-BUFFER*2),random.randrange(BUFFER,HEIGHT-BUFFER*2))
		self.target_radius = random.randrange(5,50)
		self.widget = (random.randrange(BUFFER,WIDTH-BUFFER*2),random.randrange(BUFFER,HEIGHT-BUFFER*2))
		self.widget_radius = 10
		self.clicked = False


def make_arc(c,r,theta1,theta2,dtheta=0.1):
	res = []
	theta = theta1
	while theta < theta2:
		res.append((c[0]+r*math.cos(theta),c[1]+r*math.sin(theta)))
		theta += dtheta
	theta = theta2
	res.append((c[0]+r*math.cos(theta),c[1]+r*math.sin(theta)))
	return res

#Global pattern names
pattern_names = ['square','triangle','circle','figure_eight']
patterns = [[(300,200),(500,200),(500,400),(300,400),(300,220)], \
	    [(400,200),(200,400),(600,400),(420,220)], \
	    make_arc((400,300),200,0,math.pi*31/16), \
	    make_arc((250,300),100,math.pi/4,math.pi*7/4)+list(reversed(make_arc((550,300),100,math.pi*5/4,math.pi*11/4)))+[(250+90,300+50)] \
	    ]

class GrabberTraceScenario(GrabberSimScenario):
	def __init__(self):
		pass
	def step(self,dt):
		pass
	def make_outline(self):
		left = []
		right = []
		for i in xrange(len(self.path)):
			s1l = None
			s1r = None
			s2l = None
			s2r = None
			if i > 0:
				d = vectorops.unit(vectorops.sub(self.path[i],self.path[i-1]))
				ofs = (self.gutter*d[1],-self.gutter*d[0])
				s1l = (d,ofs)
				s1r = (d,(-ofs[0],-ofs[1]))
			if i+1 < len(self.path):
				d = vectorops.unit(vectorops.sub(self.path[i+1],self.path[i]))
				ofs = (self.gutter*d[1],-self.gutter*d[0])
				s2l = (d,ofs)
				s2r = (d,(-ofs[0],-ofs[1]))
			if i == 0:
				left.append(tuple(vectorops.add(self.path[i],s2l[1])))
				right.append(tuple(vectorops.add(self.path[i],s2r[1])))
			elif i+1 == len(self.path):
				left.append(tuple(vectorops.add(self.path[i],s1l[1])))
				right.append(tuple(vectorops.add(self.path[i],s1r[1])))
			else:
				#figure out intersections
				if vectorops.dot(s1r[0],s2r[1]) < 0:
					#inner junction, find line intersection
					right.append(tuple(vectorops.add(self.path[i],ray_ray_intersection(s1r[1],s1r[0],s2r[1],s2r[0]))))
				else:
					#outer junction
					right.append(tuple(vectorops.add(self.path[i],s1r[1])))
					right.append(tuple(vectorops.add(self.path[i],s2r[1])))
				if vectorops.dot(s1l[0],s2l[1]) < 0:
					#inner junction, find line intersection
					left.append(tuple(vectorops.add(self.path[i],ray_ray_intersection(s1l[1],s1l[0],s2l[1],s2l[0]))))
				else:
					#outer junction
					left.append(tuple(vectorops.add(self.path[i],s1l[1])))
					left.append(tuple(vectorops.add(self.path[i],s2l[1])))
				
		self.outline = left + list(reversed(right))
	def draw(self,canvas):
		canvas.create_polygon(sum((pt for pt in self.outline),tuple()),outline="",fill="gray")
		canvas.create_line(sum((pt for pt in self.path),tuple()))
		#slider = self.eval_path(self.slider_param)
		#canvas.create_oval(slider[0]-self.slider_radius,slider[1]-self.slider_radius,slider[0]+self.slider_radius,slider[1]+self.slider_radius, fill='red', outline='black')
		canvas.create_oval(self.widget[0]-self.widget_radius,self.widget[1]-self.widget_radius,self.widget[0]+self.widget_radius,self.widget[1]+self.widget_radius, fill='yellow', outline='black')
	def instructions(self):
		return 'Drag the yellow dot through the corridor to the end of the curve.'
	def params(self):
		return {"pattern":self.pattern,"width":self.pattern_w,"height":self.pattern_h,"gutter":self.gutter}
	def state(self):
		slider = self.eval_path(self.slider_param)
		return {"slider param":self.slider_param,"slider x":slider[0],"slider y":slider[1],"slider tangent x":self.path_tangent()[0],"slider tangent y":self.path_tangent()[1],"widget x":self.widget[0],"widget y":self.widget[1]}
	def mouse_click(self,x,y):
		if (self.widget[0]-x)**2 + (self.widget[1]-y)**2 <= self.widget_radius**2:
			self.clicked = True
			self.click_ofs = (self.widget[0]-x,self.widget[1]-y)
			return True
		return False
	def mouse_release(self,x,y):
		self.clicked = False
		return False
	def do_move(self,x,y):
		oldwidget = self.widget
		oldslider = self.slider_param
		self.widget = (self.click_ofs[0]+x,self.click_ofs[1]+y)
		self.move_slider()
		slider = self.eval_path(self.slider_param)
		if vectorops.distance(self.widget,slider) > self.gutter:
			self.widget = oldwidget
			self.slider_param = oldslider
	def mouse_motion(self,x,y):
		if self.clicked:
			if abs(self.click_ofs[0]+x-self.widget[0]) > 5 or abs(self.click_ofs[1]+y-self.widget[1]) > 5:
				#long distance move -- split into smaller moves
				oldx = self.widget[0]-self.click_ofs[0]
				oldy = self.widget[1]-self.click_ofs[1]
				n = int(math.ceil(float(max(abs(oldx-x),abs(oldy-y)))/5))
				for i in xrange(n):
					self.do_move(oldx + int(float(i)/n*(x-oldx)),oldy + int(float(i)/n*(y-oldy)))
				return True

			self.do_move(x,y)
			if int(self.slider_param) >= len(self.path)-1:
				self.is_done = True
			return True
		return False
	def done(self):
		return self.is_done
	def reset(self):
		self.is_done = False
		pattern = random.randrange(len(patterns))
		self.pattern = pattern_names[pattern]
		self.path = patterns[pattern][:]
		wrange = [0.5,2.0]
		hrange = [0.5,2.0]
		bmin = list(self.path[0])
		bmax = list(self.path[0])
		for pt in self.path:
			if pt[0] < bmin[0]: bmin[0] = pt[0]
			elif pt[0] > bmax[0]: bmax[0] = pt[0]
			if pt[1] < bmin[1]: bmin[1] = pt[1]
			elif pt[1] > bmax[1]: bmax[1] = pt[1]
		wrange[1] = min(wrange[1],WIDTH*0.95/(bmax[0]-bmin[0]))
		hrange[1] = min(hrange[1],HEIGHT*0.95/(bmax[1]-bmin[1]))
		self.pattern_w = random.uniform(*wrange)
		self.pattern_h = random.uniform(*hrange)
		self.gutter = random.uniform(5,25)
		#scale the pattern
		for i in xrange(len(self.path)):
			self.path[i] = (self.pattern_w*(self.path[i][0]-WIDTH/2)+WIDTH/2,self.pattern_h*(self.path[i][1]-HEIGHT/2)+HEIGHT/2)
		self.slider_param = 0.0
		self.slider_radius = 10
		self.widget = self.path[0]
		self.widget_radius = 10
		self.clicked = False
		self.make_outline()

	def local_search(self,starting_value):
		#returns the distance and the parameter value of the closest
		#local point to the widget
		value = starting_value
		index = int(math.floor(starting_value))
		d = vectorops.distance(self.eval_path(starting_value),self.widget)
		while index >= 0 and index+1 < len(self.path):
			#find closest point on segment
			(u,cp) = segment_closest_point((self.path[index],self.path[index+1]),self.widget)
			if vectorops.distance(self.widget,cp) < d:
				d = vectorops.distance(self.widget,cp)
				if u == 0:
					value = index
					index -= 1
				elif u == 1:
					index += 1
					value = index
				else:
					print "Settled on segment",(d,index+u)
					return (d,index + u)
			else:
				print "Local minimum",(d,value)
				return (d,value)
		print "Ran out of path",(d,value)
		return (d,value)


	def move_slider(self):
		"""Do a local search around the slider parameter to bring it closer to the widget"""
		print "Search 1"
		(distance,param) = self.local_search(self.slider_param)
		index = int(math.floor(self.slider_param))
		#start looking forward and backward 1 index
		if index > 0:
			print "Search 2"
			(distance2,param2) = self.local_search(index-1)
			if distance2 < distance:
				distance = distance2
				param = param2
		if index+1 < len(self.path):
			print "Search 3"
			(distance2,param2) = self.local_search(index+1)
			if distance2 < distance:
				distance = distance2
				param = param2
		self.slider_param = param

	def eval_path(self,param):
		index = int(math.floor(param))
		u = param - index
		if index < 0:
			return self.path[0]
		if index+1 >= len(self.path):
			return self.path[-1]
		return vectorops.madd(self.path[index],vectorops.sub(self.path[index+1],self.path[index]),u)

	def path_tangent(self,eps=0.1):
		p1 = self.eval_path(self.slider_param+eps)
		p2 = self.eval_path(self.slider_param-eps)
		return vectorops.mul(vectorops.sub(p1,p2),1.0/vectorops.distance(p1,p2))


class GrabberTrajectoryScenario(GrabberSimScenario):
	def __init__(self):
		pass
	def step(self,dt):
		if self.status == 'demo':
			self.current_time += dt
			if self.current_time > self.path_times[-1]+1.0:
				self.current_time = 0.0
				self.status = 'running'
		elif self.status == 'running':
			if self.clicked or self.current_time > self.path_times[-1]:
				self.current_time += dt
			if self.current_time > self.path_times[-1]+2.0:
				self.status = 'done'
	def draw(self,canvas):
		canvas.create_line(sum((pt for pt in self.path),tuple()))
		slider = self.eval_path(self.current_time)
		if self.status == 'demo':
			canvas.create_oval(slider[0]-self.slider_radius,slider[1]-self.slider_radius,slider[0]+self.slider_radius,slider[1]+self.slider_radius, fill='red', outline='black')
		else:
			canvas.create_oval(slider[0]-self.slider_radius,slider[1]-self.slider_radius,slider[0]+self.slider_radius,slider[1]+self.slider_radius, fill='pink', outline='gray')
			canvas.create_oval(self.widget[0]-self.widget_radius,self.widget[1]-self.widget_radius,self.widget[0]+self.widget_radius,self.widget[1]+self.widget_radius, fill='yellow', outline='black')
	def instructions(self):
		return 'Drag the yellow dot in the same manner as the red dot.'
	def params(self):
		return {"pattern":self.pattern,"width":self.pattern_w,"height":self.pattern_h,"speed":self.speed}
	def state(self):
		slider = self.eval_path(self.current_time)
		return {"slider param":self.current_time,"slider x":slider[0],"slider y":slider[1],"slider velocity x":self.path_velocity()[0],"slider velocity y":self.path_velocity()[1],"widget x":self.widget[0],"widget y":self.widget[1]}
	def mouse_click(self,x,y):
		if self.status == 'demo':
			return False;
		if (self.widget[0]-x)**2 + (self.widget[1]-y)**2 <= self.widget_radius**2:
			self.clicked = True
			self.click_ofs = (self.widget[0]-x,self.widget[1]-y)
			return True
		return False
	def mouse_release(self,x,y):
		self.clicked = False
		return False
	def mouse_motion(self,x,y):
		if self.status == 'demo':
			return False;
		if self.clicked:
			self.widget = (self.click_ofs[0]+x,self.click_ofs[1]+y)
			return True
		return False
	def done(self):
		return self.status == 'done'
	def reset(self):
		self.status = 'demo'
		self.current_time = 0.0
		pattern = random.randrange(len(patterns))
		self.pattern = pattern_names[pattern]
		self.path = patterns[pattern][:]
		wrange = [0.5,2.0]
		hrange = [0.5,2.0]
		bmin = list(self.path[0])
		bmax = list(self.path[0])
		for pt in self.path:
			if pt[0] < bmin[0]: bmin[0] = pt[0]
			elif pt[0] > bmax[0]: bmax[0] = pt[0]
			if pt[1] < bmin[1]: bmin[1] = pt[1]
			elif pt[1] > bmax[1]: bmax[1] = pt[1]
		wrange[1] = min(wrange[1],WIDTH*0.95/(bmax[0]-bmin[0]))
		hrange[1] = min(hrange[1],HEIGHT*0.95/(bmax[1]-bmin[1]))
		self.pattern_w = random.uniform(*wrange)
		self.pattern_h = random.uniform(*hrange)
		if random.getrandbits(1):
                        self.pattern_w *= -1
                if random.getrandbits(1):
                        self.pattern_h *= -1
		self.path_times = [0.0]*len(self.path)
		self.speed = random.uniform(100,500)
		#scale the pattern, assign the timing
		for i in xrange(len(self.path)):
			self.path[i] = (self.pattern_w*(self.path[i][0]-WIDTH/2)+WIDTH/2,self.pattern_h*(self.path[i][1]-HEIGHT/2)+HEIGHT/2)
			if i==0:
				self.path_times[i] = 0.0
			else:
				self.path_times[i] = self.path_times[i-1]+vectorops.distance(self.path[i],self.path[i-1])/self.speed
		self.slider_radius = 10
		self.widget = self.path[0]
		self.widget_radius = 10
		self.clicked = False

	def findSegment(self,t):
		if t < self.path_times[0]:
			return -1
		# TODO: do a binary search
		for i in range(len(self.path_times)-1):
			if t < self.path_times[i+1]:
				return i
		return len(self.path_times)-1

	def eval_path(self,param):
		index = self.findSegment(param)
		if index < 0: return self.path[0]
		if index+1 >= len(self.path_times): return self.path[-1]
		u = (param - self.path_times[index])/(self.path_times[index+1]-self.path_times[index])
		return vectorops.madd(self.path[index],vectorops.sub(self.path[index+1],self.path[index]),u)

	def path_velocity(self,eps=0.1):
		p1 = self.eval_path(self.current_time)
		p2 = self.eval_path(self.current_time+eps)
		return vectorops.mul(vectorops.sub(p2,p1),1.0/eps)

class GrabberSim:
	def __init__(self):
		self.scenarioNames = ['Reach target','Follow trajectory']
		self.scenarios = dict(zip(self.scenarioNames,[GrabberReachScenario(),GrabberTrajectoryScenario()]))
		self.current_time = 0.0
		self.history = []
		self.level_status = 'running'
		
		self.dumpFiles = dict(zip(self.scenarioNames,['data/reach_%d.csv','data/traj_%d.csv']))
		self.dumpCounters = dict(zip(self.scenarioNames,[0]*len(self.scenarios)))
		for (s,fn) in self.dumpFiles.iteritems():
			for i in xrange(1,100000):
				if not os.path.exists(fn%(i,)):
					self.dumpCounters[s] = i
					break
		self.reset(self.scenarioNames[0])

	def step(self,dt):
		"""Given the time elapsed since the prior call"""
		self.current_time += dt
		self.scenarios[self.currentScenario].step(dt)
		#wait for 1 second after level is completed
		if self.level_status == 'done':
			self.wait_time += dt
			if self.wait_time > 1.0:
				#go to a new level
				self.dump_history()
				self.reset(self.currentScenario)
		elif self.done():
			self.level_status = 'done'
			self.wait_time = 0.0
		
		if self.currentScenario == 'Reach target':
			pass
		else:
			pass

	def mouse_click(self,x,y):
		if self.scenarios[self.currentScenario].mouse_click(x,y):
			self.history.append((self.current_time,self.scenarios[self.currentScenario].state()))
		
	def mouse_release(self,x,y):
		if self.scenarios[self.currentScenario].mouse_release(x,y):
			self.history.append((self.current_time,self.scenarios[self.currentScenario].state()))

	def mouse_motion(self,x,y):
		if self.scenarios[self.currentScenario].mouse_motion(x,y):
			self.history.append((self.current_time,self.scenarios[self.currentScenario].state()))		

	def reset(self,scenario):
		self.current_time = 0.0		
		self.level_status = 'running'
		self.currentScenario = scenario
		self.history = []
		self.scenarios[scenario].reset()

	def dump_history(self):
		if len(self.history) == 0: return
		fn = self.dumpFiles[self.currentScenario]%(self.dumpCounters[self.currentScenario],)
		print "Dumping to",fn
		f = open(fn,'w')
		params = self.scenarios[self.currentScenario].params()
		f.write(",".join(params.keys()))
		f.write("\n")
		f.write(",".join(str(v) for v in params.values()))
		f.write("\n")
		f.write(",".join(['time']+self.history[0][1].keys()))
		f.write("\n")
		for h in self.history:
			f.write(str(h[0])+',')
			f.write(",".join(str(v) for v in h[1].values()))
			f.write("\n")
		f.close()
		self.dumpCounters[self.currentScenario] += 1

	def draw(self,canvas):
		canvas.delete(Tkinter.ALL)
		canvas.create_rectangle(2,2,WIDTH,HEIGHT, fill='white', outline='black')
		self.scenarios[self.currentScenario].draw(canvas)
		
		if self.level_status=='running':
	       		canvas.create_text(20, 20, text=self.scenarios[self.currentScenario].instructions(), font=('verdana', 10, 'bold'), anchor='w')
	       	else:
	       		canvas.create_text(20, 20, text='Done.', font=('verdana', 10, 'bold'), anchor='w')
		return

	def done(self):
		return self.scenarios[self.currentScenario].done()

class GrabberApp(Tkinter.Frame):
	def __init__(self, master=None):
		Tkinter.Frame.__init__(self, master)
		self.pack_propagate(0)
		self.canvas =  None
		self.sim_status = 'paused'
		self.alarm = None
		self.speed = SPEED
		self.accum_time = 0.0
		self.last_time = 0.0
		self.make_sim()
		self.pack()
		# Scenario names in display order
		self.currentScenario = Tkinter.StringVar()
		self.currentScenario.set(self.sim.scenarioNames[0])
		# always make_canvas(), make_buttons() in order!
		self.make_canvas()
		self.make_buttons()
		self.reset_buttons()
		self.redraw()
	
	def update(self):
		if (self.sim_status == 'running'):
			cur_time = time.time()
			self.sim.step(cur_time-self.last_time)
			self.last_time = cur_time
			self.alarm = self.after(self.speed, self.update)
			self.redraw()

	def redraw(self):
		self.sim.draw(self.canvas)	

	def toggle_running(self):
		if self.sim_status == 'running':
			self.playpause.config(text='Start')
			self.after_cancel(self.alarm)
			self.scenarioSelect.config(state='active')
			self.sim_status = 'paused'
		elif self.sim_status == 'paused':
			self.playpause.config(text='Pause')
			self.alarm = self.after(self.speed, self.update)
			self.scenarioSelect.config(state='disabled')
			self.sim_status = 'running'
			self.last_time = time.time()
	
	def reset(self):
		if self.sim_status == 'running':
			self.toggle_running()
		self.sim.reset(self.currentScenario.get())
		self.make_canvas()
		self.reset_buttons()
		self.redraw()

	def next_test(self):
		self.reset();
		self.toggle_running()
	
	def call_with(self, fns, vals):
		for fn, val in zip(fns,vals):
			fn(*val)
			
	def reset_buttons(self):
		pass

	def make_sim(self):
		self.sim = GrabberSim()
	
	def make_buttons(self):
		# Which scenario?
		self.scenarioFrame = Tkinter.Frame(self)
		self.scenarioLabel = Tkinter.Label(self.scenarioFrame, text="Scenario:")
		self.scenarioSelect = Tkinter.OptionMenu(self.scenarioFrame, self.currentScenario,
								*self.sim.scenarioNames, command=lambda x: self.reset())
		
		# Control buttons
		self.playpause = Tkinter.Button(self, text='Start', command=self.toggle_running)
		self.resetter = Tkinter.Button(self, text='Reset', command=self.reset)
		

		# Pack it all into subframes and then into the main frame's grid
		self.scenarioLabel.pack(side=Tkinter.LEFT)
		self.scenarioSelect.pack(side=Tkinter.LEFT)
		for i,item in enumerate([self.scenarioFrame, self.playpause, self.resetter]):
			item.grid(column=0,row=i)

	def mouse_click(self,event):
		if self.sim_status == 'running':
			res=self.sim.mouse_click(event.x,event.y)
			if res:
				self.redraw()

	def mouse_release(self,event):
		if self.sim_status == 'running':
			res=self.sim.mouse_release(event.x,event.y)
			if res:
				self.redraw()

	def mouse_motion(self,event):
		if self.sim_status == 'running':
			res=self.sim.mouse_motion(event.x,event.y)
			if res:
				self.redraw()
	
	def make_canvas(self):
		if not self.canvas:
			self.canvas = Tkinter.Canvas(self,bg='white',width=WIDTH,height=HEIGHT)
			self.canvas.grid(column=1,row=0,rowspan=4)
			self.canvas.bind('<Button-1>',self.mouse_click)
			self.canvas.bind('<ButtonRelease-1>',self.mouse_release)
			self.canvas.bind('<B1-Motion>',self.mouse_motion)
			self.canvas.bind('<Motion>',self.mouse_motion)
	
def main():
	root = Tkinter.Tk()
	app = GrabberApp(root)
	root.mainloop()

if __name__ == "__main__":
	main()


