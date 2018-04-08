import redundancy_cache as redundancy
from redundancyvisualization import read_options
import multiprocessing
import os
import copy
import time
import json

if __name__ == "__main__":
	import sys
	if len(sys.argv) > 1:
		problem = sys.argv[1]
	else:
		problem = 'baxter_col'

	settings_file = None
	if len(sys.argv) > 2:
		settings_file = sys.argv[2]
	else:
		settings_file = 'Rvariable.json'
		
#	out_file = "experiments/" + problem + "_k_experiments.csv"
#	if not os.path.isdir("experiments"):
#		os.makedirs("experiments")
#	if not os.path.isfile(out_file):
#		with open(out_file, 'a') as f:
#			f.write('num_w,num_q_per_w,k,workspace_sampling_method,nondegenerate_workspace_nodes,nondegenerate_workspace_edges,total_workspace_edges,avg_num_configs_per_wnode,avg_num_self_motion_ccs,avg_max_size_self_motion_ccs,avg_avg_size_self_motion_ccs,avg_min_size_self_motion_ccs,self_motion_time,inter_wnode_time,num_csp_conflicts,csp_assignment_time,avg_num_anchors,avg_num_neighbors,num_collapsed,num_tried_to_collapse,visibility')
	
	#Whether or not to construct a self-motion manifold graph. If True, will use a visbility prm to construct the graph
	#visibility = True #deprecated. True if k > 0 or k == None, False otherwise
	parallel = True
	solveCSP = True
	visualize = False
	numTrials = 1
	# Value of k for self-motion manifold visibility prm
	#klist = [5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,0] #include zero in klist to compare results to case of visibility = False
	#klist = [5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95]
	klist = [None]
	# Number of workspace nodes	
	Nw_list = [20000, 30000]
	# Number of configuration space nodes
	Nq_list = [200]
	threads_list = [64]
	#csize_list = [0, 200, 400, 800, 1600, 3200, 6400, 12800, 25600, 51200, 102400, 204800, 409600, 819200, 1638400]
	#csize_list = [4, 8, 16, 32, 64, 128]
	csize_list = [3200]


	method = 'staggered_grid'
	
	for i in range(numTrials):
		for Nw in Nw_list:
			for Nq in Nq_list:
				for k in klist:
					for threads in threads_list:
						for csize in csize_list:
							if k == None or k > 0:
								visibility = True
							else:
								visibility = False
							if not k >= Nq:
								print "Beginning problem = " + problem + ", k = " + str(k) + ", Nq = " + str(Nq) + ", Nw = " + str(Nw) + ", threads = " + str(threads) + ", cache size = " + str(csize) + ", trial = " + str(i+1)
								print "Initializing problem..."
								
								options = {'Nw':Nw,'workspace_graph':method,'Nq':Nq,'vis':k,'cache':csize,'clean':True}
								program = redundancy.make_program(read_options(problem,options,settings_file=settings_file))
								if parallel:
									program.pr.setThreads(threads)
									program.pr.run(Nq,k,program.folder,solveCSP=solveCSP)
								else:
									sr = redundancy.SequentialResolver(problem, program.rr, Nw, method, visibility)
									sr.run(Nq,k,program.folder,solveCSP=solveCSP)

								if visualize:
									program.run()
