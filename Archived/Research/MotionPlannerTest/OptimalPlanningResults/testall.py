#!/usr/bin/python

import sys
import os
import subprocess

problemdir = "../spaces"
problemsuffix = ".xml"
problems = ["25_moving_obstacles",
            "bugtrap_0.05",
            "kink",
            "kink_with_suboptimal",
            "bar_narrowpassage_0_1",
            "bar_25_circles"]

plannerdir = "../planners"
plannersuffix = ".settings"
planners = [#"restart_rrt_planner",
            #"restart_sbl_planner",
            #"fmmstar_planner",
            "prmstar_planner",
            "rrtstar_planner",
            #"birrtstar_planner",
            "rrtstar_planner_subopt_0.1",
            "rrtstar_planner_subopt_0.2",
            "restart_rrt_shortcut_planner",
            #"restart_sbl_shortcut_planner",
            "lazy_prmstar_planner",
            "lazy_rrgstar_planner",
            #"lazy_birrgstar_planner",
            #"lazy_rrgstar_planner_subopt_0.1",
            #"lazy_rrgstar_planner_subopt_0.2"
            ]
deterministic = set(["fmmstar_planner"])

batchsize = 10
timelimit = 20.0
iterlimit = 10000000
for problem in problems:
    spacefile = os.path.join(problemdir,problem+problemsuffix)
    for planner in planners:
        plannerfile = os.path.join(plannerdir,planner+plannersuffix)
        outputfile = os.path.join("../OptimalPlanningResults",problem)
        if not os.path.exists(outputfile):
            os.makedirs(outputfile)
        outputfile = os.path.join(outputfile,planner.replace("_planner","")+".csv")
        if os.path.exists(outputfile):
            print "Skipping",outputfile
            continue
        cmdline = ["../Plan"]
        cmdline += ["-t",str(timelimit)]
        cmdline += ["-n",str(iterlimit)]
        cmdline += ["-opt"]
        cmdline += ["-p",plannerfile]
        cmdline += ["-b",str(batchsize if planner not in deterministic else 1),outputfile]
        cmdline += [spacefile]
        res = subprocess.call(cmdline)
        if res != 0:
            print "Call to "," ".join(cmdline),"failed"
            exit(0)

klamptdir = '../../../gitlibs/Klampt'
klamptproblems = [('tx90shelves','data/tx90shelves.xml','Examples/plandemo/tx90shelves.configs'),
                  ('baxterpretzel','data/baxter.xml','Examples/plandemo/baxter-pretzel2.configs')]
for probname,probworld,endpts in klamptproblems:
    if probname.startswith('baxter'):
        klampttimelimit = 60.0
    else:
        klampttimelimit = 20.0
    worldfile = os.path.join(klamptdir,probworld)
    configfile = os.path.join(klamptdir,endpts)
    for planner in planners:
        plannerfile = os.path.join(plannerdir,planner+plannersuffix)
        outputfile = os.path.join("OptimalPlanningResults",probname)
        if not os.path.exists(outputfile):
            os.makedirs(outputfile)
        outputfile = os.path.join(outputfile,planner.replace("_planner","")+".csv")
        if os.path.exists(outputfile):
            print "Skipping",outputfile
            continue
        cmdline = ["../KlamptPlan"]
        cmdline += ["-t",str(klampttimelimit)]
        cmdline += ["-n",str(iterlimit)]
        cmdline += ["-opt"]
        #use this for baxter
        if probname.startswith('baxter'):
            cmdline += ["-r","0.01"]
        cmdline += ["-p",plannerfile]
        cmdline += ["-b",str(batchsize if planner not in deterministic else 1),outputfile]
        cmdline += [worldfile,configfile]
        res = subprocess.call(cmdline)
        if res != 0:
            print "Call to "," ".join(cmdline),"failed"
            exit(0)
