# Combining Planning and Motion Planning

Jaesik Choi and Eyal Amir

## Summary

General Purpose manipulation with a robot such as delivering dishes, opening doors with a key is hard and demanding because objects are constrained in position and orientation, many non-spatial constraints interact with each other and robots may have different degrees of freedom. This paper provides a solution to the problem of general purpose robotic manipulation using a novel combination of planning and motion planning.

The approach integrates motions of a robot with other (non-physical or external-to-robot) actions to achieve a goal while manipulating objects. It considers the kinematic constraints of the robot to operate in configuration space and accommodates that with the constraints over object manipulations. Then it automatically generates high-level actions from a low-level C-space based motion planning algorithm by decomposing the planning problem into small segments, thus reducing the complexity of planning.

The key idea is to unify a general purpose planner and a motion planner into one single algorithm. The unified algorithm is composed of three subroutines: 
* Extracting logical high-level actions from a motion planner
* Finding an abstract plan from the logical domain
* Decoding it into Configuration space

## Process

The algorithm first constructs a motion plan (a tree) using a motion planning algorithm (say, RRT). Then actions which change the states of the objects are extracted from the tree. Then, it combines extracted actions with a given Knowledge Base (Object and kinematic constraints). We then create a new Knowledge Base (KB motion) by generating and grouping new propositions, based on the found actions. The knowledge Base domain is then partitioned by a simple graph decomposition algorithm and an abstract plan is found by a factored planning algorithm (similar to graph searching algorithms) which are designed for the decomposed domain. Finally, in decoding a motion plan is found from the abstract plan.

Link: http://sail.unist.ac.kr/papers/ICRA09ChoiJAmirE.pdf
