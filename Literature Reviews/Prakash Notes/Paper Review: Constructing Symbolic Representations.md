# Constructing Symbolic Representations for High-Level Planning
George Konidaris, Leslie Pack Kaelbling and Tomas Lozano-Perez

## Summary
The paper addresses the problem of constructing a symbolic description of a continuous, low-level environment for use in planning. The paper also proves that symbols that can represent the preconditions and effects of an agent's actions are both necessary and sufficient for high-level planning. The main focus is towards establishing a close relationship between an agent's actions and the symbols required to plan to use them. The agent's environment and actions completely determine the symbolic representation required for planning. This removes a critical design problem when constructing agents that combine high-level planning with low-level control. Moreover, in principle it enables an agent to construct its own symbolic representation through learning. The resulting representation can be converted into PDDL (Planning Domain Description Language) that enables very fast planning using off-shelf motion planners.

The author then provides us with the necessary background and setting required to understand the rest of the paper. This is where he goes on to define various kinds of symbols and sets that are necessary for high-level planning. Further, he also discusses about converting the symbol algera developed previously into a set-theoretic domain specification expressed in PDDL. He then cements the discussion by giving us a concrete example on how to construct such a PDDL for the continuous playroom domain. 

## Side Notes
This paper pretty much forms the basis of the "Learning Symbolic Representations for Absract High-Level Planning" paper by Konidaris. The flow of thoughts and definition of the symbols constructed are all similar to the original paper. The discussion is kept relatively simple and concise for brevity so understanding this paper shouldn't be to difficult if you had followed the original paper.

Link: http://irl.cs.brown.edu/pubs/orig_sym_aaai.pdf