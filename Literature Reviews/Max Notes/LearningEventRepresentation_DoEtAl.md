# Learning event representation: As sparse as possible, but not sparser
Tuan Do and James Pustejovsky
[TOC]
Initially published 10/02/17

##  Summary
Used video interactions of humans and objects to create sentences such as "Human pushes A towards B"


### Methods
They propose a framework for event recording and a methodology for efficient classification of an event. QSRLib was used for specific feature functions such as "Cardinal Direction", "Moving or Static" and "Qualitative Trajectory Calculus" 
For learning an event, they used Multilayer perceptron learning (MLP) for event level features and a Long-short term memory (LSTM) combined with a Conditional Random Field (CRF) to generate event description. 

### Notes
This could be of varying use to us. Its accuracy was not ideal (93-82%) but its ability to create sematic relations between objects as a way of describing events could be exremely relevant to recreating those events. "Put A on B" would be a useful sentence to generate a motion from, but "Pull B away from A" would need the addition of some numerical output to direct the length of the motion.

