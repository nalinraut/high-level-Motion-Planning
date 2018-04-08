# One Shot Imitation Learning
Yan Duan, Marcin Andrychowicz, Bradly Stadie, Jonathan Ho,
Jonas Schneider, Ilya Sutskever, Pieter Abbeel, Wojciech Zaremba
[TOC]
Initially published 3/21/17

##  Summary


In order to allow imitation of an action from a single demonstration, a neural network is trained using a set of action demonstrations and their desired final states. This was used to imitate block stacking. 

### Architecture
It begins with a demonstration network which takes as input a demonstration trajectory. to reduce the size of long samples temporal dropout is used (some % of the samples are removed). it uses a neighborhood attention network to create a query vector and context head to be used by the next layer
Next the context network processes the current state of the demonstration network and produces a context embedding based on the context embeddings produced in the neighborhood layersof the demonstration network. The embedding size is dependent on the number of blocks. the context embedding indentifies a source and target block the the robot next needs to stack, and that is passed on to the manipulation network. 
Finally, the manipulation network computes the action to b taken in order to stack a single block based on the target and source blocks. 


### Comments
one shot is slightly misleading, as this requires the network to be pre-trained on actions before it can "one-shot" replicate a new one (as long as it is close enough to the ones its been trained on). 