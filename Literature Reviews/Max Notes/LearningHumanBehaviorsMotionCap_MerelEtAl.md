# Learning human behaviors from motion capture by adversarial imitation
Josh Merel, Yuval Tassa, Dhruva TB, Sriram Srinivasan, Jay Lemmon, Ziyu Wang,
Greg Wayne, Nicolas Heess
[TOC]
Initially published 7/10/17

##  Summary
Imitation learning is easier to train and more effective at recreating natural looking movements because reward functions for realiztic movement are difficult to create. They were able to train an agent using imitation learning to perform a limited set of tasks in a manner that looks relativel natural and human.



### Methods
GAIL (Jonathan Ho and Stefano Ermon. Generative adversarial imitation learning. In Advances in Neural Information Processing Systems, pages 4565–4573, 2016.), encourages imitators to match states with whatever it is imitating. This paper specifically extends GAIL to perform with access to states (as opposed to actions) and partial states. They trained specific policies using preset reward fuctions to solve simpler tasks, then trained imitation policies off of demonstrations. Adverserial training was able to create relatively efficient trnsistions between actions such as running-> walking and running -> stair climbing. 


