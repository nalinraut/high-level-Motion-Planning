# Recurrent Models of Visual Attention
Volodymyr Mnih Nicolas Heess Alex Graves Koray Kavukcuoglu
[TOC]
Initially published 6/24/14

##  Summary
Processing large images with neural networks is hard. Lets make it easier by making a neural network model that only processes sections of an image at high resolution. 


### Architecture
This agent is modeled as a goal directed agent incentivized to select sections of the image to analyze. since it can only observe sections at a time it needs a long term memory storage to determine how to act at a given time. 
Its sensor takes a "glimpse" of the image, centered at a given location and with gradually reduced resolution further from the center of the glimpse. Its actions are directed with an incentive to correctly identify objects in the image, with an optional negative incentive for moving the glimpse to encourage expediency of recognition. 

### Comments
