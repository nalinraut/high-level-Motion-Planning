"""Some constants for the rethink electric gripper model.
Mostly """

#The hardware name
gripper_name = 'rethink electric gripper'

#The Klamp't model name
klampt_model_name = 'rethink_electric_gripper.rob'

#the number of Klamp't model DOFs
numDofs = 43

#The number of command dimensions
numCommandDims = 1

#The names of the command dimensions
commandNames = ['open-amount']

#default postures
openCommand = [1]
closeCommand = [0]

#named preset list
presets = {'open':openCommand,
           'closed':closeCommand,
           }

#range of postures
commandMinimum = [0]
commandMaximum = [1]

#range of valid command velocities
commandMinimumVelocity = [-2]
commandMaximumVelocity = [2]

def commandToConfig(command):
    """Given a rethink parallel jaw gripper command vector, in the range
    range [0] (closed) to [1] (open), returns the gripper configuration for
    the klampt model.
    """
    value = command[0]
    return [0,value*0.03,-value*0.03]

def configToCommand(config):
    """Given a gripper configuration for the klampt model, returns the
    closest command that corresponds to this configuration.  Essentially
    the inverse of commandToConfig(). 
    """
    value = (config[1]-config[2])*0.5/0.03
    return [value]
