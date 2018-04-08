#################### SYSTEM CONFIGURATION VARIABLES #######################

#change this to None to use the default mode linked to by libebolabot.so
#change this to "kinematic" to test in kinematic mode
#change this to "physical" if you are ready to test on the real robot
mode = 'physical' # 'physical'

#relative path, assumes everything is run from the Ebolabot directory
klampt_model = "klampt_models/ebolabot_col.rob"
#klampt_model = "klampt_models/hstar_amp1_col.rob"

def parse_args():
    import sys
    global mode,klampt_model
    if len(sys.argv) > 1:
        mode = sys.argv[1]
        if len(sys.argv) > 2:
            klampt_model = sys.argv[2]
