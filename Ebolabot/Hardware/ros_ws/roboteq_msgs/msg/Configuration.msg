# Configuration Message  
Header header

# control
uint8 control_mode
uint8 pid_kp 
uint8 pid_ki
uint8 pid_kd
uint8 int_cap
uint8 error_detection

# limits
float32 current_limit
uint8 ovl 
uint8 uvl 
uint32 max_rpm 
uint32 motor_accel
uint32 motor_decel

# intrinsic
uint8 pole_pairs

# triggers 
bool save_eeprom
bool e_stop
bool e_release
