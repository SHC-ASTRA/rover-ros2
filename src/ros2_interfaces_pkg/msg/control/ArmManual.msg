# Topic: /arm/control/manual

# Axis control for rotation and speed multiplier
# Direction of rotation: 
# -1: Counterclockwise
#  0: No movement
# +1: Clockwise
# Speed multiplier:
# -2: 2x speed CCW
#  0: No movement
# +2: 2x speed CW
int32 axis0  # Axis 0 control
int32 axis1  # Axis 1 control
int32 axis2  # Axis 2 control
int32 axis3  # Axis 3 control

# Effector Roll movement direction
# -1: Counterclockwise
#  0: No movement
# +1: Clockwise
int32 effector_roll

# Effector Yaw movement direction
# -1: Counterclockwise
#  0: No movement
# +1: Clockwise
int32 effector_yaw

# Gripper control
# -1: Closing
#  0: Stopped
# +1: Opening
int32 gripper

# Linear Actuator movement direction
# -1: Retracting
#  0: Stopped
# +1: Extending
int32 linear_actuator

# Laser state
# 0: Off
# 1: On
int32 laser