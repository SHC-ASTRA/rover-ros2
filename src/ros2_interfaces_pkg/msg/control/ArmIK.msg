# Topic: /arm/control/ik

# User's input on x, y, z axis described as a vector
geometry_msgs/Vector3 movement_vector  # [x, y, z] values for the arm's movement in 3D space

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