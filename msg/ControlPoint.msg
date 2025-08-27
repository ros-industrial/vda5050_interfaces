# Control point describing a trajectory in NURBS
#
# Part of the state message

# X-coordinate described in the world coordinate frame
#
# Field type: Required
float64 x

# Y-coordinate described in the world coordinate frame
#
# Field type: Required
float64 y

# The weight with which this control point pulls on the curve
#
# Range: (0, infinity)
# Default: 1.0
# Field type: Optional
float64 weight 1.0
