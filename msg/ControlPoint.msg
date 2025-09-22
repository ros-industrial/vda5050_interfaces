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

# TODO: (@sauk2) Want to check with you, since this field is optional but has a default value, we don't need to set this as a bounded array, right?
# The weight with which this control point pulls on the curve
#
# Range: (0, infinity)
# Default: 1.0
# Field type: Optional
float64 weight 1.0

