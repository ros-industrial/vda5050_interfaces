# Trajectory described as a NURBS (points defining a spline) and part of the
# state message

# The number of control points that influence a point on the curve. Increasing
# this value increases the continuity of the curve
#
# Range: [1, infinity)
# Field type: Optional
OptionalFloat64 degree

# Sequence of parameter values determining how the contro point affects the
# NURBS curve
#
# Range: [0.0, 1.0]
# Field type: Required
float64[] knot_vector

# List of control points, from the beginning to end, defining the NURBS
#
# Fieldy type: Required
ControlPoint[] control_points

# Internal field to indicate if this message is initialized with a valid data
#
# Field type: Internal
bool value_initialized false
