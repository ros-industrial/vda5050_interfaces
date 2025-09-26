# Trajectory described as a NURBS (points defining a spline)
#
# Part of the state message

# The number of control points that influence a point on the curve. Increasing
# this value increases the continuity of the curve
#
# **Note**: The v2.0.0 specification document implies this field to be mandatory
# while the v2.0.0 official schema assumes it to be optional. This package
# follows the specification document
#
# Range: [1, infinity)
# Default: 1.0
# Field type: Required
float64 degree 1.0

# Sequence of parameter values determining how the contro point affects the
# NURBS curve
#
# Range: [0.0, 1.0]
# Field type: Required
float64[] knot_vector

# List of control points, from the beginning to end, defining the NURBS
#
# Field type: Required
ControlPoint[] control_points
