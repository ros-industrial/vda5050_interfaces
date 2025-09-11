# Defines the curve on which the AGV should move between startNodeId
# and endNode in an Edge message, as a Non-Uniform Rational B-Spline (NURBS).

# Knot vector for the NURBS curve.
# Range: [0.0 ... 1.0]
# Sequence of parameter values that determine where and how control points affect the curve.
# Size = number of control points + degree + 1
#
# Field type: Required
float64[] knot_vector

# List of control points that define the NURBS curve, including the beginning and end points.
# Each entry is a ControlPoint message.
#
# Field type: Required
ControlPoint[] control_points

# Degree of the NURBS curve.
# Range: [1 ... infinity]
# Defines the number of control points that influence any given point on the curve.
# Increasing the degree increases continuity.
#
# Default value is 1 if not defined.
# Field type: Optional
# TODO: @shawnkchan should the default value be a float?
float64 degree 1
