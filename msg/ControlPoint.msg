# ControlPoint message
# Defines a single control point of a Non-Uniform Rational B-Spline (NURBS) trajectory.

# X coordinate in the world coordinate system.
#
# Field type: Required
float64 x

# Y coordinate in the world coordinate system.
#
# Field type: Required
float64 y

# Weight of the control point.
# Range: (0 ... infinity)
# Defines how strongly this control point pulls on the curve.
#
# Default value: 1.0
# Field type: Optional
float64 weight 1.0
