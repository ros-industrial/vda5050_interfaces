# Message describing the wheel arrangement and geometry

# Wheel type.
#
# Field type: Required
string type

# Enum values for the wheel type
# 
string DRIVE="DRIVE"
#
string CASTER="CASTER"
#
string FIXED="FIXED"
#
string MECANUM="MECANUM"

# If set to "true", the wheel is actively driven.
#
# Field type: Required
bool is_active_driven

# If set to "true", the wheel is actively steered.
#
# Field type: Required
bool is_active_steered

# Pose of the AGV according to its coordinate system.
#
# Field type: Required
Position position

# Nominal diameter of the wheel [m].
#
# Field type: Required
float64 diameter

# Nominal width of the wheel [m].
#
# Field type: Required
float64 width

# TODO: Confirm whether this should be a bounded array or not, since it has a default value
# Nominal displacement of the wheel's center to the rotation points.
# This field is necessary for caster wheels.
# 
# Default: 0.0
# Field type: Optional
float64 center_displacement 0.0

# Free-form text: can be used by the manufacturer to define constraints.
#
# Field type: Optional
string[<=1] constraints

