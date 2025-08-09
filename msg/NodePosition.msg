# Defines a position on a map (on a floor in world coordinate system. Part of
# the state message

# X-position of the AGV in meters on the map in world coordinate system
#
# Field type: Required
float64 x

# Y-position of the AGV in meters on the map in world coordinate system
#
# Field type: Required
float64 y

# Orientation of the AGV in radians. If provided, the AGV must assume the
# orientation on this node
#    - If the previous edge, disallows rotation then the AGV must rotate on the
#      node
#    - If the following edge has a different orientation but disallows rotation
#      then the AGV must rotate on the node before entering the edge
#
# Range: [-PI, PI]
# Field type: Optional
OptionalFloat64 theta

# Indicates how accurate the AGV must be to drive over a node for it to count
# as traversed
#    - If deviation = 0, then the AGV must traverese the node within the
#      normal tolerance of the AGV manufacturer
#    - If deviation > 0, then the AGV can traverse the node within the
#      deviation radius
#
# Field type: Optional
OptionalFloat64 allowed_deviation_xy

# Indicates the allowed deviation from orientation
#
# Range: [0, PI]
# Field type: Optional
OptionalFloat64 allowed_deviation_theta

# Map ID in which the AGV is currently presently
#
# Field type: Required
string map_id ""

# Additional map information
#
# Field type: Optional
OptionalString map_description

# Internal field to indicate if this message is initialized with a valid data
#
# Field type: Internal
bool value_initialized false
