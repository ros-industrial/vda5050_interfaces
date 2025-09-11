# Defines the position of a node on a map in a project-specific world coordinate system

# X-position [m] on the map in reference to the map coordinate system.
# Precision is up to the specific implementation.
#
# Field type: Required
float64 x

# Y-position [m] on the map in reference to the map coordinate system.
# Precision is up to the specific implementation.
#
# Field type: Required
float64 y

# Unique identification of the map in which the position is referenced.
# Each map must share the same project-specific global origin of coordinates.
# Example: if an AGV uses an elevator, the node reappears on the target floor map.
#
# Field type: Required
string map_id

# Absolute orientation [rad] of the AGV at the node.
# Range: [-Pi ... Pi]
# Optional: vehicle can plan the path itself.
# If defined, the AGV must assume this orientation at the node.
# If previous edge disallows rotation, the AGV must rotate at this node.
# If following edge requires rotation, the AGV must rotate here before entering.
#
# Field type: Optional
float64[<=1] theta

# Indicates the radial distance [m] an AGV must be from a node for it to be counted as traversed.
# If = 0: no deviation allowed (strict tolerance).
# If > 0: node is considered traversed if the AGV passes within this radius.
#
# Field type: Optional
float64 allowed_deviation_xy

# Allowed deviation for theta angle [rad].
# Range: [0 ... Pi]
# Defines acceptable range for an AGV's heading at this node:
#   theta - allowedDeviationTheta  to  theta + allowedDeviationTheta
#
# Field type: Optional
float64 allowed_deviation_theta

# Additional information about the map.
#
# Field type: Optional
string map_description
