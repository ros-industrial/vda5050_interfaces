# Defines a position of the AGV on a map in world coordinates

# X-position (in meters) on the map in reference to the map coordinate system
# Precision is implementation-specific
#
# Field type: Required
float64 x

# Y-position (in meters) on the map in reference to the map coordinate system
# Precision is implementation-specific
#
# Field type: Required
float64 y

# Orientation of the AGV in radians in range [-PI, PI]
#
# Field type: Required
float64 theta

# Unique ID of the map inwhich the position is referenced
#
# Field type: Required
string map_id ""

# Additional information about the map
#
# Field type: Optional
string[<=1] map_description

# Flag to indicate the position initialization of the AGV
bool position_initialized false

float64[<=1] localization_score

float64[<=1] deviation_range
