# Defines the position on the map in world coordinate system. Part of
# the state message

# X-position of the AGV in meters on the map in world coordinate system
#
# Field type: Required
float64 x

# Y-position of the AGV in meters on the map in world coordinate system
#
# Field type: Required
float64 y

# Orientation of the AGV in radians
#
# Field type: Required
float64 theta

# Map ID in which the AGV is currently presently
#
# Field type: Required
string map_id ""

# Additional map information
#
# Field type: Optional
OptionalString map_description

# Indicates if the AGV's position is initialized
#     - True if position is initialized
#     - False if position is not initialized
#
# Field type: Required
bool position_initialized false

# Desribes the quality of localization and can be used to describe the accuracy
# of the current position information. Primarily for logging and visualization
# purposes
#     - 0.0 denotes position unknown
#     - 1.0 denotes position known
#
# Field type: Optional
OptionalFloat64 localization_score

#
OptionalFloat64 deviation_range

bool value_initialized false
