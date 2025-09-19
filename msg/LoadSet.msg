# LoadSet message
# Represents one load set supported by the AGV.

# Unique name of the load set (e.g., DEFAULT, SET1, etc.)
#
# Field type: Required
string set_name

# Type of load (e.g., EPAL, XLT1200, etc.)
#
# Field type: Required
string load_type

# Array of load positions between load handling devices that this load set is valid for.
# If this parameter does not exist or is empty, this load set is valid for all load
# handling devices on this AGV.
#
# Field type: Optional
string[] load_positions

# Bounding box reference as defined in parameter loads[] in the state message.
#
# Field type: Optional
BoundingBoxReference[<=1] bounding_box_reference

# Load dimensions as defined in parameter loads[] in the state message.
#
# Field type: Optional
LoadDimensions[<=1] load_dimensions

# Maximum weight of this load type [kg].
#
# Field type: Optional
float64[<=1] max_weight

# Minimum allowed height for handling this load type [m].
# References bounding_box_reference.
#
# Field type: Optional
float64[<=1] min_load_handling_height

# Maximum allowed height for handling this load type [m].
# References bounding_box_reference.
#
# Field type: Optional
float64[<=1] max_load_handling_height

# Minimum allowed depth for handling this load type [m].
# References bounding_box_reference.
#
# Field type: Optional
float64[<=1] min_load_handling_depth

# Maximum allowed depth for handling this load type [m].
# References bounding_box_reference.
#
# Field type: Optional
float64[<=1] max_load_handling_depth

# Minimum allowed tilt for handling this load type [rad].
#
# Field type: Optional
float64[<=1] min_load_handling_tilt

# Maximum allowed tilt for handling this load type [rad].
#
# Field type: Optional
float64[<=1] max_load_handling_tilt

# Maximum allowed speed for handling this load type [m/s].
#
# Field type: Optional
float64[<=1] agv_speed_limit

# Maximum allowed acceleration for handling this load type [m/s²].
#
# Field type: Optional
float64[<=1] agv_acceleration_limit

# Maximum allowed deceleration for handling this load type [m/s²].
#
# Field type: Optional
float64[<=1] agv_deceleration_limit

# Approximate time for picking up the load [s].
#
# Field type: Optional
float64[<=1] pick_time

# Approximate time for dropping the load [s].
#
# Field type: Optional
float64[<=1] drop_time

# Free-form description of this load handling set.
#
# Field type: Optional
string[<=1] description

