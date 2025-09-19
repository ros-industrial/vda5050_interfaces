# PhysicalParameters message
# Describes physical properties of the AGV.

# Minimal controlled continuous speed of the AGV [m/s]
#
# Field type: Required
float64 speed_min

# Maximum speed of the AGV [m/s]
#
# Field type: Required
float64 speed_max

# Maximum acceleration with maximum load [m/s²]
#
# Field type: Required
float64 acceleration_max

# Maximum deceleration with maximum load [m/s²]
#
# Field type: Required
float64 deceleration_max

# Minimum height of AGV [m]
#
# Field type: Required
float64 height_min

# Maximum height of AGV [m]
#
# Field type: Required
float64 height_max

# Width of AGV [m]
#
# Field type: Required
float64 width

# Length of AGV [m]
#
# Field type: Required
float64 length

# Minimal controlled continuous rotation speed of the AGV [rad/s]
#
# Field type: Optional
float64[<=1] angular_speed_min

# Maximum rotation speed of the AGV [rad/s]
#
# Field type: Optional
float64[<=1] angular_speed_max

