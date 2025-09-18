# PhysicalParameters message
# Describes physical properties of the AGV.

# Minimal controlled continuous speed of the AGV [m/s]
#
# Field type: Required
float64 speedMin

# Maximum speed of the AGV [m/s]
#
# Field type: Required
float64 speedMax

# Maximum acceleration with maximum load [m/s²]
#
# Field type: Required
float64 accelerationMax

# Maximum deceleration with maximum load [m/s²]
#
# Field type: Required
float64 decelerationMax

# Minimum height of AGV [m]
#
# Field type: Required
float64 heightMin

# Maximum height of AGV [m]
#
# Field type: Required
float64 heightMax

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
float64 angularSpeedMin

# Maximum rotation speed of the AGV [rad/s]
#
# Field type: Optional
float64 angularSpeedMax

