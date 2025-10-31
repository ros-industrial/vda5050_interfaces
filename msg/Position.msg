# Message describing the pose of the AGV

# X-position in the AGV's coordinate system [m].
#
# Field type: Required
float64 x

# Y-position in the AGV's coordinate system [m].
#
# Field type: Required
float64 y

# Orientation of the wheel in the AGV coordinate system [rad].
# Necessary for fixed wheels
#
# Field type: Optional
float64[<=1] theta

