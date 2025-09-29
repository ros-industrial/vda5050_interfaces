# Point of reference for the location of the bounding box. The point of
# reference is always the center of the bounding box bottom surface
# (at height = 0) and is described in coordinates of the AGV’s coordinate
# system
#
# Part of the state message

# X-coordinate of the point of reference
#
# Field type: Required
float64 x

# Y-coordinate of the point of reference
#
# Field type: Required
float64 y

# Z-coordinate of the point of reference
#
# Field type: Required
float64 z

# Orientation of the bounding box
#
# Field type: Optional
float64[<=1] theta
