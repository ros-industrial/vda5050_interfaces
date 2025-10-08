# Information about the load the AGV is carrying
#
# Part of the state message

# Unique ID of the load
#     - Empty field if the AGV can identify the load but hasn't identified
#       it yet
#     - Optional if the AGV cannot identify the load
#
# Field type: Optional
string[<=1] load_id

# Type of the load
#
# Field type: Optional
string[<=1] load_type

# Indicates which load handling or carrying unit of the AGV is used (eg, front,
# back, etc) if the AGV has multiple positions
#
# Field type: Optional
string[<=1] load_position

# Point of reference for the location of the bounding box. It is always the
# center of the bounding box's bottom surface and is described in AGV's
# coordinate system
#
# Field type: Optional
BoundingBoxReference[<=1] bounding_box_reference

# Dimensions of the load's bounding box in meters
#
# Field type: Optional
LoadDimensions[<=1] load_dimensions

# Absolute weight of the load in kilograms
#
# Field type: Optional
float64[<=1] weight
