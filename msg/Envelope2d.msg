# AGV envelope curves in 2D. 

# Name of the envelope curve set
# 
# Field type: Required
string set

# Envelope curve as an x/y-polygon.
# Polygon is assumed as closed and shall be non-self-intersecting.
#
# Field type: Required
PolygonPoint[] polygon_points

# Free-form text: description of envelope curve set.
#
# Field type: Optional
string[<=1] description


