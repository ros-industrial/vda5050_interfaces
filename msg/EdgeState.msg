# Provides information about the edge
#
# Part of the state message

# Unique ID of the edge
#
# Field type: Required
string edge_id

# A sequence ID to differentiate multiple edges with the same ID
#
# Field type: Required
int32 sequence_id

# Additional edge information
#
# Field type: Optional
string[<=1] edge_description

# Defines if the edge is part of the base or horizon
#     - True indicates edge is part of the base
#     - False indicates edge is part of the horizon
#
# Field type: Required
bool released

# The trajectory in NURBS, defined as starting from the point the AGV starts
# to enter the edge until the point where it reports the next node as traversed
#
# Field type: Optional
Trajectory[<=1] trajectory
