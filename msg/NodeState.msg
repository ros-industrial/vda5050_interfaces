# Provides information about the node
#
# Part of the state message

# Unique ID of the the node
#
# Field type: Required
string node_id

# A sequence ID to differentiate multiple nodes with the same ID
#
# Field type: Required
uint32 sequence_id 0

# Additional node information
#
# Field type: Optional
string[<=1] node_description

# Position of the node on the active AGV map in world coordinate system
#
# Field type: Optional
NodePosition[<=1] node_position

# Defines if the node is part of the base or horizon
#     - True indicates node is part of the base
#     - False indicates node is part of the horizon
#
# Field type: Required
bool released
