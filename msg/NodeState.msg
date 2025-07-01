# Contains node information for state message

# Unique ID of the the node
#
# Field type: Required
string node_id

# A sequence ID to differentiate to multiple node with the same ID
#
# Field type: Required
int32 sequence_id 0

# Additional node information
#
# Field type: Optional
OptionalString node_description

# Position of the node on the active AGV map in world coordinate system
#
# Field type: Optional
NodePosition node_position

# Defines if the node is part of the base or horizon
#     - True indicates node is part of the base
#     - False indicates node is part of the horizon
#
# Field type: Required
bool released
