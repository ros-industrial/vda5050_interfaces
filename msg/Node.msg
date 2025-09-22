# Defines a single node in an Order message.

# Unique node identification
#
# Field type: Required
string node_id

# Number to track the sequence of nodes and edges in an order and to simplify order updates.
# Distinguishes between nodes that may be passed more than once in the same order.
# Runs across all nodes/edges of the same order and resets when a new orderId is issued.
#
# Field type: Required
uint32 sequence_id

# "true" indicates that the node is part of the base
# "false" indicates that the node is part of the horizon
#
# Field type: Required
bool released

# Array of actions to be executed at this node.
# Empty array if no actions are required.
#
# Field type: Optional
Action[] actions

# Node position, required only for vehicle-types that need node positions.
# (e.g., not required for line-guided vehicles)
#
# Field type: Optional
NodePosition[<=1] node_position

# Additional information about the node
#
# Field type: Optional
string[<=1] node_description

