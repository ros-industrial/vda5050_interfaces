# A message containing the state information of the AGV
#
# Published by the AGV on the topic /state to inform the master control of the
# current AGV state

# Message header that contains the common fields required across all messges
Header header

# Unique order ID of the current order or the previous finished order. The
# orderId is kept until a new order is received
#
# Default value: Empty string if no previous orderId is available
# Field type: Required
string order_id ""

# Order Update Identification to identify that an order update has been
# accepted by the AGV.
#
# Default value: 0 if no previous orderUpdateId is available
# Field type: Required
uint32 order_update_id 0

# Unique ID of the zone set that the AGV currently uses for path planning. Must
# be the same as the one used in the order, otherwise the AGV is to reject the
# order
#
# Field type: Optional
OptionalString zone_set_id

# Node ID of last reached node or, if AGV is currently on a node, current node
#
# Default value: Empty string if no ID is available
# Field type: Required
string last_node_id

# Sequence ID of the last reached node or, if the AGV is currently on a node,
# ID of current node
#
# Default value: 0 if no ID is available
# Field type: Required
uint32 last_node_sequence_id 0

# Indicates the driving and/or rotation movements of the AGV. Does not include
# lift movements
#
# Field type: Required
bool driving false

# Indicates the paused state of the AGV, either because of a physical button
# push or an instantAction, and that it can resume the order
#
# Field type: Optional
OptionalBool paused

# Request a new base from the master control when the AGV is almost at the end
# of its base and about to reduce speed
#
# Field type: Optional
OptionalBool new_base_request

# Distance in meters, used by line-guided AGVs to indicate distance it has drove
# past the lastNodeID
#
# Field type: Optional
OptionalFloat64 distance_since_last_node

# Current operating mode of the AGV
#
# Field type: Required
string operating_mode

# Enum values for operatingMode field
string OPERATING_MODE_AUTOMATIC="AUTOMATIC"
string OPERATING_MODE_SEMIAUTOMATIC="SEMIAUTOMATIC"
string OPERATING_MODE_MANUAL="MANUAL"
string OPERATING_MODE_SERVICE="SERVICE"
string OPERATING_MODE_TEACHIN="TEACHIN"

# Array of nodes that need to be traversed for fulfilling the order
#
# Default value: Empty list if AGV is idle
# Field type: Required
NodeState[] node_states

# Array of edges that need to be traversed for fulfilling the order
#
# Default value: Empty list if AGV is idle
# Field type: Required
EdgeState[] edge_states

# Position of the AGV
#
# Field type: Optional
AGVPosition agv_position

# Velocity of the AGV in vehicle coordinates
#
# Field type: Optional
Velocity velocity

# Array for information about the loads the AGV is currently carrying. If an
# empty list is received, then the master control should assume the AGV
# can reason about its load state and that it currently does not carry any
# load
#
# Field type: Optional
Load[] loads

# Array with a list of current actions and actions to be finished. This may
# include actions from previous nodes that are still in progress. When an
# action is completed, an updated state message must be published with
# actionStatus set to finished. The actionStates are kept until a new order
# is received
#
# Field type: Required
ActionState[] action_states

# Battery information of the AGV
#
# Field type: Required
BatteryState battery_state

# Array of errors. Errors are kept until resolution
#
# Field type: Required
Error[] errors

# Array of information messages for visualization/debugging. There no
# specification when these messages are deleted
#
# Field type: Optional
Information[] information

# Safety state information of the AGV
#
# Field type: Required
SafetyState safety_state
