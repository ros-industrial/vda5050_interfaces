# Describes the state of the action carried out by the AGV
#
# Part of the state message

# Unique identification for the action
#
# Field type: Required
string action_id

# The type of this action. Only for visualization purposes as the order
# already knows the type of the action
#
# Field type: Optional
string[<=1] action_type

# Additional information regarding the action
#
# Field type: Optional
string[<=1] action_description

# Current progress of the action
#
# Field type: Required
string action_status

# Enum values for actionStatus
#
# WAITING: Action was received by the the AGV but it has not reached the node
# or edge where it triggers it
string ACTION_STATUS_WAITING="WAITING"
#
# INITIALIZING: Action was triggered, preparation is underway
string ACTION_STATUS_INITIALIZING="INITIALIZING"
#
# RUNNING: Action is currently being executed
string ACTION_STATUS_RUNNING="RUNNING"
#
# PAUSED: Action is paused because of a pause, an instantAction or an external
# trigger
string ACTION_STATUS_PAUSED="PAUSED"
#
# FINISHED: Action has finished executing and result reported
string ACTION_STATUS_FINISHED="FINISHED"
#
# FAILED: Action could not be finished
string ACTION_STATUS_FAILED="FAILED"

# Description of the result. Errors will be transmitted in errors
#
# Field type: Optional
string[<=1] result_description
