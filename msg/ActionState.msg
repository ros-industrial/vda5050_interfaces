# Describes an action the state of the action being performed by the AGV

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
string action_status ""

# Enum values for action status
string ACTION_STATUS_WAITING="WAITING"
string ACTION_STATUS_INITIALIZING="INITIALIZING"
string ACTION_STATUS_RUNNING="RUNNING"
string ACTION_STATUS_PAUSED="PAUSED"
string ACTION_STATUS_FINISHED="FINISHED"
string ACTION_STATUS_FAILED="FAILED"

# Description of the result. Errors will be transmitted in errors
#
# Field type: Optional
string[<=1] result_declaration
