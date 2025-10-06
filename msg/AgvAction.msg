# Actions with parameters supported by the AGV.
# This includes standard actions specified in VDA5050 and manufacturer-specific
# actions.

# Unique type of action corresponding to action.actionType
#
# Field type: Required
string action_type

# Array of allowed scopes for using this action type.
# eg: ["INSTANT", "NODE"]
#
# Field type: Required
string[] action_scopes

# Enum values for allowed scopes in action_scopes.
#
# Instant: useable as an instant_action
string ACTION_SCOPES_INSTANT="INSTANT"
#
# Node: useable on nodes
string ACTION_SCOPES_NODE="NODE"
#
# Edge: useable on edges
string ACTION_SCOPES_EDGE="EDGE"

# TODO: (@sauk2) Is naming the field type as "FactsheetActionParameter" good enough for deconfliction?
# Array of parameters an action has. If not defined, the action has no
# parameters.
# NOTE: This message is different from the one used in the Order message.
#
# Field type: Optional
FactsheetActionParameter[] factsheet_action_parameters

# Free-form text: description of the result.
#
# Field type: Optional
string[<=1] result_description

# Free-form text: description of the action.
#
# Field type: Optional
string[<=1] action_description

# Array of possible blocking types for defined action.
#
# Field type: Optional
string[] blocking_types 

# Enum values for possible blocking types.
#
# None: action can be executed in parallel while driving.
string NONE="NONE"
#
# Soft: action can be executed in parallel but AGV cannot be driving.
string SOFT="SOFT"
#
# Hard: action cannot be executed in parallel and AGV cannot be driving.
string HARD="HARD"

