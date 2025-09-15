# A message describing an action that the AGV can perform

# Name of action as described in the first column of Actions and Parameters
# Identifies the function of the action
#
# Field Type: Required
string action_type

# Unique ID to identify the action and map it to the actionState in
# the State message
#
# Field Type: Required
string action_id

# Regulates if the action is allowed to be executed during movement and/or 
# paralel to other actions.
#
# Field Type: Required
string blocking_type

# Enum values for blockingType
#
# NONE: action can happen in parallel with others, including movement
string NONE="NONE"
#
# SOFT: action can happen simultaneously with thers, but not while moving
string SOFT="SOFT"
#
# HARD: no other action can be performed while this action is running
string HARD="HARD"

# Additional information on the action
#
# Field Type: Optional
string[<=1] action_description

# Array of actionParameter objects for the indicated action. 
# If not defined, the action has no parameters
# eg: deviceId, loadId, external Triggers
# 
# Field Type: Optional
ActionParameter[] action_parameters 


