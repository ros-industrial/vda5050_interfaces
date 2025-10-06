# NOTE: This message is different from ActionParameter.msg used in Order.msg 
# TODO: (sauk2) should we rename ActionParameter.msg (in the Order message) to OrderActionParameter.msg for consistency with how we're naming it here?
# Parameters that an action has.

# Key string for the parameter
#
# Field type: Required
string key

# Data type of value field in Order.ActionParameter
#
# Field type: Required
string value_data_type

# Enum values for value_data_type
#
string VALUE_DATA_TYPE_BOOL="BOOL"
#
string VALUE_DATA_TYPE_NUMBER="NUMBER"
#
string VALUE_DATA_TYPE_INTEGER="INTEGER"
#
string VALUE_DATA_TYPE_FLOAT="FLOAT"
#
string VALUE_DATA_TYPE_OBJECT="OBJECT"
#
string VALUE_DATA_TYPE_ARRAY="ARRAY"

# Free-form text: description of the parameter
#
# Field type: Optional
string[<=1] description

# If set to "true", then this is an optional parameter. 
#
# Field type: Optional
bool[<=1] is_optional

