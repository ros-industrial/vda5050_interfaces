# NOTE: This message is different from ActionParameter.msg used in Order.msg 
# TODO: (sauk2) should we rename ActionParameter.msg to OrderActionParameter.msg for consistency?
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
string BOOL="BOOL"
#
string NUMBER="NUMBER"
#
string INTEGER="INTEGER"
#
string FLOAT="FLOAT"
#
string OBJECT="OBJECT"
#
string ARRAY="ARRAY"

# Free-form text: description of the parameter
#
# Field type: Optional
string[<=1] description

# If set to "true", then this is an optional parameter. 
#
# Field type: Optional
bool[<=1] is_optional