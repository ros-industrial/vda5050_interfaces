# Messages carrying information for visualization or debugging
#
# Part of the state message

# Type or name of the info
#
# Field type: Required
string info_type

# Array of references to identify the source of the info
#
# Field type: Optional
InfoReference[] info_references

# Verbose description of the info
#
# Field type: Optional
string[<=1] info_description

# Type of info
#
# Field type: Required
string info_level

# Enum values for infoLevel
#
# INFO: Used for visualization
string INFO_LEVEL_INFO="INFO"
#
# DEBUG: Used for debugging
string INFO_LEVEL_DEBUG="DEBUG"
