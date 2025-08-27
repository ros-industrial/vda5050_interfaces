# Error information conveyed by the AGV
#
# Part of the state message

# Type or name of the error
#
# Field type: Required
string error_type

# Array of references to identify the source of the error
#
# Field type: Optional
ErrorReference[] error_references

# Verbose description of the error
#
# Field type: Optional
string[<=1] error_description

# Severity of error
#
# Field type: Required
string error_level

# Enum values for errorLevel
#
# WARNING: AGV is ready to drive without human intervention
string ERROR_LEVEL_WARNING="WARNING"
# FATAL: AGV is not in running condition
string ERROR_LEVEL_FATAL="FATAL"
