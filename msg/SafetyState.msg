# Provides information about the safety state of the AGV
#
# Part of the state message

# Emergency stop status
#
# Field type: Required
string e_stop

# Enum values for eStop
#
# AUTOACK: Auto-acknowledgeable E-Stop is activated
string E_STOP_AUTOACK="AUTOACK"
#
# MANUAL: E-Stop has to be acknowledged manually at the vehicle
string E_STOP_MANUAL="MANUAL"
#
# REMOTE: facility E-Stop has to be acknowledged remotely
string E_STOP_REMOTE="REMOTE"
#
# NONE: No E-Stop activated
string E_STOP_NONE="NONE"

# Protective field violation
#
# Field type: Required
bool field_violation
