# A message containing the connection information

# Message header
Header header

# The connection state of the AGV
string connection_state ""

# Enum values for connectionState
#
# ONLINE: The connection between the broker and AGV is active
string ONLINE="ONLINE"
#
# OFFLINE: The connection between AGV and broker has gone offline in a
# coordinated way
string OFFLINE="OFFLINE"
#
# CONNECTIONBROKEN: The connection between AGV and broker has ended
# unexpectedly (e.g. used in MQTT last-will-message)
string CONNECTIONBROKEN="CONNECTIONBROKEN"
