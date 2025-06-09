# A message containing the connection information

# Message header
Header header

# The connection state of the AGV
uint8 connection_state

# ONLINE: The connection between the broker and AGV is active
uint8 CONNECTION_STATE_ONLINE=0

# OFFLINE: The connection between AGV and broker has gone offline in a
# coordinated way
uint8 CONNECTION_STATE_OFFLINE=1

# CONNECTIONBROKEN: The connection between AGV and broker has ended
# unexpectedly (e.g. used in MQTT last-will-message)
uint8 CONNECTION_STATE_CONNECTIONBROKEN=2

