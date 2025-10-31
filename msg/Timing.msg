# Timing information.

# Minimum interval sending order messages to the AGV [s]
#
# Field type: Required
float32 min_order_interval

# Minimum interval for sending state messages [s]
#
# Field type: Required
float32 min_state_interval

# Default interval for sending state messages. If not defined, the default value
# from the main document is used [s]
#
# Field type: Optional
float32[<=1] default_state_interval

# Default interval for sending messages on visualization topic [s]
#
# Field type: Optional
float32[<=1] visualization_interval

