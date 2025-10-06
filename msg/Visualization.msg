# A message containing AGV position and/or velocity for visualization
#
# Published by the AGV on topic /visualization for near-real time
# position update.

# Message header that contains the common fields required across all messages
Header header

# Position of the AGV
#
# Field type: Optional
AGVPosition[<=1] agv_position

# Velocity of the AGV in vehicle coordinates
#
# Field type: Optional
Velocity[<=1] velocity
