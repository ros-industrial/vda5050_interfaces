# Edge message
# Defines a directional connection between two nodes.

# Unique edge identification.
#
# Field type: Required
string edge_id

# Number to track the sequence of nodes and edges in an order, simplifying updates.
# Distinguishes between nodes/edges that may be revisited in the same order.
# Resets when a new orderId is issued.
#
# Field type: Required
int32 sequence_id

# The nodeId of the start node for this edge.
#
# Field type: Required
string start_node_id

# The nodeId of the end node for this edge.
# 
# Field type: Required
string end_node_id

# Indicates if the edge is part of the base or horizon.
# "true" indicates that edge belongs to the base.
# "false" indicates that edge belongs to the horizon.
# 
# Field type: Required
bool released

# Array of actionIds to be executed on the edge.
# Array is empty if no actions are required.
# An action triggered by an edge will only be active for the time that the AGV is
# traversing it.
# When the AGV leaves the edge, the action will stop and the state before entering the
# edge will be restored.
#
# Field type: Required
Action[] actions

# Additional information about the edge.
#
# Field type: Optional
string edge_description

# Permitted maximum speed [m/s] on the edge.
# Speed is defined by the fastest measurement of the vehicle.
#
# Field type: Optional
float64[<=1] max_speed

# Permitted maximum height [m] of the vehicle including load on the edge.
#
# Field type: Optional
float64[<=1] max_height

# Permitted minimal height [m] of the load handling device on the edge.
#
# Field type: Optional
float64[<=1] min_height

# Orientation of the AGV on the edge [rad]
# If the AGV starts in a different orientation, rotate the vehicle on the edge to
# the desired orientation if rotationAllowed is true.
# If rotationAllowed is false, rotate before entering the edge.
# If it is not possible to rotate before entering the edge, reject the order.
# If no trajectory is defined, apply the rotation to the direct path betwen the two
# connecting nodes of the edge .
# If a trajectory is defined for the edge, apply the orientation to the trajectory.
#
# Field type: Optional
float64[<=1] orientation

# Defines if the orientation field is interpreted relative to the global
# project specific map coordinate system or tangential to the edge.
# If tangential to the edge, 0.0 = forwards, Pi = backwards.
# 
# Default value: TANGENTIAL
# Field type: Optional
string orientation_type "TANGENTIAL"

# Enum values for orientation_type
#
# GLOBAL: relative to the global project specific map coordinate system.
string GLOBAL="GLOBAL"
#
# TANGENTIAL: tangential to the edge.
string TANGENTIAL="TANGENTIAL"

# Sets direction at junctions for line-guided or wire-guided vehicles.
# To be defined initially (vehicle-individual).
#
# Field type: Optional
string[<=1] direction

# "true" indicates that rotation is allowed on the edge.
# "false" indicates that rotation is not allowed on the edge.
# No limit, if not set.
#
# Field type: Optional
bool[<=1] rotation_allowed

# The maximum rotation speed [rad/s].
# No limit, if not set.
#
# Field type: Optional
float64[<=1] max_rotation_speed

# The Trajectory object for this edge as a Non-Uniform Rational B-Spline (NURBS).
# Defines the path that the AGV should move between the start node and end node
# of the edge.
# This field can be omitted if the AGV cannot process trajectories or if the AGV
# plans its own trajectory.
#
# Field type: Optional
Trajectory[<=1] trajectory

# Length of the path from the start node to the end node [m].
# Used by line-guided AGVs to decrease their speed before reaching a stop position.
# 
# Field type: Optional
float64[<=1] length