# The message schema to communicate from master control to the AGV

# Message header that contains the common fields required across all messages
#
# Field type: Required
Header header

# Order identification used to identify multiple order messages that belongs
# to the same order.
#
# Field type: Required
string orderId

# Order update identification used to identify multiple order messages that 
# belong to the same order.
#
# Field type: Required
uint32 orderUpdateId

# An array of Node messages to be traversed for fulfilling the order.
# A valid order consists of at least one Node. 
# For the case of only one Node, leave the Edge array empty.
#
# Field type: Required
Node[] nodes

# An array of Edge objects to be traversed for fulfilling the order.
#
# Field type: Required
Edge[] edges

# A unique identifier of the zone set that the AGV has to use for navigation
# or that was used by master control for planning. 
# If no zones are used, leave empty.
#
# Field type: Optional
string[<=1] zoneSetId