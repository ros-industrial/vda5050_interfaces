# The message schema to communiccate from master control to the AGV

# Message header that contains the common fields required across all messages
#
# Field type: Required
Header header

# Field type: Required
string orderId

# Field type: Required
uint32 orderUpdateId

# Field type: Required
Node[] nodes

# Field type: Required
Edge[] edges

# Field type: Optional
string[<=1] zoneSetId