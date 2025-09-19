# This message defines the maximum lengths of strings used in MQTT communication
# and other identifiers within the VDA5050 specification.

# Maximum MQTT message length.
#
# Field type: Optional
uint32[<=1] msg_len

# Maximum length of serial number part in MQTT topics.
# Affected parameters:
# - order.serialNumber
# - instantActions.serialNumber
# - state.serialNumber
# - visualization.serialNumber
# - connection.serialNumber
#
# Field type: Optional
uint32[<=1] topic_serial_len

# Maximum length of all other parts in MQTT topics.
# Affected parameters:
# - order.timestamp
# - order.version
# - order.manufacturer
# - instantActions.timestamp
# - instantActions.version
# - instantActions.manufacturer
# - state.timestamp
# - state.version
# - state.manufacturer
# - visualization.timestamp
# - visualization.version
# - visualization.manufacturer
# - connection.timestamp
# - connection.version
# - connection.manufacturer
#
# Field type: Optional
uint32[<=1] topic_elem_len

# Maximum length of ID strings.
# Affected parameters:
# - order.orderId
# - order.zoneSetId
# - node.nodeId
# - nodePosition.mapId
# - action.actionId
# - edge.edgeId
# - edge.startNodeId
# - edge.endNodeId
#
# Field type: Optional
uint32[<=1] id_len

# Maximum length of enum and key strings.
# Affected parameters:
# - action.actionType
# - action.blockingType
# - edge.direction
# - actionParameter.key
# - state.operatingMode
# - load.loadPosition
# - load.loadType
# - actionState.actionStatus
# - error.errorType
# - error.errorLevel
# - errorReference.referenceKey
# - info.infoType
# - info.infoLevel
# - safetyState.eStop
# - connection.connectionState
#
# Field type: Optional
uint32[<=1] enum_len

# Maximum length of loadId strings.
#
# Field type: Optional
uint32[<=1] load_id_len

# If true, ID strings must contain numerical values only.
#
# Field type: Optional
bool[<=1] id_numerical_only
