# Factsheet message
# Contains the AGV's capabilities, limits, features, and configuration 
# as required by the VDA5050 specification.

# Message header that contains the common fields required across all messages
#
# Field type: Required
Header header  

# Parameters that generally specify the class and the capabilities of the AGV
#
# Field type: Required
TypeSpecification typeSpecification  

# Parameters that specify the basic physical properties of the AGV
#
# Field type: Required
PhysicalParameters physicalParameters  

# Limits for length of identifiers, arrays, strings, and similar in MQTT communication
#
# Field type: Required
ProtocolLimits protocolLimits  

# Supported features of VDA5050 protocol
#
# Field type: Required
ProtocolFeatures protocolFeatures  

# Detailed definition of AGV geometry
#
# Field type: Required
AgvGeometry agvGeometry  

# Abstract specification of load capabilities
#
# Field type: Required
LoadSpecification loadSpecification  

# Summary of current software and hardware versions on the vehicle 
# and optional network information
# 
# Field type: Required
VehicleConfig vehicleConfig  

