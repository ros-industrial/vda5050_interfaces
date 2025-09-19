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
TypeSpecification type_specification  

# Parameters that specify the basic physical properties of the AGV
#
# Field type: Required
PhysicalParameters physical_parameters  

# Limits for length of identifiers, arrays, strings, and similar in MQTT communication
#
# Field type: Required
ProtocolLimits protocol_limits  

# Supported features of VDA5050 protocol
#
# Field type: Required
ProtocolFeatures protocol_features  

# Detailed definition of AGV geometry
#
# Field type: Required
AgvGeometry agv_geometry

# Abstract specification of load capabilities
#
# Field type: Required
LoadSpecification load_specification  

# Summary of current software and hardware versions on the vehicle 
# and optional network information
# 
# Field type: Required
VehicleConfig vehicle_config  

