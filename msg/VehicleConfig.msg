# VehicleConfig message
# Details the software and hardware versions running on the vehicle,
# as well as a brief summary of network information.

# Array of key-value pair objects containing software and hardware information.
#
# Field type: Optional
VersionInfo[] versions

# Information about the vehicle's network connection.
# The listed information shall not be updated while the vehicle is operating.
#
# Field type: Optional
Network[<=1] network

