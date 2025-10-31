# Network message
# Information about the vehicle's network configuration.

# Array of Domain Name Servers (DNS) used by the vehicle.
#
# Field type: Optional
string[] dns_servers

# Array of Network Time Protocol (NTP) servers used by the vehicle.
#
# Field type: Optional
string[] ntp_servers

# A priori assigned IP address used to communicate with the MQTT broker.
# Note: This IP address should not be modified/changed during operations.
#
# Field type: Optional
string[<=1] local_ip_address

# The subnet mask used in the network configuration corresponding to the local IP address.
#
# Field type: Optional
string[<=1] netmask

# The default gateway used by the vehicle, corresponding to the local IP address.
#
# Field type: Optional
string[<=1] default_gateway

