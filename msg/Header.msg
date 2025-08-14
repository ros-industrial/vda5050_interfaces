# The VDA 5050 header sent with each message. These are common fields
# across all message schemas

# The header ID of the message. It is defined per topic and incremented by
# 1 with each message sent but not necessarily received
uint32 header_id 0

# Timestamp in milliseconds in UTC indicating a specific point relative to
# a clock's 0 point
#
# Note: When using the serialization/deserialization utilities this will be
# converted to/from ISO8601 format YYYY-MM-DDTHH:mm:ss.ssZ
int64 timestamp

# Version of the protocol [Major].[Minor].[Patch] (e.g. 1.3.2)
string version

# Manufacturer of the AGV
string manufacturer

# Serial number of the AGV
string serial_number

