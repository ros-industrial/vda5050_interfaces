# Provides information related to the battery
#
# Part of the state message

# State of charge in percent (%). If the AGV only provides values for good
# or bad battery levels, these will be indicated as 20% (bad) and 80% (good)
#
# Field type: Required
float64 battery_charge

# Value of the battery voltage
#
# Field type: Optional
float64[<=1] battery_voltage

# State of health of the battery in percent
#
# Field type: Optional
int8[<=1] battery_health

# Charging state of the AGV. True if charging is in progress
#
# Field type: Required
bool charging

# Estimated reach with the current state of charge in meters
#
# Field type: Optional
uint32[<=1] reach
