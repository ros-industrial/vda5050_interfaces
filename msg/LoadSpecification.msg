# Message specifying the load handling and supported load types of the AGV

# Array of load positions / load handling devices.
# This array contains the valid values for "state.loads[].loadPosition" and for the
# action parameter "lhd" of pick-and-drop actions.
# If this array doesn't exist or is empty, the AGV has no load handling device.
#
# Field type: Optional
string[] load_positions

# Array of load sets that can be handled by the AGV.
#
# Field type: Optional
LoadSet[] load_sets

