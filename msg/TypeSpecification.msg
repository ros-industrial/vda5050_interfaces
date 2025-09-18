# TypeSpecification message
# This message describes general properties of the AGV type.

# Free text generalized series name as specifid by manufacturer
#
# Field type: Required
string seriesName

# Simplified description of AGV kinematics type.
# Allowed values: [DIFF, OMNI, THREEWHEEL]
#
# Field type: Required
string agvKinematic

# TODO: (@shawnkchan) check if these are meant to be enums
# Enum values for agvKinematic
#
# Diff: differential drive
string DIFF="DIFF"
#
# Omni: omni-directional vehicle
string OMNI="OMNI"
#
# Threewheel: three-wheel-driven vehicle or vehicle with similar kinematics
string THREEWHEEL="THREEWHEEL"

# TODO: (@shawnkchan) check if these are meant to be enums
# Simplified description of AGV class.
# Allowed values: [FORKLIFT, CONVEYOR, TUGGER, CARRIER]
#
# Field type: Required
string agvClass

# Enum values for agvClass
#
# Forklift: forklift
string FORKLIFT="FORKLIFT"
#
# Conveyor: AGV with conveyors on it
string CONVEYOR="CONVEYOR"
#
# Tugger: tugger
string TUGGER="TUGGER"
#
# Carrier: load carrier with or without lifting unit
string CARRIER="CARRIER"

# Maximum loadable mass [kg]
#
# Field type: Required
float64 maxLoadMass

# Simplified description of localization types.
# Example values: NATURAL, REFLECTOR, RFID, DMC, SPOT, GRID
# 
#Field type: Required
string[] localizationTypes

# Path planning types supported by the AGV, sorted by priority.
# Example values: PHYSICAL_LINE_GUIDED, VIRTUAL_LINE_GUIDED, AUTONOMOUS
#
# Field type: Required
string[] navigationTypes

# Free text human-readable description of the AGV type series
#
# Field type: Optional
string[<=1] seriesDescription

