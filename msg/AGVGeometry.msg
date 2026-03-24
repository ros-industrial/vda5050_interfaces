# Message defining the geometry properties of the AGV.
# eg: outlines and wheel positions.

# Array of wheels, containing wheel arrangement and geometry.
#
# Field type: Optional
WheelDefinition[] wheel_definitions

# Array of AGV envelope curves in 2D.
# eg: the mechanical envelopes for unloaded and loaded state, the safety fields
# for different speed cases.
#
# Field type: Optional
Envelope2d[] envelopes2d

# Array of AGV envelope curves in 3D.
#
# Field type: Optional
Envelope3d[] envelopes3d

