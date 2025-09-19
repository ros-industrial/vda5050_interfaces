# Supported or requireed optional parameters.

# Full name of optional parameter.
# eg: "order.nodes.nodePosition.allowedDeviationTheta".
#
# Field type: Required
string parameter

# Type of support for the optional parameter.
# 
# Field type: Required
string support

# Enum values for support field.
#
# Supported: optional parameter is supported like specified.
string SUPPORTED="SUPPORTED"
#
# Required: optional parameter is required for proper AGV operation.
string REQUIRED="REQUIRED"

# Free-form text: description of optional parameter.
# eg: 
#   - Reason why the optional parameter direction is necessary for this AGV type.
#   - The parameter nodeMarker shall contain unsigned integers only.
#   - NURBS support is limited to straight lines and circular segments.
#
# Field type: Optional
string[<=1] description