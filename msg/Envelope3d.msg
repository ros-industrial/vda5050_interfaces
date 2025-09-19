# Message for an envelope curve in 3D

# Name of the envelope curve set.
#
# Field type: Required
string set

# Format of the data, eg: DXF.
#
# Field type: Required
string format

# TODO: (@shawnkchan) I assume we will leave this as a string and let the user handle the data format on their end?
# 3D-envelope curve data as a string.
# Format of this data is specified in the 'format' field, and should be
# handled accordingly.
#
# Field type: Optional
string[<=1] data 

# Protocol and URL definition for downloading the 3D-envelope curve data.
# eg: ftp://xxx.yyy.com/ac4dgvhoif5tghji
#
# Field type: Optional
string[<=1] url

# Free-form text: description of envelope curve set.
#
# Field type: Optional
string[<=1] description

