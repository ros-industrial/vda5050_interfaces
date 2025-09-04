# Message that enables users to select between array, boolean,
# number, string, object for the parameterValue field in the
# actionParameter message.
#
# Used to determine which of the next *_value fields are set

# Type of this parameter
#
# Field Type: Required
uint8 type

# Enum values for type
uint8 PARAMETER_BOOL=1
uint8 PARAMETER_INTEGER=2
uint8 PARAMAETER_DOUBLE=3
uint8 PARAMETER_STRING=4
uint8 PARAMETER_BOOL_ARRAY=5
uint8 PARAMETER_INTEGER_ARRAY=6
uint8 PARAMETER_DOUBLE_ARRAY=7
uint8 PARAMETER_STRING_ARRAY=8

bool bool_value

int64 integer_value

float64 float_value

string string_value

# Homogenous arrays
#
bool[] bool_array_value

int64[] integer_array_value

float64[] double_array_value

string[] string_array_value

# TODO: What about heterogenous arrays?
# TODO: Enable objects to be stored
