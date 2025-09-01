# A message containing an instantAction information
#
# Published by Master Control on the topic /instantActions to inform an AGV
# of a set of actions to be executed as soon as they arrive

# Message header
Header header

# Array of actions that need to be performed immediately and 
# are not part of the regular order
Action[] actions