Header header

string order_id ""

uint32 order_update_id 0

string[<=1] zone_set_id

string last_node_id

uint32 last_node_sequence_id 0

bool driving false

bool[<=1] paused

bool[<=1] new_base_request

float64[<=1] distance_since_last_node

string operating_mode

string OPERATING_MODE_AUTOMATIC="AUTOMATIC"
string OPERATING_MODE_SEMIAUTOMATIC="SEMIAUTOMATIC"
string OPERATING_MODE_MANUAL="MANUAL"
string OPERATING_MODE_SERVICE="SERVICE"
string OPERATING_MODE_TEACHIN="TEACHIN"

NodeState[] node_states

EdgeState[] edge_states

AGVPosition[<=1] agv_position

Velocity[<=1] velocity

Load[] loads

ActionState[] action_states

BatteryState battery_state

Error[] errors

Information[] information

SafetyState safety_state
