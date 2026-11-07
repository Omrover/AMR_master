# Estado de la misión Pick & Place publicado por el Behavior Tree.
std_msgs/Header header

uint8 IDLE = 0
uint8 NAVIGATING_PICKUP = 1
uint8 ACQUIRING = 2
uint8 PICKING = 3
uint8 NAVIGATING_DROPOFF = 4
uint8 PLACING = 5
uint8 DOCKING = 6
uint8 WAITING_BATTERY = 7
uint8 COMPLETED = 8
uint8 FAILED = 9

string mission_id
uint8 stage
string stage_name
string detail
float32 progress            # 0..1
