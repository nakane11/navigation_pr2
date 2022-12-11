uint8 SPOT=0
uint8 ELEVATOR=1
uint8 DOOR=2
uint8 BUTTON=3
uint8 ENTRANCE=4
uint8 OBJECT=5
uint8 PEOPLE=6

string name
string floor
# goal position
geometry_msgs/Pose pose

# type of the node
uint8 type
# threshold for moving
float32 range
# pause for some reason
float32 wait
# speak when asked to on the spot,
# or on the way
string description
# speak on arrival
string read_out
# keywords for considering the route
string[] keys

