std_msgs/Header header
uint8 tracker_id

uint8 REQUEST_OFF = 0
uint8 REQUEST_TRACK = 1
uint8 REQUEST_ADJ = 2
uint8 request
uint32 fid
geometry_msgs/Point[2] bbox          # TRACK:pixel [up-left, down-right]
