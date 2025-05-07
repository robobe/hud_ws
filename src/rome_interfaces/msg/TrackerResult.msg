std_msgs/Header header
uint8 tracker_id
uint32 fid
uint8 STATUS_OFF = 0
uint8 STATUS_TRACK = 1
uint8 STATUS_LOST = 2
uint8 status
uint8 quality                     # normalize to 0-100 
geometry_msgs/Point angle         # offset from frame center to bounding_box center in degree
uint8 BBOX_UP_LEFT = 0
uint8 BBOX_DOWN_RIGHT = 1
geometry_msgs/Point[2] bbox      # pixel [up-left, down-right]
