# defines a YOLO detection result

# class probability
int32 class_id
string class_name
float64 score

# ID for tracking
string id

# depth value from 2d bounding box center
float64 depth

geometry_msgs/Point point_3d

# 2D bounding box surrounding the object in pixels
yolo_msgs/BoundingBox2D bbox