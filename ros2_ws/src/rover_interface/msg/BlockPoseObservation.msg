# The block position. Note that it does not include orientation 
# as this is not output by the YOLO-based CV node
geometry_msgs/PointStamped position

# The shape and colour of the block identified by the CV node
BlockShape shape 
BlockBinColor color

# The bounding box of the image
BoundingBox bbox
