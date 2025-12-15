# Unique identifier used to distinguish between different blocks
uint8 id

# Smoothed position of the block from multiple observations 
geometry_msgs/PointStamped position

# Block features
BlockShape shape
BlockBinColor color

# State for tracking whether a given block has been successfully collected
bool state 
bool COLLECTED = true
bool NOT_COLLECTED = false
