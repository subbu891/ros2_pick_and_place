# One object found by the camera.
string id                          # e.g. "red_0"
string color                       # "red", "green" or "blue"
geometry_msgs/PoseStamped pose     # where it is (position + orientation) and in which frame
float32 pixel_area                 # blob size in the image, useful for debugging
float32 confidence                 # 0..1
