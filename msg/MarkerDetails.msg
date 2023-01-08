# Agent ID
string id

# Structure to render
# details in rviz_markers/structures.yaml
# leave empty to remove the marker
string structure

# Optional colour to set marker componant scheme
# details in rviz_markers/colours.yaml
# leave empty to load default scheme
string colour
#std_msgs/ColorRGBA todo

# The subscribable location topic (use ~ for /ns/)
# leave empty to use only a mandatory given pose
string tf_source_topic
string tf_source_type

# Optional pose to send
geometry_msgs/Pose pose
