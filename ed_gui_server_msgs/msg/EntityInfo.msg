string id
string type
float32 existence_probability
bool has_pose
geometry_msgs/Pose pose

# Number indicating the revision of the visual. If this number
# has changed, the visual has changed (and should be queried
# again).
# Special case: visual_revision == 0 means no visual available.
uint32 visual_revision
uint32 volumes_revision

Color color

Polygon polygon
