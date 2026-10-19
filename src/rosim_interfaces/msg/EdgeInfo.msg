# One selectable edge of an imported workpiece model.
# edge_index is the TopExp_Explorer enumeration order, identical in viewer and
# planning nodes because both use the same librosim_occ code path.
string model_id
uint32 edge_index
geometry_msgs/Point[] polyline     # discretized polyline, world frame
float64 length                     # meters
bool closed
