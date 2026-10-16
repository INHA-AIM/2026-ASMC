# One 2D detection in original-image pixels of the camera named by the
# enclosing CameraObjectBoxArray header.frame_id.
float32 xmin
float32 ymin
float32 xmax
float32 ymax

# Canonical mission16: vehicle / dynamic_obstacle / static_obstacle.
# Legacy/experimental profiles may emit raw labels; inspect producer config.
# Group is mobility eligibility, not measured motion or collision risk.
string class_name
# Original detector label, preserved across grouping. Consumers apply their
# own eligibility policy; a label alone does not establish collision risk.
string source_class_name

float32 confidence
bool confidence_valid
