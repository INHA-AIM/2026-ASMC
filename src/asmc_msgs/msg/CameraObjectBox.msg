# One 2D detection in original-image pixels of the camera named by the
# enclosing CameraObjectBoxArray header.frame_id.
float32 xmin
float32 ymin
float32 xmax
float32 ymax

# Normalized group shared by all consumers: person, vehicle or obstacle.
# person covers pedestrians, cyclists and children; vehicle covers cars,
# trucks, buses and motorcycles; obstacle is any other object whose kind is
# not distinguished.
string class_name
# Raw label emitted by the detector model (for example truck, bicycle or a
# fine-tuned MORAI label). Model-dependent: use it for diagnostics and
# refinement, not as the safety-relevant class.
string source_class_name

float32 confidence
bool confidence_valid
