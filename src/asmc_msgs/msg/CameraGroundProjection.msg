# Internal, non-latched projection context for visualization and BBox adapters.
# Computed once by the family
# worker; consumers MUST NOT query IMU again or apply this plane twice.
# Header stamp is the latest image stamp in this exact three-view bundle.
std_msgs/Header header
string producer_session_id
uint64 generation
string projection_session_id
string config_id
bool imu_corrected
string reason
# Arrays have equal length N; coefficients are [dz_dx, dz_dy, z_at_origin]
# for each view, flattened to 3*N. All views use M1 or all use nominal M0.
string[] views
std_msgs/Header[] image_headers
string[] calibration_ids
float64[] plane_coefficients
