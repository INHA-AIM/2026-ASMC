"""Shared synthetic lane geometry for BBOX classifier and controller tests."""
from asmc_msgs.msg import LaneBoundaryFamily, LaneBoundaryFamilyMember
from geometry_msgs.msg import Point32

HASH = "691b998247c960bba06d84513b217709a55f8744530978e32d9028289e217070"

def family(curve=0.0, shift=0.0, gap=False, inferred=False, stop_x=15.0):
    msg = LaneBoundaryFamily()
    msg.header.frame_id = "base_link"
    msg.valid = True
    msg.expected_boundary_count = 3
    msg.operating_domain = "shadow_two_way_v1"
    msg.algorithm_id = "twinlite_v6_shadow_boundary_family_candidate_v3"
    msg.representation = "shared_parametric_bspline_normal_offsets_v2"
    msg.config_sha256 = HASH
    msg.valid_x_min_m = 0.0
    msg.valid_x_max_m = stop_x
    msg.left_width_m = msg.right_width_m = 3.5
    msg.data_residual_p50_m = msg.inlier_residual_p50_m = 0.05
    msg.data_residual_p95_m = msg.inlier_residual_p95_m = 0.12
    msg.weighted_rms_m = 0.08
    for role, color, y in [(1, 1, 5.25), (2, 2, 1.75), (3, 1, -1.75)]:
        member = LaneBoundaryFamilyMember()
        member.role, member.color = role, color
        member.source_mask = 1
        member.overall_state = 1
        member.confidence = 0.9
        member.residual_valid = True
        member.residual_p50_m, member.residual_p95_m = 0.05, 0.12
        member.observed_length_m = stop_x
        member.support_state = []
        for i in range(int(stop_x * 4) + 1):
            x = i / 4.0
            member.polyline.append(Point32(x, y + shift + curve * x*x, -0.35))
            state = 0 if gap and 5.75 <= x <= 6.25 else (2 if inferred and x >= 6 else 1)
            member.support_state.append(state)
        msg.boundaries.append(member)
    return msg
