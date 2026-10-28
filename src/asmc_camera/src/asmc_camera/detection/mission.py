"""Competition v1 mobility eligibility, not measured motion."""
GROUPS = {
    'vehicle': ('car', 'bus', 'truck'),
    'dynamic_obstacle': ('person', 'dog', 'deer', 'bicycle', 'stroller'),
    'static_obstacle': ('motorcycle', 'bollard', 'box', 'concrete_barrier',
                        'steel_barricade', 'traffic_drum', 'plastic_barrier', 'traffic_cone'),
}
CLASSES = ('person', 'dog', 'deer', 'bicycle', 'car', 'motorcycle', 'bus', 'truck',
           'bollard', 'box', 'concrete_barrier', 'steel_barricade', 'traffic_drum',
           'plastic_barrier', 'traffic_cone', 'stroller')
CLASSES_NO_CONE = tuple(c for c in CLASSES if c != 'traffic_cone')
GROUPS_NO_CONE = {group: tuple(c for c in names if c != 'traffic_cone') for group, names in GROUPS.items()}


def profile_classes(profile):
    if profile == 'mission16_one2one_nms':return CLASSES
    if profile == 'mission15_one2one_nms':return CLASSES_NO_CONE
    raise ValueError('unknown mission head profile: '+profile)

def group_of(name):
    return next((group for group, names in GROUPS.items() if name in names), None)
