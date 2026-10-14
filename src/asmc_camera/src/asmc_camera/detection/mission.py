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

def group_of(name):
    return next((group for group, names in GROUPS.items() if name in names), None)
