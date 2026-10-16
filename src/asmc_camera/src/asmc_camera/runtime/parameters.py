"""Explicit component configuration; independent of a worker's ROS node name."""


class Parameters:
    def __init__(self, namespace=None, getter=None):
        if getter is None:
            import rospy
            getter = rospy.get_param
        self.namespace, self.getter = namespace, getter

    def get(self, name, *default):
        if self.namespace and name.startswith('~'):
            name = self.namespace.rstrip('/') + '/' + name[1:]
        return self.getter(name, *default)


ROLES = {'yolo': 'objects', 'twinlite': 'lane', 'resnet18': 'traffic_light'}
LEGACY = {'objects': '/camera_object_detector', 'lane': '/camera_semantic_inference',
          'traffic_light': '/traffic_light_infer'}


def control_paths(model):
    role = ROLES[model]
    root = '/perception/camera/' + role
    return root + '/set_mode', root + '/status', LEGACY[role]


def fatal_cuda_error(error):
    message = str(error).lower()
    return any(s in message for s in ('cuda error', 'out of memory', 'illegal memory access',
                                      'device-side assert', 'cublas_status_execution_failed'))
