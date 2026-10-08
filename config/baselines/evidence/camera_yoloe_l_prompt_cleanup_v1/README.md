# Prompt cleanup / side Ego CPU evidence

This is CPU geometry and fake-backend coverage, not GPU recall or operational acceptance.

- `side_ego_geometry.json`: train-only reference calibration, source SHA, train/selection visible-actor overlap cases.
- `side_mask_unit_tests.log`: 62 CPU detection tests, including per-view masks and failure isolation.
- `side_mask_launch_params.yaml`: ROS launch parsing with side masks explicitly enabled; no inference nodes started.

Original rich42/base27 runs remain under `camera_yoloe_l_first_v1`. New vocabulary reinference,
PyTorch/TensorRT consistency, close-object recall and throughput await simulator shutdown confirmation.
Source RGB remains local in datasets/artifacts. See the linked handoff document for the review gallery.
