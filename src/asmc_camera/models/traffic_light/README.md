# Traffic-light model artifact

`best.pt` is the deployed ResNet18 checkpoint consumed by
`asmc_camera/traffic_light_node.py`.

- SHA-256: `70ddaf222e7bc6ff2b4748c4b07c1b24dec72a2671f39bc18e0b243ddd93c0d5`
- model input size: 384
- output topics: `/perception/traffic_light/detection`, `/traffic_light/state`,
  `/traffic_light/detail`
- frozen A/B evidence: `config/baselines/perception_traffic_light_parity_v1.yaml`

Do not overwrite this file in place. A retrained model needs a new artifact
name or version, checksum, representative-bag evaluation, and closed-loop
acceptance update before the default launch is changed.
