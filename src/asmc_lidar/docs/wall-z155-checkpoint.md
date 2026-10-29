# Wall filtering and Z threshold checkpoint

The filtering work was checkpointed from fad73f82068eee3869e91c92bc6bef4b2e713752 and rebased onto main af911727 for a Draft PR. This branch does not modify main directly or include the separate 1029_2 planning changes.
It preserves LiDAR filtering before the new steel RGB pose model, static map lock, and 8091 steel overlay.

## Launch

`roslaunch asmc_lidar lidar_wall_vehicle.launch`

This launch selects config/lidar_wall_vehicle.yaml. Generic lidar.launch alone does not select this profile. Do not run two LiDAR pipelines together.

## Included settings and behavior

- Camera point refinement is enabled for all three calibrated cameras.
- Vehicle history is enabled. Previously confirmed vehicles retain protection using current LiDAR support and ego motion; separation still requires supporting measurements.
- Tunnel cluster mean sensor Z threshold: enabled, min_sensor_z_m=-1.55. This is a sensor-coordinate cluster mean, not height above ground.
- Wall plane fitting tolerance: 0.15 m. Wall point removal distance: 0.25 m.
- Confirmed wall geometry takes precedence over camera projection protection.
- The HD-map gate enables wall processing from 30 m before tunnel entry through 10 m after exit, with route proximity, heading and hysteresis checks.
- The approach gate applies wall processing only, preserving normal ground and height filtering. It does not enable the full tunnel ROI/Z pipeline outside the tunnel.
- Wall history is preserved across gate transitions. Lower wall segments require current geometric support before removal.
- Camera association avoids reusing one camera box indiscriminately for neighboring observations.
- Earlier operating settings are also preserved: the default YAML near-ground distance changes from 0.30 to 0.23 m, and the common/tunnel minimum cluster point counts change from 5/10 to 3. These affect generic launch users as well; the dedicated profile carries the current tested settings.

## Map dependency

shadow_wall_gate.yaml embeds a route window derived from official route SHA256 50658991e607d9339d76e4cd6cb169dfc733ea53b93de2c3e222460bb497cc05 and the metadata gps_shadow_candidate interval 1877.744798966 to 1996.730791531 m. Update the route window and interval together if the official route changes.

## Evidence and limits

In a saved approach scene with 126 GPS-valid scans, large wall-associated observations (at least 9 m) decreased from 252 to zero; the examined foreground set retained all 110 points. This is evidence for that scene, not a general recall or safety guarantee.

Earlier dog-scene analysis recovered 23 of 47 target frames with Z -1.55; -1.65 did not add recoveries. The final 0.25 m wall band and Z -1.55 configuration still needs broader driving validation. The approach gate itself preserves the selected common-ground policy; the inherited YAML changes relative to main are listed above.
