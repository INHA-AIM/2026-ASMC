# Container ROS sessions

Interactive shell profiles keep each checkout on its own master. The host-side installer previews by default:

```bash
python3 tools/runtime/install_container_profiles.py /path/to/profiles.json
python3 tools/runtime/install_container_profiles.py /path/to/profiles.json --apply --backup /path/to/new-backup
```

The JSON contains `containers`: each entry names `container`, verified `source_root`, `devel_setup`,
`master_port`, `planning_port`, `lidar_port`, and `lease_directory`; `extra_setup` is optional.
The actual `/root/ws` target must match. All master/HTTP ports must be unique.
Only `/root/.bashrc` and the installed `.asmc-*` shell/guard files change. Source worktrees,
Docker mounts/Config.Env, existing process environments and noninteractive test scripts remain unchanged.
Backups contain original bytes; reopen the terminal after installation. No node or master auto-starts.
To roll back, stop this session and restore the original `.bashrc` from its backup; unused helper files can stay.

`roscore` selects the session master port unless the user explicitly supplies a port.
`roslaunch` supplies `dashboard_port` for planner autonomy/dashboard and LiDAR view, honoring explicit overrides.
An occupied HTTP port is rejected before ROS registration. Graph-only inspection bypasses runtime guards.
An explicitly launched full bridge holds the existing two-alias physical lease and checks canonical UDP ports.
Ctrl-C is forwarded to its own child launch and releases the lease. Other owners are never terminated.

The three installed personal profiles are main11311/8088/8091, LOCAL11348/8090/8092,
and Localization11372/8089/8097. Automated Localization keeps private11371.
The private workspace record `.storage/manifests/container-standalone-20261029-r1/` owns exact bindings,
backups and the six-console/three-master smoke evidence. Those machine-specific files are not team presets.
A single physical simulator still supports one active bridge/controller; different masters do not clone the vehicle.
