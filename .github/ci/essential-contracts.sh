#!/usr/bin/env bash
set -euo pipefail
python3 tools/ci/check_coverage.py
python3 -m unittest discover -s tools/ci -p 'test_*.py'
python3 -m unittest discover -s tools/build -p 'test_*.py'
python3 tools/audit/check_documentation_migration.py
python3 -m unittest tools/audit/test_documentation_migration.py
python3 tools/audit/check_runtime_documentation.py
python3 -m unittest tools/audit/test_runtime_documentation.py
python3 tools/bridge/render_morai_network_profiles.py --check
python3 tools/audit/validate_bridge_profiles.py
python3 -m unittest tools/audit/test_bridge_profiles.py
python3 tools/audit/validate_p1b_contract.py
python3 tools/audit/validate_perception_topic_policy.py
python3 tools/audit/validate_perception_runtime_acceptance.py
python3 -m unittest tools/audit/test_p1b_contract.py tools/audit/test_perception_topic_policy.py tools/audit/test_perception_runtime_acceptance.py
python3 tools/audit/check_lidar_canonicalization.py
PYTHONPATH=src/data_collection/src python3 -m unittest discover -s src/data_collection/test -p 'test_storage_paths.py'
PYTHONPATH=src/data_collection/src python3 -m unittest discover -s src/data_collection/test -p 'test_data_locations.py'
python3 -m unittest tools/audit/test_storage_launchers.py tools/audit/test_data_location_preflight.py
python3 src/planning_inputs/test/runtime_launch_contract_test.py
python3 src/planning_inputs/test/object_filter_contract_test.py
python3 -m unittest tools/audit/test_git_history_assets.py
python3 -m unittest tools/perception/test_object_engine_selection.py
python3 src/asmc_planner/tests/behavior/dashboard_startup_test.py
python3 tools/runtime/test_standalone.py
node src/asmc_planner/tests/behavior/dashboard_motion_test.js
node src/asmc_planner/tests/behavior/dashboard_shadow_test.js
python3 -m compileall -q src tools/ci tools/scen_runner/scen_runner

PYTHONPATH=tools/scen_runner python3 -m unittest discover -s tools/scen_runner/test/unit -p 'test_trial_monitor.py'
