import tempfile
from pathlib import Path
import shutil
import subprocess
import unittest

from tools.build.build_workspace import plan, build_directory


class CommandTest(unittest.TestCase):
    def test_modes_share_one_make_goal_and_optimization_flags(self):
        for args, expected in [([], 'asmc_runtime'), (['full'], 'asmc_full'),
                               (['all', 'ground_projection_test'], 'asmc_full;ground_projection_test')]:
            cmd = plan(args)
            self.assertEqual(cmd[:2], ['catkin_make', 'asmc_requested'])
            self.assertIn('-DASMC_BUILD_REQUESTS='+expected, cmd)
            self.assertIn('-DCMAKE_BUILD_TYPE=Release', cmd)
        self.assertIn('-DCATKIN_ENABLE_TESTING=ON', plan(['full']))

    def test_split_options_and_cmake_values_are_not_targets(self):
        cmd = plan(['asmc_planner_node', '-j', '2', '-l2', '-D', 'CMAKE_BUILD_TYPE=Debug', '-G', 'Unix Makefiles'])
        self.assertIn('-DASMC_BUILD_REQUESTS=asmc_planner_node', cmd)
        self.assertNotIn('-DCMAKE_BUILD_TYPE=Release', cmd)
        self.assertIn('Unix Makefiles', cmd)

    def test_custom_cache_launcher_and_ci_flags_are_preserved(self):
        cmd = plan(['one', '-DCMAKE_BUILD_TYPE=', '-DCMAKE_CXX_COMPILER_LAUNCHER=custom'], '/usr/bin/ccache')
        self.assertIn('-DCMAKE_C_COMPILER_LAUNCHER=/usr/bin/ccache', cmd)
        self.assertNotIn('-DCMAKE_CXX_COMPILER_LAUNCHER=/usr/bin/ccache', cmd)
        self.assertNotIn('-DCMAKE_BUILD_TYPE=Release', cmd)
        self.assertIn('-DCMAKE_CXX_COMPILER_LAUNCHER=', plan([], '/usr/bin/ccache', 'off'))
        with self.assertRaises(ValueError): plan([], None, 'on')

    def test_invalid_target_or_private_override_is_rejected(self):
        for args in [['foo;bar'], ['-DASMC_BUILD_REQUESTS=bogus'], ['-j'], ['--make-args', 'all']]:
            with self.assertRaises(ValueError): plan(args)

    def test_original_catkin_operations_remain_explicit(self):
        self.assertEqual(plan(['--catkin','--make-args','all']), ['catkin_make','--make-args','all'])
        self.assertEqual(plan(['install','-j2']), ['catkin_make','install','-j2'])
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(build_directory(['catkin_make','--build',d]), Path(d))


@unittest.skipUnless(shutil.which('cmake') and shutil.which('make') and shutil.which('c++'), 'CMake C++ fixture tools required')
class BuildGraphTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name); self.build = self.root/'build';self.package=self.root/'package';self.package.mkdir()
        self.module = Path(__file__).resolve().parents[2]/'cmake/AsmcBuildProfiles.cmake'
        # Same names as runtime packages; only a tiny real executable is needed.
        self.runtime = ['asmc_local_planner_node','asmc_local_controller_node','mpc_controller_node','asmc_control_selector_node','asmc_competition_speed_state_node',
                        'planning_vehicle_state_node','planning_objects_adapter','lidar_pipeline_node','vehicle_state_node',
                        'camera_bbox_lane_node','camera_bbox_lane_change_node','_lane_association',
                        'asmc_camera_generate_messages','asmc_hd_map_generate_messages','lane_boundary_adapter_generate_messages']
        (self.root/'main.cpp').write_text('int main(){return 0;}\n')
        (self.root/'common.cmake').write_text('file(APPEND "${CMAKE_CURRENT_LIST_DIR}/visits.txt" "visit\\n")\n')
        (self.root/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.10)
project(profile_fixture CXX)
option(CATKIN_ENABLE_TESTING "tests" ON)
add_custom_target(common_generate_messages COMMAND ${CMAKE_COMMAND} -P ${CMAKE_SOURCE_DIR}/common.cmake)
add_executable(asmc_planner_node main.cpp)
add_dependencies(asmc_planner_node common_generate_messages)
'''+'\n'.join('add_custom_target('+n+')' for n in self.runtime)+'''
add_custom_target(run_tests COMMAND ${CMAKE_COMMAND} -E false)
add_subdirectory(package)
include("'''+str(self.module)+'''")
''')
        (self.package/'CMakeLists.txt').write_text('''add_executable(excluded_test EXCLUDE_FROM_ALL ../main.cpp)
add_dependencies(excluded_test common_generate_messages)
''')

    def configure(self, *flags, okay=True):
        r = subprocess.run(['cmake','-S',str(self.root),'-B',str(self.build),*flags],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        if okay:self.assertEqual(r.returncode,0,r.stdout)
        else:self.assertNotEqual(r.returncode,0,r.stdout)
        return r.stdout

    def build_target(self):
        r = subprocess.run(['cmake','--build',str(self.build),'--target','asmc_requested','--','-j2'],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        self.assertEqual(r.returncode,0,r.stdout)

    def test_runtime_excludes_tests_full_builds_excluded_test_without_running_it(self):
        self.configure('-DASMC_BUILD_REQUESTS=asmc_runtime'); self.build_target()
        self.assertTrue((self.build/'asmc_planner_node').exists())
        self.assertFalse((self.build/'package/excluded_test').exists())
        self.configure('-DASMC_BUILD_REQUESTS=asmc_full'); self.build_target()
        self.assertTrue((self.build/'package/excluded_test').exists())
        self.assertEqual((self.root/'visits.txt').read_text().splitlines(), ['visit','visit'])

    def test_selected_targets_visit_common_dependency_once(self):
        self.configure('-DASMC_BUILD_REQUESTS=asmc_planner_node;excluded_test'); self.build_target()
        self.assertEqual((self.root/'visits.txt').read_text().splitlines(), ['visit'])

    def test_full_tests_disabled_and_unknown_target_fail_closed(self):
        self.assertIn('requires CATKIN_ENABLE_TESTING', self.configure('-DASMC_BUILD_REQUESTS=asmc_full','-DCATKIN_ENABLE_TESTING=OFF',okay=False))
        self.assertIn('Unknown/recursive', self.configure('-DASMC_BUILD_REQUESTS=typo',okay=False))

    def test_runtime_missing_native_fails_at_configuration(self):
        p=self.root/'CMakeLists.txt';original=p.read_text()
        for required in ('_lane_association','asmc_local_planner_node','asmc_local_controller_node'):
            with self.subTest(required=required):
                p.write_text(original.replace('add_custom_target('+required+')',''))
                self.assertIn(required,self.configure('-DASMC_BUILD_REQUESTS=asmc_runtime',okay=False))
