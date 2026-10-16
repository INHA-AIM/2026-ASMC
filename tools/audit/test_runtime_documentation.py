"""Contract failures that previously allowed unusable command examples."""
import json
from pathlib import Path
import tempfile
import unittest
from tools.audit.check_runtime_documentation import launch_args, render, shell_commands, check_commands, check_labels


class RuntimeDocumentationTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        base = self.root/'src/example'
        (base/'launch').mkdir(parents=True)
        (base/'package.xml').write_text('<package><name>example</name></package>')
        self.launch = base/'launch/run.launch'
        self.launch.write_text('''<launch>
          <arg name="required"/><arg name="enabled" default="true"/>
          <arg name="empty" default=""/><arg name="internal" default="1"/>
          <include file="child.launch"><arg name="child_only" value="x"/></include>
        </launch>''')
        self.guide = self.root/'guide.md'
        self.config = {'command_guides':['guide.md'], 'launches':[
            {'title':'Example', 'path':'src/example/launch/run.launch', 'internal_args':['internal']}]}

    def test_include_values_are_not_public_arguments(self):
        self.assertNotIn('child_only', launch_args(self.launch))
        text = render(self.root, self.config)
        self.assertIn('필수', text)
        self.assertIn('(빈 문자열)', text)
        self.assertNotIn('`internal`', text)

    def test_missing_package_and_removed_launch_fail(self):
        self.guide.write_text('```bash\nroslaunch retired run.launch\nroslaunch example absent.launch\n```\n')
        errors,count = check_commands(self.root,self.config)
        self.assertEqual(count,2)
        self.assertEqual(len(errors),2)

    def test_multiline_argument_typo_fails(self):
        self.guide.write_text('```bash\nroslaunch example run.launch \\\n  enabledd:=false required:=yes\n```\n')
        errors,count = check_commands(self.root,self.config)
        self.assertEqual(count,1)
        self.assertIn('enabledd',errors[0])

    def test_valid_override_does_not_execute_substitution(self):
        self.guide.write_text('```bash\nroslaunch example run.launch required:="$(touch sentinel)" enabled:=false\n```\n')
        self.assertEqual(check_commands(self.root,self.config),([],1))
        self.assertFalse((self.root/'sentinel').exists())

    def test_prose_and_historical_text_are_not_executed(self):
        text='roslaunch missing outside.launch\n```text\nroslaunch missing historical.launch\n```\n'
        self.assertEqual(list(shell_commands(text)),[])

    def test_changed_default_changes_reference(self):
        before=render(self.root,self.config)
        self.launch.write_text(self.launch.read_text().replace('default="true"','default="false"'))
        self.assertNotEqual(before,render(self.root,self.config))

    def test_table_commands_and_required_arg_are_checked(self):
        self.guide.write_text('| Execute | `roslaunch example run.launch enabled:=false` |\n')
        errors,count=check_commands(self.root,self.config)
        self.assertEqual(count,1)
        self.assertIn('missing required arg',errors[0])

    def test_label_id_or_group_drift_fails(self):
        (self.root/'config/datasets').mkdir(parents=True)
        (self.root/'config/datasets/camera_object_labels_v2.json').write_text(
            json.dumps({'pilot_target_classes':['person','box']}))
        (self.root/'src/asmc_camera/config/camera').mkdir(parents=True)
        (self.root/'src/asmc_camera/config/camera/object_classes_mission16_v1.yaml').write_text(
            'class_groups: {groups: {dynamic_obstacle: [person], static_obstacle: [box]}}')
        (self.root/'docs/competition').mkdir(parents=True)
        table=self.root/'docs/competition/obstacle-assets-and-labels.md'
        table.write_text('| 0 | person | dynamic_obstacle | Person |\n| 1 | box | static_obstacle | Box |\n')
        self.assertEqual(check_labels(self.root),[])
        table.write_text(table.read_text().replace('static_obstacle','vehicle'))
        self.assertTrue(check_labels(self.root))

    def test_duplicate_declarations_fail(self):
        self.launch.write_text('<launch><arg name="x"/><arg name="x"/></launch>')
        with self.assertRaises(ValueError):
            launch_args(self.launch)


if __name__=='__main__':
    unittest.main()
