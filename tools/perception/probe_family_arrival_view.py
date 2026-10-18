#!/usr/bin/env python3
"""Launch the arrival-driven family renderer on a remapped debug topic.

The formerly experimental path is now the production implementation. Historical
10Hz comparisons can instantiate FamilyViewNode(arrival_driven=False) in a probe.
No inference, refitting or operational control is started by this tool.
"""
import rospy
from asmc_camera.nodes.family_view_node import FamilyViewNode


if __name__ == '__main__':
    rospy.init_node('family_arrival_trial')
    FamilyViewNode()
    rospy.spin()
